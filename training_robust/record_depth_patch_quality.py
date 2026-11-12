"""Passive registered-depth diagnostics; no policy, commands, or ground truth.

Nearest recorded depth is an independent measurement. This recorder cannot
establish which depth image, or fallback range source, the detector used.
"""
import argparse
from collections import deque
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from continual_native_curriculum import save
from record_motion_calibration import owner_alive


def patch_quality(message, uv):
    encoding = message.encoding
    if encoding not in ('16UC1', '32FC1'):
        return dict(available=False, reason='unsupported_depth_encoding', encoding=encoding)
    dtype = np.dtype(('>' if message.is_bigendian else '<') + ('u2' if encoding == '16UC1' else 'f4'))
    if (message.step < message.width*dtype.itemsize
            or len(message.data) < message.height*message.step):
        return dict(available=False, reason='invalid_depth_stride_or_buffer')
    u,v = map(int,uv)
    if not (0 <= u < message.width and 0 <= v < message.height):
        return dict(available=False, reason='marker_center_outside_depth')
    values = np.ndarray((message.height,message.width),dtype=dtype,
        buffer=message.data,strides=(message.step,dtype.itemsize))
    patch = values[max(0,v-2):v+3,max(0,u-2):u+3].astype(float)
    if encoding == '16UC1': patch *= .001
    valid = patch[np.isfinite(patch)&(patch>.15)&(patch<8.)]
    result = dict(available=True,encoding=encoding,depth_width=message.width,
        depth_height=message.height,patch_values=int(patch.size),valid_values=int(valid.size))
    if valid.size:
        median = float(np.median(valid))
        result.update(median_m=median,median_absolute_deviation_m=float(np.median(abs(valid-median))),
            p90_minus_p10_m=float(np.quantile(valid,.9)-np.quantile(valid,.1)))
    return result


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain',type=int,required=True)
    parser.add_argument('--native-pid',type=int,required=True)
    parser.add_argument('--native-output',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    if not owner_alive(args.native_pid,args.native_output):
        raise RuntimeError('Require an actual bound native benchmark process')
    context=rclpy.Context(); rclpy.init(context=context,domain_id=args.domain)
    node=Node('read_only_depth_patch_quality',context=context)
    executor=SingleThreadedExecutor(context=context);executor.add_node(node)
    frames=deque(maxlen=8);pending=deque(maxlen=128)
    meta=dict(complete=False,domain=args.domain,native_pid=args.native_pid,
        native_output=str(args.native_output),observations=0,matched_depth=0,
        recorder_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        source='Independent nearest registered-depth patch at published marker center',
        detector_range_provenance_known=False,ground_truth_available=False,robot_commands_published=False)
    started=flushed=time.monotonic()
    def stamp(header):return header.stamp.sec+header.stamp.nanosec/1e9
    node.create_subscription(Image,'oakd/rgb/preview/depth',
        lambda message:frames.append((stamp(message.header),message)),qos_profile_sensor_data)
    def observation(message):
        try:
            value=json.loads(message.data)
            if value.get('uv') is not None:pending.append((time.monotonic(),value))
        except (ValueError,KeyError,TypeError):meta['malformed_observations']=meta.get('malformed_observations',0)+1
    node.create_subscription(String,'mission/marker_observations',observation,10)
    with args.output.open('x') as stream:
        def write(value):
            row={k:value.get(k) for k in ('id','stamp','uv','min_edge_pixels','camera_point','lidar_camera_point')}
            row['detector_range_provenance_known']=False
            if frames:
                when,message=min(frames,key=lambda pair:abs(pair[0]-value['stamp']))
                row['depth_stamp_sim_s']=when;row['depth_minus_observation_sim_s']=when-value['stamp']
                if abs(when-value['stamp'])<=.15+1e-9:
                    row['depth_patch']=patch_quality(message,value['uv'])
                    meta['matched_depth']+=int(row['depth_patch'].get('available',False))
                else:row['depth_patch']=dict(available=False,reason='no_depth_within_0_15_sim_s')
            else:row['depth_patch']=dict(available=False,reason='no_recorded_depth')
            stream.write(json.dumps(row,allow_nan=False)+'\n');meta['observations']+=1
        try:
            while time.monotonic()-started<5500:
                executor.spin_once(timeout_sec=.1)
                while pending and time.monotonic()-pending[0][0]>=.3:write(pending.popleft()[1])
                if time.monotonic()-flushed<1.:continue
                stream.flush();flushed=time.monotonic()
                if not owner_alive(args.native_pid,args.native_output):
                    meta.update(complete=True,stop_reason='native_process_ended');break
                save(args.output.with_suffix('.json'),meta)
            else:meta['stop_reason']='recorder_wall_limit_native_not_restarted'
        finally:
            while pending:write(pending.popleft()[1])
            stream.flush();meta['wall_seconds']=time.monotonic()-started
            save(args.output.with_suffix('.json'),meta)
            executor.shutdown();node.destroy_node();rclpy.shutdown(context=context)


if __name__=='__main__':main()
