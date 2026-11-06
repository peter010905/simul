"""Read-only shadow decoder: never publishes targets or camera evidence."""
import argparse
import hashlib
import inspect
import json
import os
from pathlib import Path
import time
from types import SimpleNamespace

import rclpy
from osp_mission.detector import Detector


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',required=True)
    parser.add_argument('--seconds',type=float,default=120.)
    parser.add_argument('--pixels',type=int,default=12)
    parser.add_argument('--stop-file',type=Path,help='Stop when this episode completion file appears; never span two layouts')
    args=parser.parse_args()
    root=Path(args.output);root.mkdir(parents=True,exist_ok=False)
    rclpy.init(args=['--ros-args','-r','__node:=shadow_low_pixel_detector',
                    '-r','targets:=audit/low_pixel_targets',
                    '-r','mission/processed_camera:=audit/processed_camera',
                    '-r','mission/detector_status:=audit/detector_status',
                    '-r','mission/marker_observations:=audit/marker_observations',
                    '-p','use_sim_time:=true','-p',f'min_marker_pixels:={args.pixels}'])
    node=Detector();observations=[];publications=[]
    # Even the remapped audit publishers are replaced with local recording.
    # The production mission receives no observation or heartbeat from here.
    node.processed_camera=SimpleNamespace(publish=lambda message:None)
    node.diagnostic_pub=SimpleNamespace(publish=lambda message:None)
    node.observation_pub=SimpleNamespace(get_subscription_count=lambda:1,
        publish=lambda m:observations.append(json.loads(m.data)))
    def capture(message):
        points=[dict(id=m.id,x=m.pose.position.x,y=m.pose.position.y)
                for m in message.markers if m.ns=='point']
        if points:publications.append(dict(stamp=node.get_clock().now().nanoseconds/1e9,points=points))
    node.pub=SimpleNamespace(publish=capture)
    start=time.monotonic()
    try:
        while time.monotonic()-start<args.seconds:
            if args.stop_file is not None and args.stop_file.exists():break
            rclpy.spin_once(node,timeout_sec=.1)
        report=dict(read_only=True,min_marker_pixels=args.pixels,wall_seconds=time.monotonic()-start,
                    ros_domain=os.environ.get('ROS_DOMAIN_ID'),
                    detector_source_sha256=hashlib.sha256(Path(inspect.getsourcefile(Detector)).read_bytes()).hexdigest(),
                    stop_reason='episode_completed' if args.stop_file is not None and args.stop_file.exists() else 'observation_window_ended',
                    diagnostics=node.diagnostics,confirmed_ids=[i for i,v in node.observations.items() if len(v)>=3],
                    observations=observations,provisional_publications=publications,
                    note='Shadow decoder observations are diagnostic only and do not count as mission discoveries')
        (root/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k not in ('observations','provisional_publications')},indent=2))
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
