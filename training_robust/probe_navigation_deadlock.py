"""Read-only comparison of local costmap footprint costs and live LiDAR."""
import argparse
import json
import math
from pathlib import Path
import time

import numpy as np


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def summarize(costmap, robot, ranges, angle_min, angle_increment, sensor):
    info=costmap.metadata
    grid=np.asarray(costmap.data,dtype=np.uint8).reshape(info.size_y,info.size_x)
    yy,xx=np.indices(grid.shape)
    ox,oy,oa=info.origin.position.x,info.origin.position.y,yaw(info.origin.orientation)
    gx,gy=(xx+.5)*info.resolution,(yy+.5)*info.resolution
    x=ox+math.cos(oa)*gx-math.sin(oa)*gy
    y=oy+math.sin(oa)*gx+math.cos(oa)*gy
    rx,ry,ra=robot
    bx=math.cos(ra)*(x-rx)+math.sin(ra)*(y-ry)
    by=-math.sin(ra)*(x-rx)+math.cos(ra)*(y-ry)
    footprint=bx*bx+by*by<=.189**2
    rear=(bx<0)&(bx>-.43)&(np.abs(by)<.23)
    angles=angle_min+np.arange(len(ranges))*angle_increment+sensor[2]
    good=np.isfinite(ranges)
    px=sensor[0]+ranges[good]*np.cos(angles[good])
    py=sensor[1]+ranges[good]*np.sin(angles[good])
    hits=(px<0)&(px>-.43)&(np.abs(py)<.23)
    return dict(costmap_frame=costmap.header.frame_id,
        costmap_cells=grid.size,footprint_max_cost=int(grid[footprint].max()) if footprint.any() else None,
        footprint_lethal_cells=int(np.count_nonzero(footprint&(grid==254))),
        rear_sweep_lethal_cells=int(np.count_nonzero(rear&(grid==254))),
        rear_sweep_unknown_cells=int(np.count_nonzero(rear&(grid==255))),
        scan_finite_beams=int(good.sum()),scan_total_beams=len(ranges),
        live_scan_rear_sweep_hits=int(np.count_nonzero(hits)),
        live_scan_min_hit_distance_from_base_m=float(np.hypot(px,py).min()) if len(px) else None,
        odom_robot=list(robot),base_sensor=list(sensor),costmap_origin=[ox,oy,oa],
        note='Costmap cells and raw scan are observed ROS inputs, not scenario ground truth')


def main():
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,ReliabilityPolicy,qos_profile_sensor_data
    from rclpy.time import Time
    from nav2_msgs.msg import Costmap
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from tf2_ros import Buffer,TransformListener
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain',type=int,required=True)
    parser.add_argument('--seconds',type=float,default=8)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    context=rclpy.Context();rclpy.init(context=context,domain_id=args.domain)
    node=Node('navigation_deadlock_readonly_probe',context=context)
    executor=SingleThreadedExecutor(context=context);executor.add_node(node)
    tf=Buffer(node=node);listener=TransformListener(tf,node)
    latest={};counts={}
    def receive(name,message):
        latest[name]=message;counts[name]=counts.get(name,0)+1
    node.create_subscription(Costmap,'local_costmap/costmap_raw',lambda m:receive('local_costmap',m),
        QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
    node.create_subscription(LaserScan,'scan',lambda m:receive('scan',m),qos_profile_sensor_data)
    node.create_subscription(String,'mission/status',lambda m:receive('mission_status',m),10)
    deadline=time.monotonic()+args.seconds
    try:
        while time.monotonic()<deadline:executor.spin_once(timeout_sec=.2)
        report=dict(domain=args.domain,received=counts,complete=False)
        if 'mission_status' in latest:report['mission_status']=json.loads(latest['mission_status'].data)
        if 'local_costmap' in latest and 'scan' in latest:
            costmap,scan=latest['local_costmap'],latest['scan']
            base=tf.lookup_transform(costmap.header.frame_id,'base_link',Time()).transform
            sensor=tf.lookup_transform('base_link',scan.header.frame_id,Time()).transform
            robot=(base.translation.x,base.translation.y,yaw(base.rotation))
            mounting=(sensor.translation.x,sensor.translation.y,yaw(sensor.rotation))
            ranges=np.asarray(scan.ranges,dtype=float)
            ranges[(ranges<=scan.range_min)|(ranges>=scan.range_max)]=np.inf
            report.update(summarize(costmap,robot,ranges,scan.angle_min,scan.angle_increment,mounting),complete=True)
            np.savez_compressed(args.output.with_suffix('.npz'),
                data=np.asarray(costmap.data).reshape(costmap.metadata.size_y,costmap.metadata.size_x),
                resolution=costmap.metadata.resolution,origin=report['costmap_origin'],robot=robot,
                ranges=ranges,angle_min=scan.angle_min,angle_increment=scan.angle_increment,sensor=mounting)
        args.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
        print(json.dumps(report,indent=2),flush=True)
    finally:
        executor.shutdown();node.destroy_node();rclpy.shutdown(context=context)


if __name__=='__main__':main()
