"""Emit a measured point or an explicitly range-free bearing at the image exposure."""
import json
import math
import numpy as np
from geometry_msgs.msg import PointStamped
from rclpy.time import Time
from std_msgs.msg import String
from tf2_geometry_msgs import do_transform_point
from osp_mission.hybrid_geometry import preview_geometry
from osp_mission.thorough_borders import borders
from osp_mission.wall_route import camera_pose,yaw as rotation_yaw
from osp_mission.r32_drive import grid_from_message

def publish(node,msg,image,decoded):
    cal=preview_geometry(node.info)
    if cal is node.info and (msg.width,msg.height)==(250,250):
        return
    outlines=borders(image)
    if not outlines:
        return
    stamp=Time.from_msg(msg.header.stamp)
    odom=node.tf.lookup_transform(node.p('odom_frame'),msg.header.frame_id,stamp)
    camera=camera_pose(odom.transform)
    if camera is None:
        return
    t=stamp.nanoseconds/1e9
    if decoded is None and getattr(node,'print_glance',None) is not None:
        glance=node.print_glance.update(t,t,camera,image,cal.k[0],cal.k[2],outlines,None)
        if glance:
            node.glance_pub.publish(String(data=json.dumps(dict(stamp=t,angle=math.radians(glance['yaw_delta_deg'])))))
    for u,v,size in sorted(outlines,key=lambda hit:-hit[2])[:4]:
        if decoded is not None and any(np.linalg.norm(c.reshape(-1,2).mean(axis=0)-[u,v])<.45*size for c in decoded):
            continue
        # Rotate a point on the optical ray through the full extrinsics (including pitch).
        ray=np.array([(u-cal.k[2])/cal.k[0],(v-cal.k[5])/cal.k[4],1.])
        pt=PointStamped()
        pt.header=msg.header
        pt.point.x,pt.point.y,pt.point.z=map(float,ray)
        ray_odom=do_transform_point(pt,odom).point
        origin=odom.transform.translation
        bearing=math.atan2(ray_odom.y-origin.y,ray_odom.x-origin.x)
        xyz=node.lidar_point(ray,msg.header.frame_id,stamp)
        point=None
        if xyz is not None:
            pt.point.x,pt.point.y,pt.point.z=map(float,xyz)
            position=do_transform_point(pt,odom).point
            point=[position.x,position.y]
        node.print_pub.publish(String(data=json.dumps(dict(stamp=t,bearing=bearing,point=point,
            observer=[origin.x,origin.y],uv=[u/msg.width,v/msg.height],size=size),allow_nan=False)))

def record_exposure(owner,msg,camera,endpoints,transform):
    stamp=Time.from_msg(msg.header.stamp)
    base=owner.tf.lookup_transform(owner.p('map_frame'),owner.p('base_frame'),stamp).transform
    a=rotation_yaw(base.rotation)
    dx,dy=camera[0]-base.translation.x,camera[1]-base.translation.y
    extrinsics=(math.cos(a)*dx+math.sin(a)*dy,-math.sin(a)*dx+math.cos(a)*dy,camera[2]-a)
    cal=preview_geometry(msg)
    owner.hybrid.schedule.observe(grid_from_message(owner.map),camera,endpoints,transform,
                                  cal.k[0],2*math.atan(msg.width/(2*cal.k[0])),extrinsics)
