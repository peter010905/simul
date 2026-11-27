"""Local costmap coordinates with odom's XY axes and a floor-relative height."""
import math

from geometry_msgs.msg import TransformStamped


FLOOR_FRAME = 'osp_local_floor'


def configure_local_floor(nav):
    local = nav['local_costmap']['local_costmap']['ros__parameters']
    behavior = nav['behavior_server']['ros__parameters']
    if local['global_frame'] != 'odom' or behavior['local_frame'] != 'odom':
        raise ValueError('The local floor profile requires an odom-based rolling costmap')
    local['global_frame'] = behavior['local_frame'] = FLOOR_FRAME


def floor_transform(odom, parent, base):
    """Copy the base height at the acquisition stamp, preserving planar odometry."""
    height = odom.pose.pose.position.z
    if odom.header.frame_id != parent or odom.child_frame_id != base or not math.isfinite(height):
        return None
    tf = TransformStamped()
    tf.header = odom.header
    tf.child_frame_id = FLOOR_FRAME
    tf.transform.translation.z = height
    tf.transform.rotation.w = 1.0
    return tf
