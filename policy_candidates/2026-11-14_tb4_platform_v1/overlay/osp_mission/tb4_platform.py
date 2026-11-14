"""Shared nominal TB4 Standard Jazzy interface; not a hardware calibration."""
import json
from pathlib import Path


def load_profile(package):
    return json.loads((Path(package)/'config/tb4_platform.json').read_text())


def configure_model(doc, profile, control_params):
    def set_text(parent, tag, value):
        for element in parent.getElementsByTagName(tag):
            if element.firstChild: element.firstChild.data=str(value)
    set_text(doc,'render_engine','ogre2')
    for element in doc.getElementsByTagName('parameters'):
        if element.firstChild and 'irobot_create_control' in element.firstChild.data:
            element.firstChild.data=str(control_params)
    # Keep upstream body/inertias/wheels and the encoder odom -> base_link TF.
    # No fitted wheel multipliers, simulator pose input or second odometry EKF.
    for sensor in doc.getElementsByTagName('sensor'):
        kind=sensor.getAttribute('type')
        if kind=='rgbd_camera':
            set_text(sensor,'update_rate',profile['camera_rate_hz'])
            set_text(sensor,'horizontal_fov',profile['camera_hfov_rad'])
            for img in sensor.getElementsByTagName('image'):
                set_text(img,'width',profile['camera_width'])
                set_text(img,'height',profile['camera_height'])
        elif sensor.getAttribute('name')=='rplidar':
            set_text(sensor,'update_rate',profile['lidar_rate_hz'])
            for horizontal in sensor.getElementsByTagName('horizontal'):
                set_text(horizontal,'samples',profile['lidar_samples'])
            for distance in sensor.getElementsByTagName('range'):
                set_text(distance,'min',profile['lidar_min_m'])
                set_text(distance,'max',profile['lidar_max_m'])
        elif kind=='imu':set_text(sensor,'update_rate',profile['imu_rate_hz'])
    return doc


def contract_errors(observed, profile):
    """Fail closed on absent, stale, conflicting or incompatible robot input."""
    errors=[]
    for key,minimum in [('odom',10.),('scan',3.),('rgb',10.),('imu',40.),('hazards',.1)]:
        sensor=observed.get(key,{})
        if sensor.get('count',0)<3 or not -.1 <= sensor.get('age_s',999) <= 2.5:
            errors.append(key+': missing/stale')
        elif sensor.get('rate_hz',0)<minimum:errors.append(key+': low acquisition rate')
    odom=observed.get('odom',{})
    if (odom.get('frame'),odom.get('child_frame'))!=(profile['odom_frame'],profile['base_frame']):
        errors.append('odom: unexpected frames')
    if observed.get('odom_publishers')!=1:errors.append('odom: require one publisher')
    if observed.get('odom_tf_authorities')!=1:errors.append('odom TF: require one authority')
    if observed.get('unexpected_tf_publishers'):errors.append('TF: unexpected publisher')
    if observed.get('velocity_types')!=[profile['velocity_type']] or not observed.get('velocity_subscribers'):
        errors.append('velocity: stamped consumer unavailable or conflicting type')
    rgb=observed.get('rgb',{});info=observed.get('camera_info',{})
    wanted=(profile['camera_width'],profile['camera_height'])
    if (rgb.get('width'),rgb.get('height'))!=wanted:errors.append('RGB: unexpected preview size')
    if (info.get('width'),info.get('height'))!=wanted or not info.get('valid_intrinsics'):
        errors.append('CameraInfo: invalid calibration')
    if not info.get('frame') or info.get('frame')!=rgb.get('frame'):errors.append('camera: frame mismatch')
    if not observed.get('sensor_tf_connected'):errors.append('sensor TF: disconnected')
    return errors
