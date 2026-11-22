"""Horizontal bearing model measured on the recorded OAK-D square preview."""
from types import SimpleNamespace


def preview_geometry(info):
    # The fit is for this stream and camera profile. ROS CameraInfo remains intact.
    if ((info.width,info.height)!=(250,250) or
        info.header.frame_id.lstrip('/')!='oakd_rgb_camera_optical_frame' or
        abs(info.k[0]-199.2553)>.5 or abs(info.k[2]-123.6739)>.5):
        return info
    k=list(info.k)
    k[0]*=1.79
    return SimpleNamespace(k=k)
