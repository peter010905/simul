from types import SimpleNamespace
from unittest.mock import Mock
import numpy as np
import pytest
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo
import osp_mission.detector as module


@pytest.fixture
def detector():
    rclpy.init()
    node=module.Detector()
    identity=TransformStamped();identity.transform.rotation.w=1.
    node.tf=SimpleNamespace(lookup_transform=lambda *a,**kw:identity,
                            can_transform=lambda *a,**kw:True)
    try:yield node
    finally:node.destroy_node();rclpy.shutdown()


def frame(node,width=640,hardware=False):
    info=CameraInfo();info.width=info.height=width
    info.header.frame_id='oakd_rgb_camera_optical_frame' if hardware else 'camera'
    info.k=([199.2553,0.,123.6739,0.,199.2553,127.0769,0.,0.,1.] if hardware else
            [500.,0.,width/2,0.,500.,width/2,0.,0.,1.]);info.d=[0.]*5
    node.on_info(info)
    image=CvBridge().cv2_to_imgmsg(np.full((width,width),255,dtype=np.uint8),'mono8')
    image.header.frame_id=info.header.frame_id
    image.header.stamp=node.get_clock().now().to_msg()
    return image


def quad(x,y):return np.array([[[x,y],[x+40,y],[x+40,y+40],[x,y+40]]],dtype=np.float32)


def test_bad_first_id_does_not_hide_valid_neighbour_and_heartbeat_is_after_work(detector,monkeypatch):
    image=frame(detector)
    monkeypatch.setattr(module,'decode',lambda *args:([quad(40,100),quad(400,100)],np.array([[7],[103]])))
    detector.visual_sighting=lambda *a:None
    def ranging(ray,*args):
        if ray[0]<0:raise RuntimeError('missing TF for first tag')
        return ray*2
    detector.lidar_point=ranging
    after=[];detector.processed_camera=SimpleNamespace(publish=lambda msg:after.append(detector.diagnostics['accepted']))
    detector.on_rgb(image)
    assert detector.diagnostics['marker_errors']==1
    assert 7 not in detector.observations and len(detector.observations[103])==1
    assert after==[1]


def test_tf_readiness_retry_never_uses_blocking_lookup_in_input_callback(detector):
    image=frame(detector)
    detector.tf.can_transform=lambda *a,**kw:False
    detector.on_rgb=Mock()
    detector.enqueue_rgb(image);detector.drain_rgb()
    assert not detector.on_rgb.called and len(detector.exposure_queue.rows)==1
    detector.tf.can_transform=lambda *a,**kw:True
    detector.drain_rgb()
    detector.on_rgb.assert_called_once_with(image)
    assert not detector.exposure_queue.rows


def test_hardware_focal_fit_is_applied_once_to_all_sensor_rays(detector,monkeypatch):
    image=frame(detector,250,True)
    monkeypatch.setattr(module,'decode',lambda *args:([quad(40,100)],np.array([[7]])))
    rays=[];directions=[];heartbeat=[]
    detector.visual_range.observe=lambda ident,stamp,origin,direction,pixels:directions.append(direction.copy())
    detector.lidar_point=lambda ray,*args:(rays.append(ray.copy()) or ray*2)
    detector.processed_camera=SimpleNamespace(publish=heartbeat.append)
    detector.on_rgb(image)
    expected=np.array([(60.-123.6739)/(199.2553*1.79),(120.-127.0769)/199.2553,1.])
    np.testing.assert_allclose(rays[0],expected)
    np.testing.assert_allclose(directions[0],expected/np.linalg.norm(expected))
    assert heartbeat[0].k[0]==pytest.approx(199.2553*1.79)
    assert detector.info.k[0]==199.2553
