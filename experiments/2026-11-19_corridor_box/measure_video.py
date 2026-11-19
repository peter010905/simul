"""Record the full robot RGB stream with simulation timestamps and measured heading."""
import csv
import math
from pathlib import Path
import sys

import cv2
from cv_bridge import CvBridge
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.parameter import Parameter

BASE = Path(__file__).resolve().parent.parent / '2026-11-19_pair_7_103'
sys.path.insert(0, str(BASE))
from measure import Trial, main, seconds


class CameraTrial(Trial):
    def __init__(self, *args):
        super().__init__(*args)
        self.bridge = CvBridge()
        self.frames = self.path/'camera'
        self.frames.mkdir()
        self.count = 0
        self.finished = False
        self.frame_log = (self.path/'frames.csv').open('w')
        self.frames_csv = csv.writer(self.frame_log)
        self.frames_csv.writerow(['frame', 'elapsed_sim_s'])
        self.map_rate_client = None
        self.map_rate_request = None
        if self.native and '-track-' in self.path.name:
            self.map_rate_client = self.node.create_client(SetParametersAtomically,
                '/global_costmap/global_costmap/set_parameters_atomically')
        self.heading_log = (self.path/'headings.csv').open('w')
        self.headings_csv = csv.writer(self.heading_log)
        self.headings_csv.writerow(['elapsed_sim_s','x','y','yaw_rad'])

    def prepare_start(self):
        if self.map_rate_client is None:
            return True
        if self.map_rate_request is None:
            if self.map_rate_client.service_is_ready():
                request = SetParametersAtomically.Request()
                request.parameters = [Parameter(name, value=4.).to_parameter_msg()
                                      for name in ('update_frequency', 'publish_frequency')]
                self.map_rate_request = self.map_rate_client.call_async(request)
            return False
        if not self.map_rate_request.done():
            return False
        result = self.map_rate_request.result().result
        if not result.successful:
            raise RuntimeError(f'Global costmap setup failed: {result.reason}')
        self.map_rate_client = None
        self.event('global_costmap_configured', update_hz=4., publish_hz=4.)
        return True

    def image(self, msg):
        if self.start_sim is None or self.finished:
            return
        elapsed = seconds(msg.header.stamp)-self.start_sim
        if elapsed < 0:
            return
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.count += 1
        if not cv2.imwrite(str(self.frames/f'{self.count:05d}.jpg'), image, [cv2.IMWRITE_JPEG_QUALITY, 95]):
            raise OSError('Camera frame could not be saved')
        self.frames_csv.writerow([self.count, elapsed])

    def truth(self, msg):
        super().truth(msg)
        if self.start_sim is None or self.finished:
            return
        for tf in msg.transforms:
            if tf.child_frame_id == 'turtlebot4':
                p,q = tf.transform.translation,tf.transform.rotation
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                self.headings_csv.writerow([seconds(tf.header.stamp)-self.start_sim,p.x,p.y,yaw])

    def run(self):
        try:
            result = super().run()
            if self.count < 2:
                raise RuntimeError('Camera recording missing')
            return dict(result, camera_frames=self.count)
        finally:
            self.finished = True
            self.frame_log.close()
            self.heading_log.close()


if __name__ == '__main__':
    main(CameraTrial, BASE/'pair.launch.py')
