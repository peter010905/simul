"""Reject impossible simulator chassis-speed measurements before EKF fusion."""
import argparse
from pathlib import Path
import shutil

from prepare_approach_witness_runtime import replace_once

ROOT=Path(__file__).resolve().parents[1]


def prepare(source,destination):
    if destination.exists():raise FileExistsError('Use a new immutable candidate')
    package=destination/'src/osp_mission'
    shutil.copytree(source/'src/osp_mission',package,
        ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
    path=package/'osp_mission/calibrated_wheel_twist.py';code=path.read_text()
    code=replace_once(code,'from nav_msgs.msg import Odometry',
        'from nav_msgs.msg import Odometry\nfrom std_msgs.msg import String\nimport json')
    code=replace_once(code,'class CalibratedWheelTwist(Node):', '''def plausible(message, maximum):
    """Zero disables the guard; a rejected sample is never clipped or fabricated."""
    if not math.isfinite(maximum) or maximum<0:
        raise ValueError('Invalid calibrated speed limit')
    speed=math.hypot(message.twist.twist.linear.x,message.twist.twist.linear.y)
    return math.isfinite(speed) and (maximum==0 or speed<=maximum)


class CalibratedWheelTwist(Node):''')
    code=replace_once(code,"        self.output=self.create_publisher(TwistWithCovarianceStamped,'wheel/twist_calibrated',10)",
        "        self.declare_parameter('maximum_calibrated_linear_speed',0.)\n"
        "        self.maximum=float(self.get_parameter('maximum_calibrated_linear_speed').value)\n"
        "        if not math.isfinite(self.maximum) or self.maximum<0:raise ValueError('Invalid calibrated speed limit')\n"
        "        self.accepted=0;self.rejected=0;self.maximum_rejected_speed=0.\n"
        "        self.status=self.create_publisher(String,'mission/wheel_twist_status',10)\n"
        "        self.create_timer(5.,self.publish_status)\n"
        "        self.output=self.create_publisher(TwistWithCovarianceStamped,'wheel/twist_calibrated',10)")
    code=replace_once(code,"        try:self.output.publish(correct(message,self.scale))",'''        try:
            calibrated=correct(message,self.scale)
            if not plausible(calibrated,self.maximum):
                self.rejected+=1
                self.maximum_rejected_speed=max(self.maximum_rejected_speed,
                    math.hypot(calibrated.twist.twist.linear.x,calibrated.twist.twist.linear.y))
                self.get_logger().warning('Rejected implausible encoder chassis speed before EKF fusion',throttle_duration_sec=5.)
                return
            self.accepted+=1;self.output.publish(calibrated)''')
    code=replace_once(code,'\n\ndef main(args=None):', '''

    def publish_status(self):
        self.status.publish(String(data=json.dumps(dict(
            linear_scale=self.scale,maximum_calibrated_linear_speed=self.maximum,
            accepted=self.accepted,rejected=self.rejected,
            maximum_rejected_speed_m_s=self.maximum_rejected_speed,
            ground_truth_input=False))))


def main(args=None):''')
    path.write_text(code)
    path=package/'launch/control.launch.py';code=path.read_text()
    code=replace_once(code,"parameters=[{'use_sim_time':True,'linear_scale':0.78}]))",
        "parameters=[{'use_sim_time':True,'linear_scale':0.78,\n"
        "                     'maximum_calibrated_linear_speed':0.75}]))")
    path.write_text(code)
    path=package/'test/test_sim_state_estimation.py';code=path.read_text()
    code=replace_once(code,"assert publishers==['wheel/twist_calibrated']",
        "assert publishers==['mission/wheel_twist_status','wheel/twist_calibrated']")
    path.write_text(code)
    shutil.copy2(ROOT/'scripts/test_velocity_guard_runtime.py',package/'test/test_velocity_guard_runtime.py')
    return destination


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('source','destination'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args();print(prepare(args.source.resolve(),args.destination.resolve()))
