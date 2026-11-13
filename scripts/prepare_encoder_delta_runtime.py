"""Stage encoder pose-increment velocity for the existing calibrated EKF input."""
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
    shutil.copy2(ROOT/'training_robust/policy_candidates/encoder_delta_twist.py',package/'osp_mission/encoder_delta_twist.py')
    path=package/'osp_mission/calibrated_wheel_twist.py';code=path.read_text()
    code=replace_once(code,'import json\n','import json\nfrom osp_mission.encoder_delta_twist import EncoderDelta\n')
    code=replace_once(code,'        self.accepted=0;self.rejected=0;self.maximum_rejected_speed=0.',
        "        self.declare_parameter('velocity_source','reported')\n"
        "        self.velocity_source=self.get_parameter('velocity_source').value\n"
        "        if self.velocity_source not in ('reported','pose_delta'):raise ValueError('Invalid encoder velocity source')\n"
        "        self.delta=EncoderDelta()\n"
        "        self.accepted=0;self.rejected=0;self.maximum_rejected_speed=0.")
    code=replace_once(code,'            calibrated=correct(message,self.scale)',
        "            if self.velocity_source=='pose_delta':\n"
        "                message=self.delta.update(message)\n"
        "                if message is None:return\n"
        "            calibrated=correct(message,self.scale)")
    code=replace_once(code,'            linear_scale=self.scale,maximum_calibrated_linear_speed=self.maximum,',
        '            velocity_source=self.velocity_source,linear_scale=self.scale,maximum_calibrated_linear_speed=self.maximum,')
    path.write_text(code)
    path=package/'launch/control.launch.py';code=path.read_text()
    code=replace_once(code,"'linear_scale':0.78,","'linear_scale':1.0,")
    code=replace_once(code,"'maximum_calibrated_linear_speed':0.75}]))",
        "'maximum_calibrated_linear_speed':0.0,'velocity_source':'pose_delta'}]))")
    path.write_text(code)
    shutil.copy2(ROOT/'scripts/test_encoder_delta_runtime.py',package/'test/test_encoder_delta_runtime.py')
    return destination


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('source','destination'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args();print(prepare(args.source.resolve(),args.destination.resolve()))
