"""Prepare an isolated hardware copy of the v13 candidate and its Nav2 config."""
import argparse
from pathlib import Path
import shutil

import yaml


def hardware_mission(source):
    # ROS headers use ROS time. Mission durations continue to use monotonic time.
    replacements = {
        "self.marker_focus.observe(evidence,observer[:2],self.now(),self.p('map_frame'))":
            "self.marker_focus.observe(evidence,observer[:2],self.get_clock().now().nanoseconds/1e9,self.p('map_frame'))",
        "self.marker_focus.request_interrupt(int(evidence['id']),self.targets,self.now())":
            "self.marker_focus.request_interrupt(int(evidence['id']),self.targets,self.get_clock().now().nanoseconds/1e9)",
        "heading,choices,self.targets,self.now())":
            "heading,choices,self.targets,self.get_clock().now().nanoseconds/1e9)",
        "Time.from_msg(pose.header.stamp).nanoseconds/1e9,self.now())":
            "Time.from_msg(pose.header.stamp).nanoseconds/1e9,self.get_clock().now().nanoseconds/1e9)",
    }
    for old, new in replacements.items():
        expected = 2 if old.startswith('heading,') else 1
        if source.count(old) != expected:
            raise ValueError(f'Unexpected v13 source at {old}. Review the hardware adapter.')
        source = source.replace(old, new)
    return source


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--destination', type=Path, required=True)
    args = parser.parse_args()
    destination = args.destination / 'src' / 'osp_mission'
    if destination.exists():
        shutil.rmtree(destination)
    shutil.copytree(args.source, destination,
                    ignore=shutil.ignore_patterns('__pycache__', '.pytest_cache'))
    mission = destination / 'osp_mission' / 'mission.py'
    mission.write_text(hardware_mission(mission.read_text()))
    # These callback fixtures previously mocked only the mission-duration clock.
    # Add the ROS clock used by the hardware callback while retaining assertions.
    test_path = destination / 'test' / 'test_marker_focus.py'
    tests = test_path.read_text()
    old_clock = 'now=lambda:10.2,'
    if tests.count(old_clock) != 2:
        raise ValueError('Unexpected marker-focus clock fixtures.')
    tests = tests.replace(old_clock, old_clock + '\n        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10_200_000_000)),')
    test_path.write_text(tests)
    # RPP is the controller used by v13's Gazebo experiments. The real robot uses
    # wall/ROS time through robot.launch.py's use_sim_time=false argument.
    config_path = destination / 'config' / 'nav2_gazebo.yaml'
    config = yaml.safe_load(config_path.read_text())
    (destination / 'config' / 'nav2_robot_policy.yaml').write_text(yaml.safe_dump(config))


if __name__ == '__main__':
    main()
