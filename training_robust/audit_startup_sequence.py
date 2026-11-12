"""Read-only native evidence that Nav2 first ran after SLAM readiness."""
import argparse
import json
from pathlib import Path
import re


def audit(episode):
    report_path=episode/'startup_guard.json'
    report=json.loads(report_path.read_text()) if report_path.exists() else {}
    text=(episode/'launch.log').read_text(errors='replace')
    nav_times=[float(value) for value in re.findall(
        r'\[controller_server-\d+\].*?\[(\d+\.\d+)\] \[controller_server\]',text)]
    ready=report.get('finished_wall_epoch_s')
    guard_ready=bool(report.get('success') and report.get('observed_slam_state')==3
        and report.get('map_ready') and report.get('reason')=='slam_active_map_tf_scan_ready')
    ordered=bool(guard_ready and ready is not None and nav_times and min(nav_times)>=ready)
    return dict(evidence_type='native_startup_report_and_wall_timestamped_ROS_log',
        episode=str(episode),readiness_report_passed=guard_ready,
        readiness_wall_epoch_s=ready,first_controller_log_wall_epoch_s=min(nav_times) if nav_times else None,
        nav2_started_after_readiness_verified=ordered,
        caveat='Startup ordering alone is not mission success or generalization evidence')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('episode',type=Path);args=parser.parse_args()
    print(json.dumps(audit(args.episode.resolve()),indent=2))
