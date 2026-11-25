"""Install an optional raw-scan acquisition profile in an isolated Gazebo stage."""

import argparse
import json
from pathlib import Path
import shutil

from hardware import replace


def prepare(parent, destination, signature):
    profile = json.loads(signature.read_text())
    if (
        len(profile["validation"]) < 2
        or not profile["missing"]
        and not profile["short"]
    ):
        raise ValueError("A held-out raw-scanner validation is required")
    if (
        max(v["valid_fraction_in_selected"] for v in profile["validation"].values())
        > 0.02
    ):
        raise ValueError("Selected rays are unreliable on held-out data")
    destination.mkdir()
    for n in ("src", "frozen-config"):
        shutil.copytree(
            parent / n,
            destination / n,
            ignore=shutil.ignore_patterns("__pycache__", "*.pyc"),
        )
    for n in (
        "real_sim_profile.json",
        "real_sim_manifest.json",
        "hardware_parity.json",
        "optics.json",
    ):
        shutil.copy2(parent / n, destination / n)
    shutil.copyfile(signature, destination / "scan_signature.json")
    pkg = destination / "src/osp_mission"
    shutil.copyfile(
        Path(__file__).with_name("scan_model.py"),
        pkg / "osp_mission/twin_scan_model.py",
    )
    replace(
        pkg / "setup.py",
        "'twin_camera_info =",
        "'twin_raw_scan = osp_mission.twin_scan_model:main','twin_camera_info =",
    )
    bridge = pkg / "launch/bridge.launch.py"
    replace(bridge, "             'scan')", "             'twin/render_scan')")
    replace(
        bridge,
        "    ld.add_action(lidar_bridge)",
        """    ld.add_action(lidar_bridge)
    ld.add_action(Node(package='osp_mission',executable='twin_raw_scan',parameters=[{'use_sim_time':True}]))""",
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    for k in ("parent", "destination", "signature"):
        p.add_argument(k, type=Path)
    prepare(**vars(p.parse_args()))
