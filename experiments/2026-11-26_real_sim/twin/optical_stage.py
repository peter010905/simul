"""Add measured image formation and independently recorded camera metadata to a fresh sim stage."""

import argparse
import json
from pathlib import Path
import shutil
import statistics

from hardware import replace
from persistent_scene import stream


def install(parent, dest, focal, run):
    if dest.exists():
        raise FileExistsError(dest)
    parity = json.loads((parent / "hardware_parity.json").read_text())
    if parity["run"] != run.name:
        raise ValueError("Reference camera run differs from stage policy")
    rows = json.loads(focal.read_text())
    # Freeze using the other recordings, including neither the reference nor final 5 m validation run.
    held = {run.name, sorted(rows)[-1]}
    train = [v["focal_quantiles"][1] for k, v in rows.items() if k not in held and v]
    estimate = statistics.median(train)
    _, m = next(stream(run, ["/oakd/rgb/preview/camera_info"]))
    params = dict(
        width=m.width,
        height=m.height,
        k=list(m.k),
        r=list(m.r),
        p=list(m.p),
        d=list(m.d),
        distortion_model=m.distortion_model,
        frame=m.header.frame_id,
        render_fx=estimate,
        render_fy=m.k[4],
        heldout=sorted(held),
    )
    dest.mkdir()
    for n in ("src", "frozen-config"):
        shutil.copytree(
            parent / n, dest / n, ignore=shutil.ignore_patterns("__pycache__", "*.pyc")
        )
    for n in (
        "real_sim_profile.json",
        "real_sim_manifest.json",
        "hardware_parity.json",
    ):
        shutil.copy2(parent / n, dest / n)
    (dest / "optics.json").write_text(json.dumps(params, indent=2))
    pkg = dest / "src/osp_mission"
    shutil.copyfile(
        Path(__file__).with_name("oak_model.py"), pkg / "osp_mission/twin_oak_model.py"
    )
    sensor = pkg / "osp_mission/sensor_model.py"
    replace(
        sensor,
        "        result = apply(result, json.loads(Path(path).read_text()))",
        """        result = apply(result, json.loads(Path(path).read_text()))
        from osp_mission.twin_oak_model import configure
        configure(result, json.loads(Path(path).with_name('optics.json').read_text()))""",
    )
    replace(
        pkg / "setup.py",
        "'calibrated_wheel_twist =",
        "'twin_camera_info = osp_mission.twin_oak_model:main','calibrated_wheel_twist =",
    )
    bridge = pkg / "launch/bridge.launch.py"
    replace(bridge, "'oakd/rgb/preview/camera_info')", "'twin/render_camera_info')")
    replace(
        bridge,
        "    ld.add_action(oakd_camera_bridge)",
        """    ld.add_action(oakd_camera_bridge)
    ld.add_action(Node(package='osp_mission',executable='twin_camera_info',parameters=[{'use_sim_time':True}]))""",
    )
    print(json.dumps(params))


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    for key in ("parent", "dest", "focal", "run"):
        p.add_argument(key, type=Path)
    install(**vars(p.parse_args()))
