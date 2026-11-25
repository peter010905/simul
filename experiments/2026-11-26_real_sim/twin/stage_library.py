"""Stage the logged policies for an explicitly common reconstructed start."""

import argparse
import hashlib
import json
import subprocess
from pathlib import Path
from hardware import prepare


def config_evidence(run, workspace):
    """Record which files still match the physical archive after simulator adaptation."""
    result = {}
    for recorded in sorted((run / "config").glob("*.yaml")):
        path = next(
            (
                p
                for p in (
                    workspace / "frozen-config" / recorded.name,
                    workspace / "src/osp_mission/config" / recorded.name,
                )
                if p.is_file()
            ),
            None,
        )
        result[recorded.name] = bool(
            path and path.read_bytes() == recorded.read_bytes()
        )
    meta = json.loads((run / "run.json").read_text())
    weights = workspace / "frozen-config/policy.json"
    return dict(
        weights_match=hashlib.sha256(weights.read_bytes()).hexdigest()
        == meta["policy_sha256"],
        source_commit_recorded=bool(meta.get("git_commit")),
        yaml_matches=result,
        initial_odometry_adapted=True,
    )


def stage(args):
    poses = args.logs / args.reference
    stages = {}
    for a in json.loads(args.audit.read_text()):
        run = args.logs / a["run"]
        ws = args.destination / ("stage-" + a["run"])
        if run == poses and args.primary:
            ws = args.primary
        if not ws.exists():
            profile = (
                args.profiles / f"measured-slam{int(a['slam']['max_laser_range'])}.json"
            )
            prepare(run, ws, profile, poses)
            subprocess.run(
                [
                    "colcon",
                    "build",
                    "--symlink-install",
                    "--packages-select",
                    "osp_mission",
                ],
                cwd=ws,
                check=True,
            )
        proof = json.loads((ws / "hardware_parity.json").read_text())
        if (
            proof["run"] != run.name
            or proof.get("pose_run", proof["run"]) != poses.name
            or not (ws / "install/setup.bash").is_file()
        ):
            raise ValueError("Stage identity does not match the requested pairing")
        stages[run.name] = dict(
            workspace=str(ws),
            parent=a["workspace"],
            slam=a["slam"]["max_laser_range"],
            archive_evidence=config_evidence(run, ws),
        )
    if args.optical:
        optical = args.optical.resolve()
        identity = json.loads((optical / "hardware_parity.json").read_text())
        if (
            identity["run"] != poses.name
            or not (optical / "optics.json").exists()
            or not (optical / "install/setup.bash").exists()
        ):
            raise ValueError("Optical workspace must match the reference policy")
        stages[poses.name]["optical_workspace"] = str(optical)
    if args.scanner:
        scanner = args.scanner.resolve()
        if (
            not args.optical
            or json.loads((scanner / "hardware_parity.json").read_text())["run"]
            != poses.name
            or not (scanner / "scan_signature.json").is_file()
            or not (scanner / "install/setup.bash").exists()
        ):
            raise ValueError("Scanner workspace must match the calibrated policy")
        stages[poses.name]["scanner_workspace"] = str(scanner)
    args.output.write_text(json.dumps(stages, indent=2))


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    for k in ("audit", "logs", "profiles", "destination", "output"):
        p.add_argument(k, type=Path)
    p.add_argument("--reference", required=True)
    p.add_argument("--primary", type=Path)
    p.add_argument("--optical", type=Path)
    p.add_argument("--scanner", type=Path)
    stage(p.parse_args())
