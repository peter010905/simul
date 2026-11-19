"""Install the native MCAP lossless storage preset in a separate simulation recorder."""
import argparse
from pathlib import Path


def install(source, destination):
    package = destination / "osp_project"
    package.mkdir(parents=True, exist_ok=True)
    for path in source.glob("*.py"):
        if path.name != "drive_evidence.py":
            target = package / path.name
            if target.is_symlink() and target.resolve() == path.resolve():
                continue
            target.symlink_to(path.resolve())
    text = (source / "drive_evidence.py").read_text()
    anchor = "max_bagfile_size=256 * 1024 * 1024)"
    if text.count(anchor) != 1:
        raise ValueError("Expected the pinned R32 MCAP writer")
    (package / "drive_evidence.py").write_text(
        text.replace(anchor, "storage_preset_profile='zstd_fast', " + anchor))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    args = parser.parse_args()
    install(args.source, args.destination)
