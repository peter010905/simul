"""Build an optional local CPU accelerator, preserving exact sampled geometry."""

import hashlib
import json
from pathlib import Path
import subprocess


root=Path(__file__).resolve().parents[1]
source=root/'src/osp_mission/osp_mission/raycast.cpp';output=source.with_name('_raycast.so')
command=['c++','-std=c++17','-O3','-ffp-contract=off','-fno-fast-math','-shared','-fPIC',str(source),'-o',str(output)]
subprocess.run(command,check=True)
(root/'training_robust/raycast_build.json').write_text(json.dumps(dict(command=command,
    source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),library_sha256=hashlib.sha256(output.read_bytes()).hexdigest()),indent=2))
print(output)
