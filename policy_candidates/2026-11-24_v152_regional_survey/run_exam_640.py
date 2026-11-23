"""Run the archived live-SLAM exam with detector input set to 640×480 at 5 FPS."""
import os
from pathlib import Path
import sys


def main():
    runner = Path(sys.argv.pop(1)).absolute()
    source = runner.read_text()
    for before, after in (
        ('camera=[480,480,5]', 'camera=[640,480,5]'),
        ("CAMERA_BENCH_WIDTH='480'", "CAMERA_BENCH_WIDTH='640'"),
    ):
        if source.count(before) != 1:
            raise ValueError(f'Unexpected archived runner: {before}')
        source = source.replace(before, after, 1)
    os.environ['OSP_SURVEY_DISTANT_HINTS'] = 'false'
    sys.argv[0] = str(runner)
    sys.path[0] = str(runner.parent)
    exec(compile(source, str(runner), 'exec'), {'__file__': str(runner), '__name__': '__main__'})


if __name__ == '__main__':
    main()
