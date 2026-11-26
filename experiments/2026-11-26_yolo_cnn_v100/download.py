"""Pin and fetch only the public dataset's images/COCO labels, never its code."""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import shutil
import tarfile
import time
import urllib.request

REVISION = '7506a466db549c3ca1e187299316196acca2819c'
BASE = 'https://huggingface.co/datasets/vanessabajaj/aruco-synthetic-detection/resolve/' + REVISION + '/'


def main():
    p = argparse.ArgumentParser(); p.add_argument('--output', type=Path, required=True)
    a = p.parse_args(); a.output.mkdir(parents=True, exist_ok=True)
    for name in ['README.md', 'annotations/instances_train2017.json', 'annotations/instances_val2017.json']:
        dest = a.output / name; dest.parent.mkdir(parents=True, exist_ok=True)
        if not dest.exists():
            with urllib.request.urlopen(BASE + name, timeout=60) as response:
                dest.write_bytes(response.read())
    expected = {}
    for split in ['train', 'val']:
        data = json.loads((a.output / f'annotations/instances_{split}2017.json').read_text())
        for image in data['images']:
            assert (image['width'], image['height']) == (640, 480)
            expected[f'{split}2017/' + image['file_name']] = image
    start = time.monotonic(); files = []; last = start
    # Streaming extraction avoids keeping a second 1.18 GB archive on disk.
    with urllib.request.urlopen(BASE + 'aruco_dataset.tar.gz', timeout=120) as response:
        with tarfile.open(fileobj=response, mode='r|gz') as archive:
            for member in archive:
                parts = PurePosixPath(member.name).parts
                if not member.isfile() or len(parts) < 2:
                    continue
                key = '/'.join(parts[-2:])
                if key not in expected:
                    continue
                dest = a.output / key; dest.parent.mkdir(parents=True, exist_ok=True)
                stream = archive.extractfile(member)
                with dest.open('wb') as f:
                    shutil.copyfileobj(stream, f)
                files.append(dict(file=key, bytes=dest.stat().st_size,
                                  sha256=hashlib.sha256(dest.read_bytes()).hexdigest()))
                if time.monotonic() - last > 15:
                    print('downloaded', len(files), '/', len(expected), 'images', flush=True)
                    last = time.monotonic()
    assert len(files) == len(expected), (len(files), len(expected))
    manifest = dict(dataset='vanessabajaj/aruco-synthetic-detection', revision=REVISION,
                    source_url=BASE, native_width=640, native_height=480,
                    count=len(files), bytes=sum(x['bytes'] for x in files),
                    elapsed_seconds=time.monotonic() - start, files=files)
    (a.output / 'download-manifest.json').write_text(json.dumps(manifest, indent=2))
    print({k: v for k, v in manifest.items() if k != 'files'}, flush=True)


if __name__ == '__main__':
    main()
