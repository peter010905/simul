"""Replay the two held-out physical runs without changing the robot policy."""
import argparse
import csv
import json
from pathlib import Path
from time import perf_counter_ns

import cv2
import numpy as np
import torch

from patch_cnn import MarkerPatchCNN, crop, normalize


SHORT_POSITIVES = {84, 86, 104, 118}
SHORT_NEGATIVES = {
    35, 36, 37, 42, 43, 44, 45, 46, 47, 82, 83, 85, 87, 88, 91, 94, 96,
    102, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 119, 120,
}
EARLY_PARTIAL_CANDIDATES = {554, 555}


def replay(root, rows, predictions, positive_indices, negative_indices, model):
    results, timings = [], []
    for row in rows:
        index = row['index']
        if row['quad'] is None:
            continue
        gray = cv2.imread(str(root / row['frame_file']), cv2.IMREAD_GRAYSCALE)
        if gray is None:
            raise OSError(root / row['frame_file'])
        start = perf_counter_ns()
        patch = crop(gray, row['quad'])
        score = model(torch.from_numpy(normalize(patch[None]))).sigmoid().item()
        timings.append((perf_counter_ns() - start) / 1e6)
        label = 'marker' if index in positive_indices else ''
        if index in negative_indices:
            if label:
                raise ValueError(f'Conflicting ground truth for candidate {index}')
            label = 'non-marker'
        results.append(dict(
            index=index, time_s=row['t'], short_px=row['short'], group=row['group'],
            structure=bool(predictions.get(index, False)), cnn_score=score, label=label,
        ))
    return results, timings[10:]


def evaluate(rows, accepts):
    detected = [r for r in rows if r['label'] == 'marker' and accepts(r)]
    false = [r for r in rows if r['label'] == 'non-marker' and accepts(r)]
    early = min(detected, key=lambda r: r['time_s'], default=None)
    return dict(
        true=len(detected), false=len(false),
        first_s=early['time_s'] if early else None,
        first_index=early['index'] if early else None,
        accepted_6_to_10=sum(6 <= r['short_px'] <= 10 for r in detected),
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('gallery', 'short_run', 'labels', 'model_dir'):
        parser.add_argument(name, type=Path)
    args = parser.parse_args()
    labels = json.loads(args.labels.read_text())
    document = (args.gallery / 'gallery-data.js').read_text().strip()
    gallery = json.loads(document.removeprefix('window.candidateData=').removesuffix(';'))
    rows_640 = gallery['640px']['rows']
    with (args.gallery / 'rectangle-results.csv').open() as stream:
        confirmed_42 = {
            int(row['index']) for row in csv.DictReader(stream)
            if row['run'] == '640px' and row['logged_overlap_ids'] == '42'
        }
    with (args.gallery / 'original-filter-results.csv').open() as stream:
        structure_640 = {
            int(r['index']): r['original_filter'] == 'True'
            for r in csv.DictReader(stream) if r['run'] == '640px'
        }
    rows_250 = json.loads((args.short_run / 'data.json').read_text())['250px-holdout']['rows']
    structure_250 = {
        int(k): v for k, v in json.loads((args.short_run / 'structure.json').read_text()).items()
    }
    torch.set_num_threads(1)
    cv2.setNumThreads(1)
    model = MarkerPatchCNN().eval()
    model.load_state_dict(torch.load(args.model_dir / 'marker-patch-cnn.pt', weights_only=True))
    with torch.inference_mode():
        out_640, time_640 = replay(
            args.gallery, rows_640, structure_640, confirmed_42 | EARLY_PARTIAL_CANDIDATES,
            set(labels['holdout_640_negative_indices']), model,
        )
        out_250, time_250 = replay(
            args.short_run, rows_250, structure_250, SHORT_POSITIVES, SHORT_NEGATIVES, model,
        )

    all_results = out_640 + out_250
    new_false_scores = [
        r['cnn_score'] for r in all_results if r['label'] == 'non-marker' and not r['structure']
    ]
    cutoff = float(np.nextafter(max(new_false_scores), np.inf))
    report = dict(
        matched_threshold_on_reviewed_holdouts=cutoff,
        mac_cpu_ms=dict(median=float(np.median(time_640 + time_250)),
                        p95=float(np.percentile(time_640 + time_250, 95))),
    )
    methods = {
        'structure': lambda r: r['structure'],
        'cnn_at_half': lambda r: r['cnn_score'] >= .5,
        'structure_or_cnn_at_matched_false_count': lambda r: (
            r['structure'] or r['cnn_score'] >= cutoff
        ),
    }
    with (args.model_dir / 'scores.csv').open('w', newline='') as stream:
        scores = csv.DictWriter(stream, ['run', *out_640[0]])
        scores.writeheader()
        for key, rows in [('640px_ID42', out_640), ('250px_ID0', out_250)]:
            scores.writerows(dict(run=key, **row) for row in rows)
            report[key] = dict(
                candidates=len(rows),
                reviewed_markers=sum(r['label'] == 'marker' for r in rows),
                reviewed_non_markers=sum(r['label'] == 'non-marker' for r in rows),
                metrics={name: evaluate(rows, rule) for name, rule in methods.items()},
            )
    (args.model_dir / 'evaluation.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
