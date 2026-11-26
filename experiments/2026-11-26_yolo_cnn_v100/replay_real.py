"""Replay the four trained locators on locally reviewed physical frames."""

import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import time

import cv2
import numpy as np
import torch
from ultralytics import YOLO, __version__ as ultralytics_version

from candidate import CandidateCNN
from evaluate import infer_cnn, infer_yolo, iou, match


def annotations(region_labels, positive_pool, blur_indices, excluded_indices):
    by_file = defaultdict(lambda: {"positive": [], "negative": []})
    positive_index = 0
    known_blur_indices = set()
    for label_path in (region_labels, positive_pool):
        for row in json.loads(label_path.read_text()):
            file = Path(row["file"])
            frame = by_file[file]
            frame["source"] = row.get("source", "250px_reviewed_original")
            frame["elapsed_s"] = row["elapsed_s"]
            if row["label"] == 0:
                frame["negative"].append(row["box"])
                continue
            sample = positive_index
            positive_index += 1
            if sample in excluded_indices:
                continue
            quad = np.asarray(row["quad"])
            edges = np.linalg.norm(quad - np.roll(quad, 1, axis=0), axis=1)
            edge_ratio = float(edges.min() / edges.max())
            is_blurred = sample in blur_indices
            if is_blurred:
                known_blur_indices.add(sample)
            frame["positive"].append({
                "sample": sample,
                "box": row["box"],
                "id": row.get("id", row.get("expected_id")),
                "short_px": float(edges.min()),
                "edge_ratio": edge_ratio,
                "oblique": edge_ratio <= 0.65,
                "visible_blur": is_blurred,
            })
    if known_blur_indices != blur_indices:
        raise ValueError("Blur labels include an out-of-range positive index")
    return by_file


def summarize(rows, model, threshold):
    groups = defaultdict(lambda: {"gt": 0, "found": 0, "top1_found": 0})
    negative = {"reviewed_regions": 0, "regions_triggered": 0}
    latencies = []
    candidate_count = 0
    for row in rows:
        output = row["models"][model]
        boxes = [region["box"] for region in row["positive"]]
        above_threshold = [hit for hit in output["hits"] if hit["score"] >= threshold]
        found = set(match(above_threshold, boxes, threshold)["matched_gt"])
        top1 = sorted(output["hits"], key=lambda hit: hit["score"], reverse=True)[:1]
        top1_found = set(match(top1, boxes, 0)["matched_gt"])
        for index, region in enumerate(row["positive"]):
            tags = ["all", "source:" + row["source"], f"id:{region['id']}"]
            if region["oblique"]:
                tags.append("oblique")
            if region["visible_blur"]:
                tags.append("visible_blur")
            if region["oblique"] and region["visible_blur"]:
                tags.append("oblique_and_blur")
            if region["short_px"] < 10:
                tags.append("short_edge_under_10px")
            for tag in tags:
                stats = groups[tag]
                stats["gt"] += 1
                stats["found"] += index in found
                stats["top1_found"] += index in top1_found
        for box in row["negative"]:
            negative["reviewed_regions"] += 1
            # A detection on a different known marker is not a negative hit.
            negative["regions_triggered"] += any(
                iou(hit["box"], box) >= 0.5
                and all(iou(hit["box"], marker) < 0.5 for marker in boxes)
                for hit in above_threshold
            )
        candidate_count += len(above_threshold)
        latencies.append(output["ms"])
    return {
        "threshold": threshold,
        "groups": dict(groups),
        "negative": negative,
        "candidate_count": candidate_count,
        "median_ms": float(np.median(latencies[1:])),
        "p95_ms": float(np.percentile(latencies[1:], 95)),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--reviewed-regions", type=Path, required=True)
    parser.add_argument("--positive-pool", type=Path, required=True)
    parser.add_argument("--blur-labels", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--cnn-score-floor", type=float, default=0.05)
    args = parser.parse_args()
    if not 0 <= args.cnn_score_floor <= 1:
        parser.error("--cnn-score-floor must be in [0, 1]")
    args.output.mkdir(parents=True, exist_ok=True)
    cv2.setNumThreads(2)
    torch.set_num_threads(4)
    review = json.loads(args.blur_labels.read_text())
    blur_indices = set(review["blur_positive_indices"])
    excluded_indices = set(review["exclude_partial_duplicate_indices"])
    frames = annotations(args.reviewed_regions, args.positive_pool, blur_indices, excluded_indices)
    artifact_dir = Path(__file__).resolve().parent / "artifacts"
    comparison = json.loads((artifact_dir / "comparison.json").read_text())
    model_info = comparison["models"]
    models = {}
    for name, info in model_info.items():
        extension = ".pt" if name.startswith("yolo") else ".pth"
        weight = artifact_dir / "weights" / (name + extension)
        if hashlib.sha256(weight.read_bytes()).hexdigest() != info["weight_sha256"]:
            raise ValueError(f"Checkpoint SHA-256 mismatch: {name}")
        if name.startswith("yolo"):
            models[name] = YOLO(weight).to("cpu")
        else:
            model = CandidateCNN()
            model.load_state_dict(torch.load(weight, map_location="cpu", weights_only=True))
            models[name] = model.eval()

    rows = []
    with (args.output / "predictions.jsonl").open("w") as log:
        for index, (file, reference) in enumerate(frames.items()):
            image = cv2.imread(str(file))
            if image is None:
                raise FileNotFoundError(file)
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            row = dict(reference, file=str(file), shape=list(gray.shape), models={})
            for name, model in models.items():
                start = time.perf_counter()
                hits = (infer_yolo(model, image) if name.startswith("yolo")
                        else infer_cnn(model, gray, "cpu", score_floor=args.cnn_score_floor))
                row["models"][name] = {"hits": hits, "ms": (time.perf_counter() - start) * 1000}
            rows.append(row)
            log.write(json.dumps(row) + "\n")
            if index % 25 == 0:
                print(f"{index + 1}/{len(frames)} frames", flush=True)

    report = {
        "frames": len(rows),
        "device": "Mac CPU, 4 PyTorch threads, 2 OpenCV threads",
        "versions": {"torch": torch.__version__, "opencv": cv2.__version__,
                     "ultralytics": ultralytics_version},
        "resolution_frames": {f"{w}x{h}": sum(row["shape"] == [h, w] for row in rows)
                              for h, w in sorted({tuple(row["shape"]) for row in rows})},
        "iou": 0.5,
        "cnn_score_floor": args.cnn_score_floor,
        "all_frames_have_exhaustive_labels": False,
        "dataset_sha256": {name: hashlib.sha256(path.read_bytes()).hexdigest()
                           for name, path in (("reviewed_regions", args.reviewed_regions),
                                              ("positive_pool", args.positive_pool),
                                              ("blur_labels", args.blur_labels))},
        "models": {name: summarize(rows, name, info["test"]["threshold"])
                   for name, info in model_info.items()},
        "low_threshold_diagnostic": {
            name: {str(threshold): summarize(rows, name, threshold)
                   for threshold in (0.05, 0.10)}
            for name in models
        },
    }
    (args.output / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
    print(f"Completed {len(rows)} real frames", flush=True)


if __name__ == "__main__":
    main()
