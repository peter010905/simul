"""Score whether a ranked candidate contains the marker for closer inspection."""

import argparse
import json
from pathlib import Path


def covers(candidate, target, minimum_coverage, maximum_area_ratio):
    cx, cy, cw, ch = candidate
    tx, ty, tw, th = target
    intersection = (
        max(0, min(cx + cw, tx + tw) - max(cx, tx))
        * max(0, min(cy + ch, ty + th) - max(cy, ty))
    )
    area = tw * th
    return (
        area > 0
        and intersection >= minimum_coverage * area
        and cw * ch <= maximum_area_ratio * area
    )


def score(rows, model, minimum_coverage=0.8, maximum_area_ratio=6, limit=None, threshold=0):
    groups = {tag: {"found": 0, "total": 0} for tag in (
        "all", "oblique", "visible_blur", "oblique_and_blur"
    )}
    selected_candidates = 0
    negative_regions = 0
    touched_negative_regions = 0
    for row in rows:
        ranked = sorted(
            (hit for hit in row["models"][model]["hits"] if hit["score"] >= threshold),
            key=lambda hit: hit["score"],
            reverse=True,
        )[:limit]
        selected_candidates += len(ranked)
        boxes = [hit["box"] for hit in ranked]
        for target in row["positive"]:
            found = any(covers(box, target["box"], minimum_coverage, maximum_area_ratio)
                        for box in boxes)
            for tag, enabled in (
                ("all", True),
                ("oblique", target["oblique"]),
                ("visible_blur", target["visible_blur"]),
                ("oblique_and_blur", target["oblique"] and target["visible_blur"]),
            ):
                if enabled:
                    groups[tag]["found"] += found
                    groups[tag]["total"] += 1
        # Partial labels allow auditing known distractors, not frame precision.
        non_target_boxes = [
            box for box in boxes
            if not any(covers(box, target["box"], minimum_coverage, maximum_area_ratio)
                       for target in row["positive"])
        ]
        for distractor in row["negative"]:
            negative_regions += 1
            touched_negative_regions += any(
                covers(box, distractor, minimum_coverage, maximum_area_ratio)
                for box in non_target_boxes
            )
    return {
        "groups": groups,
        "selected_candidates": selected_candidates,
        "mean_candidates_per_frame": selected_candidates / len(rows),
        "reviewed_nonmarker_regions": negative_regions,
        "reviewed_nonmarker_regions_covered": touched_negative_regions,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--predictions", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    rows = [json.loads(line) for line in args.predictions.read_text().splitlines()]
    reference_file = Path(__file__).resolve().parent / "artifacts" / "real-replay.json"
    reference = json.loads(reference_file.read_text())
    thresholds = {name: model["threshold"] for name, model in reference["models"].items()}
    result = {
        "frames": len(rows),
        "minimum_marker_area_coverage": 0.8,
        "maximum_candidate_to_marker_area_ratio": 6,
        "yolo_inference_score_floor": 0.001,
        "cnn_inference_score_floor": 0,
        "models": {
            name: {
                "original_threshold": dict(score(rows, name, threshold=threshold),
                                           threshold=threshold),
                "top_k": {str(limit): score(rows, name, limit=limit) for limit in (1, 3, 5)},
                "top_5_area_sensitivity": {
                    str(ratio): score(rows, name, maximum_area_ratio=ratio, limit=5)
                    for ratio in (4, 8)
                },
            }
            for name, threshold in thresholds.items()
        },
    }
    args.output.write_text(json.dumps(result, indent=2) + "\n")


if __name__ == "__main__":
    main()
