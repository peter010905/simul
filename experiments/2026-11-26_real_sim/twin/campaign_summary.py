"""Aggregate campaign_report.py rows: per policy, and paired by seed.

Usage: python3 campaign_summary.py rows.jsonl [seed prefix ...]
"""
import collections
import json
import statistics
import sys


def main():
    rows = [json.loads(l) for l in open(sys.argv[1]) if l.startswith("{")]
    prefixes = sys.argv[2:]
    rows = [r for r in rows if r.get("ok") and (not prefixes or any(r["seed"].startswith(p) for p in prefixes))]
    by = collections.defaultdict(list)
    for r in rows:
        by[r["policy"]].append(r)
    print("policy  runs  all3   found/prints  median_time_all3  bumps  by class (found/total)")
    for pol in sorted(by):
        rs = by[pol]
        cls = collections.defaultdict(lambda: [0, 0])
        for r in rs:
            for p in r["prints"]:
                cls[p["cls"]][0] += p["found"]
                cls[p["cls"]][1] += 1
        times = [r["time"] for r in rs if r["success"]]
        print(f"{pol:8s} {len(rs):4d}  {sum(r['success'] for r in rs):2d}/{len(rs):<2d}  "
              f"{sum(r['found'] for r in rs):3d}/{3*len(rs):<3d}       "
              f"{statistics.median(times) if times else float('nan'):6.0f}          "
              f"{sum(r['bumps'] for r in rs):3d}   "
              + " ".join(f"{k}:{v[0]}/{v[1]}" for k, v in sorted(cls.items())))
    # paired comparison on seeds that both policies ran
    pols = sorted(by)
    seeds = collections.defaultdict(dict)
    for r in rows:
        seeds[r["seed"]][r["policy"]] = r
    for i, a in enumerate(pols):
        for b in pols[i + 1:]:
            common = [s for s, d in seeds.items() if a in d and b in d]
            if not common:
                continue
            fa = sum(seeds[s][a]["found"] for s in common)
            fb = sum(seeds[s][b]["found"] for s in common)
            print(f"paired {a} vs {b} on {len(common)} seeds: prints {fa} vs {fb}, "
                  f"all3 {sum(seeds[s][a]['success'] for s in common)} vs {sum(seeds[s][b]['success'] for s in common)}")


if __name__ == "__main__":
    main()
