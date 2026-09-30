#!/usr/bin/env python3
"""Plots slam_eval_demo's trajectory CSVs: a top-down (x vs z) view, plus
position error against ground truth over distance traveled -- sequence 04
is a near-straight ~400m drive, so the top-down view alone barely shows the
gap between methods that the error panel makes obvious.

Usage: python3 docs/plot_trajectories.py <trajectory_dir> <output.png>
Run slam_eval_demo with a 4th argument first to produce the CSVs:
  slam_eval_demo data/sequences/04 data/poses/04.txt data/raw data/trajectories_04
"""
import sys
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt

# name -> (csv stem, label, style)
SERIES = [
    ("ground_truth", "Ground truth", dict(color="black", linewidth=2.5, zorder=5)),
    ("vio_only", "VIO-only", dict(color="#d62728", linewidth=1.3)),
    ("lidar_only", "LiDAR-only", dict(color="#9467bd", linewidth=1.3)),
    ("fused_global", "Fused (global opt.)", dict(color="#1f77b4", linewidth=1.6)),
    ("tightly_coupled_global", "Tightly-coupled (global)",
     dict(color="#2ca02c", linewidth=1.6)),
]


def read_xyz(path: Path) -> list[tuple[float, float, float]]:
    with path.open(newline="") as f:
        return [(float(r["x"]), float(r["y"]), float(r["z"])) for r in csv.DictReader(f)]


def cumulative_distance(points: list[tuple[float, float, float]]) -> list[float]:
    dist = [0.0]
    for a, b in zip(points, points[1:]):
        step = math.dist(a, b)
        dist.append(dist[-1] + step)
    return dist


def main() -> None:
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <trajectory_dir> <output.png>", file=sys.stderr)
        raise SystemExit(1)

    traj_dir, out_path = Path(sys.argv[1]), Path(sys.argv[2])
    ground_truth = read_xyz(traj_dir / "ground_truth.csv")
    path_length = cumulative_distance(ground_truth)

    fig, (ax_top, ax_err) = plt.subplots(1, 2, figsize=(12, 6))

    for stem, label, style in SERIES:
        csv_path = traj_dir / f"{stem}.csv"
        if not csv_path.exists():
            continue
        points = read_xyz(csv_path)
        xs = [p[0] for p in points]
        zs = [p[2] for p in points]
        ax_top.plot(xs, zs, label=label, **style)

        if stem == "ground_truth":
            continue
        n = min(len(points), len(ground_truth))
        errors = [math.dist(points[i], ground_truth[i]) for i in range(n)]
        ax_err.plot(path_length[:n], errors, label=label, **style)

    ax_top.set_xlabel("x (m, camera-right)")
    ax_top.set_ylabel("z (m, camera-forward)")
    ax_top.set_title("Top-down trajectory")
    ax_top.set_aspect("equal", adjustable="datalim")
    ax_top.legend(loc="best", fontsize=8)
    ax_top.grid(True, linewidth=0.4, alpha=0.5)

    ax_err.set_xlabel("distance traveled (m)")
    ax_err.set_ylabel("position error vs. ground truth (m)")
    ax_err.set_title("Error growth over the run")
    ax_err.legend(loc="best", fontsize=8)
    ax_err.grid(True, linewidth=0.4, alpha=0.5)

    fig.suptitle("KITTI sequence 04")
    fig.tight_layout()

    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=150)
    print(f"Wrote {out_path}")


if __name__ == "__main__":
    main()
