#!/usr/bin/env python3
"""
Aggregate PWT benchmark results for cross-run analysis.

Parses:
  serial_run{1,2}_{wall,cpu,thread,feats,pose}.txt
  sub_run{1..N}_{wall,cpu,thread,feats,pose}.txt

Produces tables matching persistent-worker.md format:
  - Cross-run timing variability (mean, std, CV, range)
  - Process-CPU / Wall ratio
  - SLAM features in state (mean per run, across runs)
  - ATE per run, range, std

Usage:
  python3 aggregate_pwt.py <results_dir> [--gt <groundtruth_file>]
"""
import argparse
import csv
import glob
import os
import statistics
import subprocess
import sys
from pathlib import Path


def parse_timing_csv(path):
    """Return list of floats from the 'total' (last) column, in seconds."""
    values = []
    with open(path) as f:
        for line in f:
            if line.startswith("#"):
                continue
            parts = line.strip().split(",")
            if len(parts) < 8:
                continue
            try:
                values.append(float(parts[7]))
            except ValueError:
                pass
    return values


def parse_feats_csv(path):
    """Return mean of column 1 (slam_feats_in_state) across all frames."""
    values = []
    with open(path) as f:
        for line in f:
            if line.startswith("#"):
                continue
            parts = line.strip().split(",")
            if len(parts) < 2:
                continue
            try:
                values.append(int(parts[1]))
            except ValueError:
                pass
    return statistics.mean(values) if values else None


def summary(values, unit_scale=1.0, unit_name=""):
    """Return dict with mean, std, cv, min, max, range."""
    if not values:
        return None
    scaled = [v * unit_scale for v in values]
    mean = statistics.mean(scaled)
    std = statistics.pstdev(scaled) if len(scaled) > 1 else 0.0
    return {
        "mean": mean,
        "std": std,
        "cv_pct": (std / mean * 100) if mean > 0 else 0.0,
        "min": min(scaled),
        "max": max(scaled),
        "range": max(scaled) - min(scaled),
        "n": len(scaled),
        "unit": unit_name,
    }


def run_ate(gt, pose):
    """Call ros2 run ov_eval error_singlerun posyaw and parse rmse_ori, rmse_pos."""
    try:
        result = subprocess.run(
            ["ros2", "run", "ov_eval", "error_singlerun", "posyaw", gt, pose],
            capture_output=True, text=True, timeout=60,
        )
        for line in result.stdout.splitlines():
            # Strip ANSI codes
            clean = line.replace("\x1b[0m", "").replace("\x1b[95m", "").strip()
            if clean.startswith("rmse_ori"):
                # Format: rmse_ori = 0.536 | rmse_pos = 0.042
                parts = clean.split("|")
                ori = float(parts[0].split("=")[1].strip())
                pos = float(parts[1].split("=")[1].strip())
                return ori, pos
    except Exception as e:
        print(f"  ATE failed for {pose}: {e}", file=sys.stderr)
    return None, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("results_dir")
    ap.add_argument("--gt", default="/opt/ros_ws/src/open_vins/ov_data/euroc_mav/V1_01_easy.txt",
                    help="Ground truth file (default: V1_01_easy.txt in container)")
    ap.add_argument("--ate", action="store_true",
                    help="Compute ATE by calling ros2 run ov_eval (requires ROS2 env)")
    args = ap.parse_args()

    d = Path(args.results_dir)
    if not d.is_dir():
        print(f"Not a directory: {d}", file=sys.stderr)
        sys.exit(1)

    # Collect all runs by tag prefix (serial_runN, sub_runN)
    serial_tags = sorted({p.stem.replace("_wall", "").replace("_cpu", "").replace("_thread", "")
                          .replace("_feats", "").replace("_pose", "")
                          for p in d.glob("serial_run*_*.txt")})
    sub_tags = sorted({p.stem.replace("_wall", "").replace("_cpu", "").replace("_thread", "")
                       .replace("_feats", "").replace("_pose", "")
                       for p in d.glob("sub_run*_*.txt")},
                      key=lambda s: int(s.replace("sub_run", "")))

    print(f"\n=== {d.name} ===")
    print(f"Serial runs found: {len(serial_tags)}")
    print(f"Subscribe runs found: {len(sub_tags)}")

    # Per-run wall-clock mean (total time)
    def per_run_wall_mean(tag):
        f = d / f"{tag}_wall.txt"
        v = parse_timing_csv(f) if f.exists() else []
        return statistics.mean(v) if v else None

    def per_run_cpu_mean(tag):
        f = d / f"{tag}_cpu.txt"
        v = parse_timing_csv(f) if f.exists() else []
        return statistics.mean(v) if v else None

    serial_wall = [per_run_wall_mean(t) for t in serial_tags]
    serial_cpu = [per_run_cpu_mean(t) for t in serial_tags]
    sub_wall = [per_run_wall_mean(t) for t in sub_tags]
    sub_cpu = [per_run_cpu_mean(t) for t in sub_tags]

    serial_wall = [v for v in serial_wall if v is not None]
    serial_cpu = [v for v in serial_cpu if v is not None]
    sub_wall = [v for v in sub_wall if v is not None]
    sub_cpu = [v for v in sub_cpu if v is not None]

    # ─── Cross-run timing variability (ms) ───
    print("\n── Timing variability (total frame time across reps, ms) ──")
    for label, vals in [("Serial", serial_wall), ("Subscribe", sub_wall)]:
        s = summary(vals, unit_scale=1000.0, unit_name="ms")
        if s:
            print(f"  {label} ({s['n']} runs): mean={s['mean']:.2f} std={s['std']:.3f} "
                  f"CV={s['cv_pct']:.2f}% range={s['min']:.2f}–{s['max']:.2f}")

    # ─── Process CPU / Wall ratio ───
    print("\n── Process CPU / Wall ratio (avg of per-run means) ──")
    for label, wall, cpu in [("Serial", serial_wall, serial_cpu),
                              ("Subscribe", sub_wall, sub_cpu)]:
        if wall and cpu:
            ratios = [c/w for c, w in zip(cpu, wall) if w > 0]
            if ratios:
                avg = statistics.mean(ratios)
                print(f"  {label}: mean wall={statistics.mean(wall)*1000:.2f}ms, "
                      f"mean cpu={statistics.mean(cpu)*1000:.2f}ms, ratio={avg:.2f}x")

    # ─── SLAM feature health ───
    print("\n── SLAM features in state (mean across frames, per run) ──")
    for label, tags in [("Serial", serial_tags), ("Subscribe", sub_tags)]:
        means = []
        for t in tags:
            f = d / f"{t}_feats.txt"
            if f.exists():
                m = parse_feats_csv(f)
                if m is not None:
                    means.append(m)
        if means:
            s = summary(means)
            print(f"  {label} ({s['n']} runs): mean={s['mean']:.2f} "
                  f"range={s['min']:.2f}–{s['max']:.2f}")
            if len(tags) <= 15:
                print(f"     per-run: {', '.join(f'{m:.2f}' for m in means)}")

    # ─── ATE ───
    if args.ate:
        print("\n── ATE (posyaw alignment) ──")
        for label, tags in [("Serial", serial_tags), ("Subscribe", sub_tags)]:
            oris, poss = [], []
            for t in tags:
                f = d / f"{t}_pose.txt"
                if f.exists():
                    ori, pos = run_ate(args.gt, str(f))
                    if ori is not None:
                        oris.append(ori)
                        poss.append(pos)
            if oris:
                so = summary(oris)
                sp = summary(poss)
                print(f"  {label} ({so['n']} runs):")
                print(f"    rmse_ori: mean={so['mean']:.3f} std={so['std']:.4f} "
                      f"range={so['min']:.3f}–{so['max']:.3f} (range={so['range']:.3f}°)")
                print(f"    rmse_pos: mean={sp['mean']:.4f} std={sp['std']:.5f} "
                      f"range={sp['min']:.4f}–{sp['max']:.4f} (range={sp['range']*1000:.1f}mm)")
                if len(tags) <= 15:
                    print(f"    per-run ori: {', '.join(f'{v:.3f}' for v in oris)}")
                    print(f"    per-run pos: {', '.join(f'{v:.4f}' for v in poss)}")

    # ─── Frame counts ───
    print("\n── Frame counts per run ──")
    for label, tags in [("Serial", serial_tags), ("Subscribe", sub_tags)]:
        counts = []
        for t in tags:
            f = d / f"{t}_wall.txt"
            if f.exists():
                with open(f) as fh:
                    n = sum(1 for ln in fh if not ln.startswith("#"))
                counts.append(n)
        if counts:
            s = summary(counts)
            print(f"  {label} ({s['n']} runs): mean={s['mean']:.0f} "
                  f"range={int(s['min'])}–{int(s['max'])}")

    print()


if __name__ == "__main__":
    main()
