#!/usr/bin/env python3
"""
Aggregate per-frame benchmark CSVs into cross-run summary tables.

Handles both filename conventions:

  Legacy PWT (from the deleted run_pwt_benchmark_v2.sh):
      serial_run<N>_{wall,cpu,thread,feats,pose}.txt
      sub_run<N>_{wall,cpu,thread,feats,pose}.txt

  Standard orchestrator (run_full_benchmark.sh, post-Phase-1):
      <seq>_<thr>thr[_<cam>][_rate<R>][_run<N>]_{wall,cpu,thread,feats,est}.txt

For each (seq, thr, cam, rate) "cell" the script groups all runs and reports:
- Timing variability:  mean, std, CV, range over per-run wall means (ms)
- Process CPU / Wall:  per-run wall vs cpu means and their ratio
- SLAM features:       mean over per-frame slam_feats_in_state, across runs
- ATE (with --ate):    error_singlerun rmse_ori/rmse_pos, cross-run mean/std/range

Usage:
  python3 scripts/parse_results.py <results_dir> [<results_dir> ...] [--gt <gt>] [--ate] [--brief]
"""
import argparse
import os
import re
import statistics
import subprocess
import sys
from pathlib import Path

# Strip ANSI CSI escapes from `error_singlerun` output.
_ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")

# Known per-frame CSV "clock" suffixes (last token before .txt).
_CLOCKS = ("wall", "cpu", "thread", "feats", "pose", "est")

# Standard-convention filename pattern. Captures: seq, thr, cam, rate, run, clock.
#   <seq>_<thr>thr[_<cam>][_rate<R>][_run<N>]_<clock>.txt
_STD_RE = re.compile(
    r"^(?P<seq>[A-Za-z0-9_]+?)"          # sequence (non-greedy)
    r"_(?P<thr>\d+)thr"                  # _<N>thr
    r"(?:_(?P<cam>mono|stereo))?"        # optional camera tag (default stereo, omitted)
    r"(?:_rate(?P<rate>[0-9]+(?:\.[0-9]+)?))?"  # optional _rate<R>
    r"(?:_run(?P<run>\d+))?"             # optional _run<N>
    r"_(?P<clock>" + "|".join(_CLOCKS) + r")"
    r"\.txt$"
)

# Legacy PWT-investigation pattern. Captures: mode (serial/sub), run, clock.
_LEGACY_RE = re.compile(
    r"^(?P<mode>serial|sub)_run(?P<run>\d+)_(?P<clock>" + "|".join(_CLOCKS) + r")\.txt$"
)

# Legacy rate-feasibility pattern (from the deleted run_timing_subscribe.sh).
# Wall-clock CSV has no _wall suffix; cpu/thread variants do. Treated as
# subscribe runs with run_id derived from the rate.
#   <seq>_rate<R>[_<clock>].txt
_LEGACY_RATE_RE = re.compile(
    r"^(?P<seq>[A-Za-z0-9_]+?)_rate(?P<rate>[0-9]+(?:\.[0-9]+)?)"
    r"(?:_(?P<clock>cpu|thread|feats|pose|est))?"
    r"\.txt$"
)


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


def summary(values, unit_scale=1.0):
    """Return dict with mean, std, cv, min, max, range. n<2 returns std=0."""
    if not values:
        return None
    scaled = [v * unit_scale for v in values]
    mean = statistics.mean(scaled)
    std = statistics.stdev(scaled) if len(scaled) > 1 else 0.0
    return {
        "mean": mean,
        "std": std,
        "cv_pct": (std / mean * 100) if mean > 0 else 0.0,
        "min": min(scaled),
        "max": max(scaled),
        "range": max(scaled) - min(scaled),
        "n": len(scaled),
    }


def have_ov_eval():
    """True if `ros2 run ov_eval error_singlerun` is available on this host."""
    try:
        r = subprocess.run(
            ["ros2", "pkg", "list"], capture_output=True, text=True, timeout=10
        )
        return "ov_eval" in r.stdout
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False


def run_ate(gt, traj):
    """Call ros2 run ov_eval error_singlerun posyaw and parse rmse_ori, rmse_pos."""
    try:
        result = subprocess.run(
            ["ros2", "run", "ov_eval", "error_singlerun", "posyaw", gt, traj],
            capture_output=True, text=True, timeout=60,
        )
        for line in result.stdout.splitlines():
            clean = _ANSI_RE.sub("", line).strip()
            if clean.startswith("rmse_ori"):
                # Format: rmse_ori = 0.536 | rmse_pos = 0.042
                parts = clean.split("|")
                ori = float(parts[0].split("=")[1].strip())
                pos = float(parts[1].split("=")[1].strip())
                return ori, pos
    except Exception as e:
        print(f"  ATE failed for {traj}: {e}", file=sys.stderr)
    return None, None


def classify_files(directory):
    """Walk directory once and group files by cell -> run -> clock -> path.

    A "cell" is a tuple (mode, seq, thr, cam, rate). For legacy PWT files all
    fields except mode are placeholders (None / 'V1_01_easy' / 'stereo' / 1.0).
    """
    cells = {}
    for entry in os.listdir(directory):
        path = Path(directory) / entry
        if not path.is_file():
            continue

        m = _STD_RE.match(entry)
        if m:
            seq = m.group("seq")
            thr = int(m.group("thr"))
            cam = m.group("cam") or "stereo"
            rate = float(m.group("rate")) if m.group("rate") else 1.0
            run = int(m.group("run")) if m.group("run") else 1
            mode = "serial" if m.group("run") is None else "subscribe"
            cell = (mode, seq, thr, cam, rate)
            cells.setdefault(cell, {}).setdefault(run, {})[m.group("clock")] = path
            continue

        m = _LEGACY_RE.match(entry)
        if m:
            mode = "serial" if m.group("mode") == "serial" else "subscribe"
            run = int(m.group("run"))
            cell = (mode, "V1_01_easy", None, "stereo", 1.0)
            cells.setdefault(cell, {}).setdefault(run, {})[m.group("clock")] = path
            continue

        m = _LEGACY_RATE_RE.match(entry)
        if m:
            seq = m.group("seq")
            rate = float(m.group("rate"))
            # Wall-clock has no _<clock> suffix in this legacy convention; map
            # to "wall" so it groups with the cpu/thread/etc. siblings.
            clock = m.group("clock") or "wall"
            cell = ("subscribe", seq, None, "stereo", rate)
            cells.setdefault(cell, {}).setdefault(1, {})[clock] = path
    return cells


def cell_label(cell, brief=False):
    mode, seq, thr, cam, rate = cell
    parts = [seq if seq else "?"]
    if thr is not None:
        parts.append(f"{thr}thr")
    if cam and cam != "stereo":
        parts.append(cam)
    if rate is not None and rate != 1.0:
        parts.append(f"rate={rate}")
    if brief:
        return " ".join(parts) + f" [{mode}]"
    return f"{mode:9s} " + " ".join(parts)


def per_run_means(runs, clock):
    """Return list of per-run means for the given clock CSV, in seconds."""
    out = []
    for run_id in sorted(runs):
        f = runs[run_id].get(clock)
        if not f:
            continue
        v = parse_timing_csv(f)
        if v:
            out.append(statistics.mean(v))
    return out


def report_cell(cell, runs, do_ate, gt, brief):
    label = cell_label(cell, brief=brief)
    wall = per_run_means(runs, "wall")
    cpu = per_run_means(runs, "cpu")
    feats_means = []
    for run_id in sorted(runs):
        f = runs[run_id].get("feats")
        if f:
            m = parse_feats_csv(f)
            if m is not None:
                feats_means.append(m)

    s_wall = summary(wall, unit_scale=1000.0)
    s_feats = summary(feats_means)
    s_ate_pos = s_ate_ori = None
    if do_ate and gt:
        oris, poss = [], []
        # Try _est.txt first (state dump), fall back to _pose.txt (TUM).
        for run_id in sorted(runs):
            traj = runs[run_id].get("est") or runs[run_id].get("pose")
            if not traj:
                continue
            ori, pos = run_ate(gt, str(traj))
            if ori is not None:
                oris.append(ori)
                poss.append(pos)
        if oris:
            s_ate_ori = summary(oris)
            s_ate_pos = summary(poss)

    if brief:
        bits = [label]
        if s_wall:
            bits.append(f"wall {s_wall['mean']:.2f}±{s_wall['std']:.2f} ms (n={s_wall['n']})")
        if s_feats:
            bits.append(f"SLAM {s_feats['mean']:.1f}")
        if s_ate_pos:
            bits.append(f"ATE {s_ate_pos['mean']:.4f}m")
        print("  " + "  ".join(bits))
        return

    print(f"\n  {label}  ({len(runs)} run{'s' if len(runs) != 1 else ''})")
    if s_wall:
        print(f"    wall:  mean={s_wall['mean']:.2f} ms  std={s_wall['std']:.3f}  "
              f"CV={s_wall['cv_pct']:.2f}%  range={s_wall['min']:.2f}–{s_wall['max']:.2f}")
    if cpu:
        s_cpu = summary(cpu, unit_scale=1000.0)
        ratio = (s_cpu["mean"] / s_wall["mean"]) if s_wall and s_wall["mean"] > 0 else 0
        print(f"    cpu:   mean={s_cpu['mean']:.2f} ms  std={s_cpu['std']:.3f}"
              f"   (CPU/Wall = {ratio:.2f}×)")
    if s_feats:
        print(f"    SLAM:  mean={s_feats['mean']:.2f} features  "
              f"range={s_feats['min']:.2f}–{s_feats['max']:.2f}")
        if len(feats_means) <= 15 and len(feats_means) > 1:
            print(f"           per-run: {', '.join(f'{m:.2f}' for m in feats_means)}")
    if s_ate_pos:
        print(f"    ATE:   pos rmse mean={s_ate_pos['mean']:.4f} m "
              f"(std={s_ate_pos['std']:.5f}, range {s_ate_pos['range']*1000:.1f} mm); "
              f"ori rmse mean={s_ate_ori['mean']:.3f}°")


def main():
    ap = argparse.ArgumentParser(description=__doc__.strip().split("\n")[0])
    ap.add_argument("results_dirs", nargs="+",
                    help="One or more results directories to summarise.")
    ap.add_argument("--gt", default=None,
                    help="Ground-truth TUM file for ATE. Defaults to "
                         "<workspace>/src/open_vins/ov_data/euroc_mav/<seq>.txt per cell.")
    ap.add_argument("--ate", action="store_true",
                    help="Compute ATE via `ros2 run ov_eval error_singlerun` (slow). "
                         "Auto-skipped if ov_eval is not installed.")
    ap.add_argument("--brief", action="store_true",
                    help="Terse one-line-per-cell output (orchestrator end-of-run summary).")
    args = ap.parse_args()

    do_ate = args.ate
    if do_ate and not have_ov_eval():
        print("WARNING: --ate requested but `ros2 run ov_eval` is unavailable; skipping ATE.",
              file=sys.stderr)
        do_ate = False

    ws_gt_dir = Path.home() / "workspace" / "catkin_ws_ov" / "src" / "open_vins" / "ov_data" / "euroc_mav"

    for rd in args.results_dirs:
        d = Path(rd)
        if not d.is_dir():
            print(f"Not a directory: {d}", file=sys.stderr)
            continue
        cells = classify_files(d)
        if not cells:
            print(f"\n=== {d} ===\n  (no recognised result files)")
            continue
        total_runs = sum(len(r) for r in cells.values())
        print(f"\n=== {d} ({total_runs} run{'s' if total_runs != 1 else ''} "
              f"across {len(cells)} cell{'s' if len(cells) != 1 else ''}) ===")
        for cell in sorted(cells):
            runs = cells[cell]
            gt = args.gt
            if gt is None:
                seq = cell[1]
                if seq:
                    candidate = ws_gt_dir / f"{seq}.txt"
                    if candidate.exists():
                        gt = str(candidate)
            report_cell(cell, runs, do_ate, gt, brief=args.brief)
        print()


if __name__ == "__main__":
    main()
