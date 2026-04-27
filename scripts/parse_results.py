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
import shlex
import statistics
import subprocess
import sys
from pathlib import Path

# Strip ANSI CSI escapes from `error_singlerun` output.
_ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")

# RPE per-segment line:
#   seg 8 - median_ori = 128.109 | median_pos = 3.217 (2361 samples)
_RPE_RE = re.compile(
    r"^seg\s+(\d+)\s*-\s*median_ori\s*=\s*([\d.]+)\s*\|\s*median_pos\s*=\s*([\d.]+)"
    r"\s*\((\d+)\s*samples\)"
)


def _ros_distro():
    """Pick ROS distro from /etc/os-release: jazzy on Noble, humble otherwise.
    Mirrors bench_lib.sh:source_ros and install.sh."""
    try:
        with open("/etc/os-release") as f:
            for line in f:
                if line.startswith("UBUNTU_CODENAME=") and "noble" in line:
                    return "jazzy"
    except OSError:
        pass
    return "humble"


def _ros_setup_paths():
    """Setup files to source so `ros2 run ov_eval ...` works in a fresh shell."""
    paths = []
    distro = os.environ.get("ROS_DISTRO_OVERRIDE") or _ros_distro()
    base = f"/opt/ros/{distro}/setup.bash"
    if os.path.isfile(base):
        paths.append(base)
    ws_install = Path.home() / "workspace" / "catkin_ws_ov" / "install" / "setup.bash"
    if ws_install.is_file():
        paths.append(str(ws_install))
    return paths


def _run_ros2(cmd, timeout=60):
    """Run a ros2 command with ROS env auto-sourced and headless Qt platform.
    Returns the CompletedProcess. cmd is a list of strings."""
    setup = _ros_setup_paths()
    if not setup:
        # No ROS install detected; surface that rather than silently failing.
        return subprocess.CompletedProcess(cmd, returncode=127, stdout="", stderr="")
    sourced = " && ".join(f"source {shlex.quote(p)}" for p in setup)
    quoted = " ".join(shlex.quote(c) for c in cmd)
    bash = f"{sourced} && export QT_QPA_PLATFORM=offscreen && exec {quoted}"
    return subprocess.run(
        ["bash", "-c", bash],
        capture_output=True, text=True, timeout=timeout,
    )

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


# Component column order matches the OpenVINS timing CSV header:
#   timestamp, tracking, propagation, msckf update, slam update,
#   slam delayed, re-tri & marg, total
_COMPONENTS = ("tracking", "propagation", "msckf update", "slam update",
               "slam delayed", "re-tri & marg", "total")


def parse_timing_csv_components(path):
    """Return dict {component_name: [per-frame values in seconds]}."""
    out = {c: [] for c in _COMPONENTS}
    with open(path) as f:
        for line in f:
            if line.startswith("#"):
                continue
            parts = line.strip().split(",")
            if len(parts) < 8:
                continue
            for i, c in enumerate(_COMPONENTS):
                try:
                    out[c].append(float(parts[i + 1]))
                except (ValueError, IndexError):
                    pass
    return out


def percentile(values, p):
    """Linear-interpolated percentile (matches numpy's default). p in [0,100]."""
    if not values:
        return None
    xs = sorted(values)
    if len(xs) == 1:
        return xs[0]
    k = (len(xs) - 1) * p / 100.0
    lo = int(k)
    hi = min(lo + 1, len(xs) - 1)
    frac = k - lo
    return xs[lo] + frac * (xs[hi] - xs[lo])


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
    """True if `ros2 run ov_eval error_singlerun` is available on this host.
    Auto-sources ROS — caller doesn't need to source first."""
    try:
        r = _run_ros2(["ros2", "pkg", "list"], timeout=15)
        return "ov_eval" in r.stdout
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False


def run_ate_rpe(gt, traj):
    """Run `ros2 run ov_eval error_singlerun posyaw` and parse both ATE and
    per-segment RPE. Returns dict {ate_ori, ate_pos, rpe: {seg_len: dict}} or
    None on failure. Auto-sources ROS and sets QT_QPA_PLATFORM=offscreen so it
    works on headless hosts."""
    try:
        result = _run_ros2(
            ["ros2", "run", "ov_eval", "error_singlerun", "posyaw", gt, traj],
            timeout=60,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired) as e:
        print(f"  ATE/RPE failed for {traj}: {e}", file=sys.stderr)
        return None

    out = {"ate_ori": None, "ate_pos": None, "rpe": {}}
    for line in result.stdout.splitlines():
        clean = _ANSI_RE.sub("", line).strip()
        if clean.startswith("rmse_ori"):
            try:
                parts = clean.split("|")
                out["ate_ori"] = float(parts[0].split("=")[1].strip())
                out["ate_pos"] = float(parts[1].split("=")[1].strip())
            except (ValueError, IndexError):
                pass
            continue
        m = _RPE_RE.match(clean)
        if m:
            seg = int(m.group(1))
            out["rpe"][seg] = {
                "med_ori": float(m.group(2)),
                "med_pos": float(m.group(3)),
                "samples": int(m.group(4)),
            }
    if out["ate_ori"] is None and not out["rpe"]:
        # Surface why nothing was parsed — most likely Qt/matplotlib aborted
        # before the stats lines, or the trajectory didn't match the GT.
        last = (result.stderr.strip().splitlines() or [""])[-1]
        if last:
            print(f"  ATE/RPE: no stats parsed for {Path(traj).name} "
                  f"(rc={result.returncode}, last stderr: {last[:120]})",
                  file=sys.stderr)
        return None
    return out


# Back-compat shim for callers that only need ATE (ori, pos).
def run_ate(gt, traj):
    res = run_ate_rpe(gt, traj)
    if res is None:
        return None, None
    return res["ate_ori"], res["ate_pos"]


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


def collect_ate_rpe(runs, gt):
    """Run error_singlerun on each rep's est/pose file and collect per-rep
    ATE + per-segment RPE. Returns (ate_oris, ate_poss, rpe_by_seg) where
    rpe_by_seg = {seg_len: {"med_ori": [...], "med_pos": [...]}}."""
    ate_oris, ate_poss = [], []
    rpe_by_seg = {}
    for run_id in sorted(runs):
        traj = runs[run_id].get("est") or runs[run_id].get("pose")
        if not traj:
            continue
        res = run_ate_rpe(gt, str(traj))
        if not res:
            continue
        if res["ate_ori"] is not None:
            ate_oris.append(res["ate_ori"])
            ate_poss.append(res["ate_pos"])
        for seg, vals in res["rpe"].items():
            d = rpe_by_seg.setdefault(seg, {"med_ori": [], "med_pos": []})
            d["med_ori"].append(vals["med_ori"])
            d["med_pos"].append(vals["med_pos"])
    return ate_oris, ate_poss, rpe_by_seg


def report_cell(cell, runs, do_ate, gt, brief, do_rpe=False):
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
    rpe_by_seg = {}
    if (do_ate or do_rpe) and gt:
        ate_oris, ate_poss, rpe_by_seg = collect_ate_rpe(runs, gt)
        if ate_oris:
            s_ate_ori = summary(ate_oris)
            s_ate_pos = summary(ate_poss)

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
    if do_rpe and rpe_by_seg:
        print(f"    RPE:   per-segment median, across {len(runs)} run"
              f"{'s' if len(runs) != 1 else ''}")
        for seg in sorted(rpe_by_seg):
            d = rpe_by_seg[seg]
            ms_pos = summary(d["med_pos"])
            ms_ori = summary(d["med_ori"])
            if not ms_pos:
                continue
            print(f"           seg {seg:>3d}s: pos {ms_pos['mean']:.3f}±{ms_pos['std']:.3f} m, "
                  f"ori {ms_ori['mean']:.2f}±{ms_ori['std']:.2f}°  (n={ms_pos['n']})")


def report_cell_per_component(cell, runs):
    """Paper-style per-component breakdown. Frames are pooled across all reps
    in this cell, then mean / std / p99 are computed per component, per clock.
    Matches the convention in docs/benchmark-analysis.md (cells are
    `mean ± std (p99: X)` over per-frame values, in ms)."""
    label = cell_label(cell, brief=False)
    rep_count = len(runs)

    pooled = {"wall": {}, "cpu": {}, "thread": {}}
    total_frames = 0
    for run_id in sorted(runs):
        for clock in ("wall", "cpu", "thread"):
            f = runs[run_id].get(clock)
            if not f:
                continue
            comps = parse_timing_csv_components(f)
            for comp, vals in comps.items():
                pooled[clock].setdefault(comp, []).extend(vals)
                if clock == "wall" and comp == "total":
                    total_frames += len(vals)

    if not pooled["wall"]:
        return

    plural = "s" if rep_count != 1 else ""
    print(f"\n  {label}  ({rep_count} run{plural}, {total_frames} frames pooled)")
    print(f"    {'Component':<14s} {'wall (mean±std, p99)':<24s} "
          f"{'cpu (mean±std, p99)':<24s} {'thread (mean±std, p99)':<24s}")
    for comp in _COMPONENTS:
        cells = []
        for clock in ("wall", "cpu", "thread"):
            v = pooled[clock].get(comp, [])
            if not v:
                cells.append(f"{'—':<24s}")
                continue
            v_ms = [x * 1000 for x in v]
            mean = statistics.mean(v_ms)
            std = statistics.stdev(v_ms) if len(v_ms) > 1 else 0.0
            p99 = percentile(v_ms, 99)
            cells.append(f"{f'{mean:.1f}±{std:.1f} (p99: {p99:.1f})':<24s}")
        prefix = "**" if comp == "total" else "  "
        print(f"  {prefix} {comp:<14s} {' '.join(cells)}")


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
    ap.add_argument("--per-component", action="store_true",
                    help="Paper-style per-component breakdown (tracking, propagation, "
                         "msckf update, …, total) with mean ± std and p99 over pooled frames "
                         "across all reps in each cell. Matches docs/benchmark-analysis.md.")
    ap.add_argument("--rpe", action="store_true",
                    help="Print Relative Pose Error per segment (median pos/ori per default "
                         "{8,16,24,32,40} s segments, mean ± std across reps). Auto-enables --ate.")
    ap.add_argument("--detailed", action="store_true",
                    help="Shortcut: --per-component + --ate + --rpe. Everything we can "
                         "extract from the result files for each cell.")
    args = ap.parse_args()

    # --detailed is a meta flag.
    if args.detailed:
        args.per_component = True
        args.ate = True
        args.rpe = True
    # --rpe implies --ate (same subprocess call produces both).
    if args.rpe:
        args.ate = True

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
        # Some legacy cells have thr=None (rate-feasibility files); map to -1
        # so tuple comparison doesn't trip over None vs int.
        for cell in sorted(cells, key=lambda c: tuple(-1 if x is None else x for x in c)):
            runs = cells[cell]
            gt = args.gt
            if gt is None:
                seq = cell[1]
                if seq:
                    candidate = ws_gt_dir / f"{seq}.txt"
                    if candidate.exists():
                        gt = str(candidate)
            if args.per_component:
                report_cell_per_component(cell, runs)
                # When --detailed (or --ate / --rpe alongside --per-component),
                # also print the ATE/RPE summary so the user sees everything.
                if (do_ate or args.rpe) and gt:
                    ate_oris, ate_poss, rpe_by_seg = collect_ate_rpe(runs, gt)
                    if ate_poss:
                        s_pos = summary(ate_poss)
                        s_ori = summary(ate_oris)
                        print(f"    ATE:   pos rmse mean={s_pos['mean']:.4f} m "
                              f"(std={s_pos['std']:.5f}, range {s_pos['range']*1000:.1f} mm); "
                              f"ori rmse mean={s_ori['mean']:.3f}°")
                    if args.rpe and rpe_by_seg:
                        print(f"    RPE:   per-segment median, across {len(runs)} run"
                              f"{'s' if len(runs) != 1 else ''}")
                        for seg in sorted(rpe_by_seg):
                            d = rpe_by_seg[seg]
                            ms_pos = summary(d["med_pos"])
                            ms_ori = summary(d["med_ori"])
                            if not ms_pos:
                                continue
                            print(f"           seg {seg:>3d}s: pos {ms_pos['mean']:.3f}±{ms_pos['std']:.3f} m, "
                                  f"ori {ms_ori['mean']:.2f}±{ms_ori['std']:.2f}°  (n={ms_pos['n']})")
            else:
                report_cell(cell, runs, do_ate, gt, brief=args.brief, do_rpe=args.rpe)
        print()


if __name__ == "__main__":
    main()
