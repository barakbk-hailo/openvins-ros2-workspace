#!/usr/bin/env bash
#
# Phase 2: Config sensitivity sweeps on V1_01_easy (serial mode, stereo).
# Tests which config knobs matter most for RPi5 optimization.
#
# Usage:
#   bash run_timing_sweep.sh [--tag <name>] [--slam-chi2-recovery true|false]

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/bench_lib.sh
. "$SCRIPT_DIR/scripts/bench_lib.sh"

usage() {
  cat <<'EOF'
Phase 2: Config sensitivity sweeps on V1_01_easy (serial mode, stereo).
Tests which config knobs matter most for RPi5 optimization.

Usage:
  bash run_timing_sweep.sh [--tag <name>] [--slam-chi2-recovery true|false]

Options:
  --tag <name>         output goes to $HOME/results/timing/x86/serial/sweep/<tag>/
  --slam-chi2-recovery <true|false>
                       override slam_chi2_recovery in each variant's temp config
  -h, --help           show this help and exit

Variants:
  A_downsample        — half-resolution input images
  B_num_pts_100       — 100 features (vs default 200)
  C_num_pts_300       — 300 features (vs default 200)
  D_no_slam           — MSCKF-only (max_slam=0)
  E_opencv_1thread    — single-threaded OpenCV (RPi5 thermal proxy)

Compare against the baseline:
  ros2 run ov_eval timing_comparison \
      ~/results/timing/x86/serial/stereo/V1_01_easy.txt \
      ~/results/timing/x86/serial/sweep/<tag>/*.txt
EOF
}

TAG=""
SLAM_CHI2_RECOVERY=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --tag=*)                  TAG="${1#*=}"; shift ;;
    --tag)                    TAG="${2:-}"; shift 2 ;;
    --slam-chi2-recovery=*)   SLAM_CHI2_RECOVERY="${1#*=}"; shift ;;
    --slam-chi2-recovery)     SLAM_CHI2_RECOVERY="${2:-}"; shift 2 ;;
    -h|--help)                usage; exit 0 ;;
    *)                        echo "ERROR: unknown arg: $1" >&2; usage >&2; exit 2 ;;
  esac
done

validate_chi2_recovery "$SLAM_CHI2_RECOVERY" || exit 2
require_dir "datasets" "$DATASETS_DIR" || exit 1

SEQ="V1_01_easy"
require_bag "$SEQ" || exit 1

RESULTS_DIR="$HOME/results/timing/x86/serial/sweep${TAG:+/$TAG}"
TMP_CONFIG="$CONFIG_DIR/estimator_config_sweep.yaml"

cleanup() {
  rm -f "$TMP_CONFIG" 2>/dev/null || true
  kill_stale_subscribe_nodes  # belt-and-braces; sweep is serial-only but covers Ctrl-C
}
trap cleanup EXIT INT TERM

source_ros

mkdir -p "$RESULTS_DIR"

# Helper: create a modified config (baseline timing-recording) and run serial node.
# Each variant's extra sed expressions are applied AFTER the timing-recording flips
# so they can override anything if needed.
run_sweep() {
  local NAME="$1"
  shift  # remaining args are sed expressions

  local OUT="$RESULTS_DIR/${NAME}.txt"
  if [ -f "$OUT" ]; then
    echo "=== $NAME === SKIP (exists: $OUT)"
    return
  fi

  echo "=== $NAME ==="
  make_bench_config "$TMP_CONFIG" "$SLAM_CHI2_RECOVERY" || return 1
  for sedexpr in "$@"; do
    sed -i "$sedexpr" "$TMP_CONFIG"
  done

  rm -f "$TIMING_WALL_TMP" "$TIMING_CPU_TMP" "$TIMING_THREAD_TMP"

  ros2 launch ov_msckf serial.launch.py \
      config_path:="$TMP_CONFIG" \
      path_bag:="$DATASETS_DIR/$SEQ" \
      max_cameras:=2 use_stereo:=true || {
    echo "  -> launch FAILED for $NAME" >&2
    return 1
  }

  if [ -f "$TIMING_WALL_TMP" ]; then
    local ROWS
    ROWS=$(grep -cv '^#' "$TIMING_WALL_TMP" || true)
    cp "$TIMING_WALL_TMP" "$OUT"
    echo "  -> Saved $OUT ($ROWS frames)"
    if [ "$ROWS" -lt "$MIN_ROWS_FOR_COMPLETE_RUN" ]; then
      echo "  -> WARNING: only $ROWS frames (< $MIN_ROWS_FOR_COMPLETE_RUN); run may be incomplete" >&2
    fi
  else
    echo "  -> WARNING: no timing file produced for $NAME" >&2
  fi
  if [ -f "$TIMING_CPU_TMP" ]; then
    cp "$TIMING_CPU_TMP" "${OUT%.txt}_cpu.txt"
    echo "  -> Saved ${OUT%.txt}_cpu.txt (process CPU time)"
  fi
  if [ -f "$TIMING_THREAD_TMP" ]; then
    cp "$TIMING_THREAD_TMP" "${OUT%.txt}_thread.txt"
    echo "  -> Saved ${OUT%.txt}_thread.txt (thread CPU time)"
  fi
}

echo "======== Phase 2: Sensitivity sweeps on $SEQ (stereo) ========"
echo ""

# A: Half-resolution images
run_sweep "A_downsample" \
  's/^downsample_cameras: false/downsample_cameras: true/'

# B: Fewer features (100 instead of 200)
run_sweep "B_num_pts_100" \
  's/^num_pts: 200/num_pts: 100/'

# C: More features (300 instead of 200)
run_sweep "C_num_pts_300" \
  's/^num_pts: 200/num_pts: 300/'

# D: MSCKF-only (no SLAM features)
run_sweep "D_no_slam" \
  's/^max_slam: 50/max_slam: 0/' \
  's/^max_slam_in_update: 25/max_slam_in_update: 0/'

# E: Single-threaded OpenCV (simulates constrained RPi5 thermal)
run_sweep "E_opencv_1thread" \
  's/^num_opencv_threads: 4/num_opencv_threads: 1/'

echo ""
echo "======== Done ========"
echo "Results in: $RESULTS_DIR"
echo ""
echo "Compare all variants:"
echo "  ros2 run ov_eval timing_comparison ~/results/timing/x86/serial/stereo/V1_01_easy.txt $RESULTS_DIR/*.txt"
