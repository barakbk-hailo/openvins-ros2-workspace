#!/usr/bin/env bash
#
# Phase 3: Subscribe-mode realtime feasibility test.
# Runs OpenVINS in subscribe mode while playing a bag at various rates.
#
# Usage:
#   bash run_timing_subscribe.sh [rate] [--tag <name>] [--slam-chi2-recovery true|false]
#
# --tag routes output under $HOME/results/timing/x86/subscribe/<tag>/
# so reruns don't clobber earlier results.
# --slam-chi2-recovery overrides the YAML key in the temp config
# (default: leave estimator_config.yaml's value alone — shipping default is false).

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=bench_lib.sh
. "$SCRIPT_DIR/bench_lib.sh"

usage() {
  cat <<'EOF'
Phase 3: Subscribe-mode realtime feasibility test.

Usage:
  bash run_timing_subscribe.sh [rate] [--tag <name>] [--slam-chi2-recovery true|false]

Positional:
  rate                 ros2 bag play rate (default: 1.0)

Options:
  --tag <name>         output goes to $HOME/results/timing/x86/subscribe/<tag>/
                       (default: no subdir)
  --slam-chi2-recovery <true|false>
                       override slam_chi2_recovery in the temp config
  -h, --help           show this help and exit

Output naming:
  V1_01_easy_rate<rate>.txt              (wall-clock CSV)
  V1_01_easy_rate<rate>_cpu.txt          (process-CPU CSV)
  V1_01_easy_rate<rate>_thread.txt       (VIO-thread-CPU CSV)
EOF
}

RATE="1.0"
TAG=""
SLAM_CHI2_RECOVERY=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --tag=*)                  TAG="${1#*=}"; shift ;;
    --tag)                    TAG="${2:-}"; shift 2 ;;
    --slam-chi2-recovery=*)   SLAM_CHI2_RECOVERY="${1#*=}"; shift ;;
    --slam-chi2-recovery)     SLAM_CHI2_RECOVERY="${2:-}"; shift 2 ;;
    -h|--help)                usage; exit 0 ;;
    *)                        RATE="$1"; shift ;;
  esac
done

validate_chi2_recovery "$SLAM_CHI2_RECOVERY" || exit 2
[[ "$RATE" =~ ^[0-9]+(\.[0-9]+)?$ ]] || { echo "ERROR: rate must be a positive number (got: $RATE)" >&2; exit 2; }
require_dir "datasets" "$DATASETS_DIR" || exit 1

SEQ="V1_01_easy"
require_bag "$SEQ" || exit 1

RESULTS_DIR="$HOME/results/timing/x86/subscribe${TAG:+/$TAG}"
TMP_CONFIG="$CONFIG_DIR/estimator_config_timing.yaml"

cleanup() {
  rm -f "$TMP_CONFIG" 2>/dev/null || true
  if [ -n "${OV_PID:-}" ]; then
    kill "$OV_PID" 2>/dev/null || true
    wait "$OV_PID" 2>/dev/null || true
  fi
  kill_stale_subscribe_nodes
}
trap cleanup EXIT INT TERM

source_ros
kill_stale_subscribe_nodes

make_bench_config "$TMP_CONFIG" "$SLAM_CHI2_RECOVERY" || exit 1

mkdir -p "$RESULTS_DIR"

OUT="$RESULTS_DIR/${SEQ}_rate${RATE}.txt"
if [ -f "$OUT" ]; then
  echo "SKIP (exists: $OUT)"
  exit 0
fi

echo "=== Subscribe mode: $SEQ at rate $RATE ==="
rm -f "$TIMING_WALL_TMP" "$TIMING_CPU_TMP" "$TIMING_THREAD_TMP"

# Launch OpenVINS subscribe node in background
ros2 launch ov_msckf subscribe.launch.py config_path:="$TMP_CONFIG" &
OV_PID=$!

# Wait for node to be ready
sleep 3

# Play the bag at the specified rate (blocks until done)
echo "Playing bag at --rate $RATE ..."
ros2 bag play "$DATASETS_DIR/$SEQ" --rate "$RATE" 2>&1

# Give OpenVINS time to finish processing queued messages
echo "Bag finished, waiting for OpenVINS to drain queue..."
sleep 5

# Kill OpenVINS (cleanup trap will also catch this)
kill "$OV_PID" 2>/dev/null || true
wait "$OV_PID" 2>/dev/null || true
OV_PID=""

if [ -f "$TIMING_WALL_TMP" ]; then
  ROWS=$(grep -cv '^#' "$TIMING_WALL_TMP" || true)
  cp "$TIMING_WALL_TMP" "$OUT"
  echo ""
  echo "=== Results ==="
  echo "Processed frames: $ROWS / 2912 expected"
  DROPPED=$((2912 - ROWS))
  echo "Dropped frames:   $DROPPED"
  DROP_PCT=$(awk "BEGIN {printf \"%.1f\", 100.0 * $DROPPED / 2912}")
  echo "Drop rate:        ${DROP_PCT}%"
  echo "Saved: $OUT"
  if [ -f "$TIMING_CPU_TMP" ]; then
    cp "$TIMING_CPU_TMP" "${OUT%.txt}_cpu.txt"
    echo "Saved: ${OUT%.txt}_cpu.txt (process CPU time)"
  fi
  if [ -f "$TIMING_THREAD_TMP" ]; then
    cp "$TIMING_THREAD_TMP" "${OUT%.txt}_thread.txt"
    echo "Saved: ${OUT%.txt}_thread.txt (thread CPU time)"
  fi
else
  echo "WARNING: no timing file produced" >&2
  exit 1
fi
