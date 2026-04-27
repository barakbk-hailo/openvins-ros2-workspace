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

set -eo pipefail

# Kill any stale OpenVINS subscribe processes from previous runs
pkill -9 -f "run_subscribe_msckf" 2>/dev/null || true
sleep 1

RATE="1.0"
TAG=""
SLAM_CHI2_RECOVERY=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --tag) TAG="$2"; shift 2 ;;
    --slam-chi2-recovery) SLAM_CHI2_RECOVERY="$2"; shift 2 ;;
    -h|--help) sed -n '2,11p' "$0" | sed 's/^# \?//'; exit 0 ;;
    *) RATE="$1"; shift ;;
  esac
done

case "${SLAM_CHI2_RECOVERY:-}" in ""|true|false) ;; *) echo "ERROR: --slam-chi2-recovery must be true or false (got: $SLAM_CHI2_RECOVERY)" >&2; exit 2 ;; esac

DATASETS_DIR="$HOME/datasets/euroc"
RESULTS_DIR="$HOME/results/timing/x86/subscribe${TAG:+/$TAG}"
TIMING_TMP="/tmp/traj_timing.txt"
TIMING_TMP_CPU="/tmp/traj_timing_cpu.txt"
TIMING_TMP_THREAD="/tmp/traj_timing_thread.txt"
WS_DIR="$HOME/workspace/catkin_ws_ov"
SEQ="V1_01_easy"
CONFIG_DIR="$WS_DIR/src/open_vins/config/euroc_mav"
TMP_CONFIG="$CONFIG_DIR/estimator_config_timing.yaml"

for _distro in jazzy humble; do
  if [ -f "/opt/ros/$_distro/setup.bash" ]; then source "/opt/ros/$_distro/setup.bash"; break; fi
done
source "$WS_DIR/install/setup.bash"

# Create temp config with timing enabled
cp "$CONFIG_DIR/estimator_config.yaml" "$TMP_CONFIG"
sed -i 's/^record_timing_information: false/record_timing_information: true/' "$TMP_CONFIG"
sed -i 's/^record_timing_cpu_time: false/record_timing_cpu_time: true/' "$TMP_CONFIG"
sed -i 's/^record_timing_thread_time: false/record_timing_thread_time: true/' "$TMP_CONFIG"
if [ -n "${SLAM_CHI2_RECOVERY:-}" ]; then
  sed -i "s/^slam_chi2_recovery: .*/slam_chi2_recovery: ${SLAM_CHI2_RECOVERY} # overridden via --slam-chi2-recovery/" "$TMP_CONFIG"
fi
trap 'rm -f "$TMP_CONFIG"' EXIT

mkdir -p "$RESULTS_DIR"

OUT="$RESULTS_DIR/${SEQ}_rate${RATE}.txt"
if [ -f "$OUT" ]; then
  echo "SKIP (exists: $OUT)"
  exit 0
fi

echo "=== Subscribe mode: $SEQ at rate $RATE ==="
rm -f "$TIMING_TMP" "$TIMING_TMP_CPU" "$TIMING_TMP_THREAD"

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

# Kill OpenVINS
kill $OV_PID 2>/dev/null || true
wait $OV_PID 2>/dev/null || true

if [ -f "$TIMING_TMP" ]; then
  ROWS=$(grep -cv '^#' "$TIMING_TMP" || true)
  cp "$TIMING_TMP" "$OUT"
  echo ""
  echo "=== Results ==="
  echo "Processed frames: $ROWS / 2912 expected"
  DROPPED=$((2912 - ROWS))
  echo "Dropped frames:   $DROPPED"
  DROP_PCT=$(awk "BEGIN {printf \"%.1f\", 100.0 * $DROPPED / 2912}")
  echo "Drop rate:        ${DROP_PCT}%"
  echo "Saved: $OUT"
  if [ -f "$TIMING_TMP_CPU" ]; then
    cp "$TIMING_TMP_CPU" "${OUT%.txt}_cpu.txt"
    echo "Saved: ${OUT%.txt}_cpu.txt (process CPU time)"
  fi
  if [ -f "$TIMING_TMP_THREAD" ]; then
    cp "$TIMING_TMP_THREAD" "${OUT%.txt}_thread.txt"
    echo "Saved: ${OUT%.txt}_thread.txt (thread CPU time)"
  fi
else
  echo "WARNING: no timing file produced"
fi
