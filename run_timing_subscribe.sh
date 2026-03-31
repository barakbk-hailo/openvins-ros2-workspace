#!/usr/bin/env bash
#
# Phase 3: Subscribe-mode realtime feasibility test.
# Runs OpenVINS in subscribe mode while playing a bag at various rates.
#
# Usage:
#   bash run_timing_subscribe.sh [rate]   # default rate=1.0

set -eo pipefail

RATE="${1:-1.0}"
DATASETS_DIR="$HOME/datasets/euroc"
RESULTS_DIR="$HOME/results/timing/x86/subscribe"
TIMING_TMP="/tmp/traj_timing.txt"
WS_DIR="$HOME/workspace/catkin_ws_ov"
SEQ="V1_01_easy"
CONFIG_DIR="$WS_DIR/src/open_vins/config/euroc_mav"
TMP_CONFIG="$CONFIG_DIR/estimator_config_timing.yaml"

source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"

# Create temp config with timing enabled
cp "$CONFIG_DIR/estimator_config.yaml" "$TMP_CONFIG"
sed -i 's/^record_timing_information: false/record_timing_information: true/' "$TMP_CONFIG"
trap 'rm -f "$TMP_CONFIG"' EXIT

mkdir -p "$RESULTS_DIR"

OUT="$RESULTS_DIR/${SEQ}_rate${RATE}.txt"
if [ -f "$OUT" ]; then
  echo "SKIP (exists: $OUT)"
  exit 0
fi

echo "=== Subscribe mode: $SEQ at rate $RATE ==="
rm -f "$TIMING_TMP"

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
else
  echo "WARNING: no timing file produced"
fi
