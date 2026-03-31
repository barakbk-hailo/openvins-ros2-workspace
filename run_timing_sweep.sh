#!/usr/bin/env bash
#
# Phase 2: Config sensitivity sweeps on V1_01_easy (serial mode, stereo).
# Tests which config knobs matter most for RPi5 optimization.
#
# Usage:
#   bash run_timing_sweep.sh

set -eo pipefail

DATASETS_DIR="$HOME/datasets/euroc"
RESULTS_DIR="$HOME/results/timing/x86/serial/sweep"
TIMING_TMP="/tmp/traj_timing.txt"
WS_DIR="$HOME/workspace/catkin_ws_ov"
SEQ="V1_01_easy"
CONFIG_DIR="$WS_DIR/src/open_vins/config/euroc_mav"
BASE_CONFIG="$CONFIG_DIR/estimator_config.yaml"
TMP_CONFIG="$CONFIG_DIR/estimator_config_sweep.yaml"

source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"

mkdir -p "$RESULTS_DIR"

# Helper: create a modified config and run serial node
run_sweep() {
  local NAME="$1"
  shift  # remaining args are sed expressions

  local OUT="$RESULTS_DIR/${NAME}.txt"
  if [ -f "$OUT" ]; then
    echo "=== $NAME === SKIP (exists: $OUT)"
    return
  fi

  echo "=== $NAME ==="
  cp "$BASE_CONFIG" "$TMP_CONFIG"
  sed -i 's/^record_timing_information: false/record_timing_information: true/' "$TMP_CONFIG"
  for sedexpr in "$@"; do
    sed -i "$sedexpr" "$TMP_CONFIG"
  done

  rm -f "$TIMING_TMP"

  ros2 launch ov_msckf serial.launch.py \
      config_path:="$TMP_CONFIG" \
      path_bag:="$DATASETS_DIR/$SEQ" \
      max_cameras:=2 use_stereo:=true

  if [ -f "$TIMING_TMP" ]; then
    ROWS=$(grep -cv '^#' "$TIMING_TMP" || true)
    cp "$TIMING_TMP" "$OUT"
    echo "  -> Saved $OUT ($ROWS frames)"
  else
    echo "  -> WARNING: no timing file produced for $NAME"
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
