#!/usr/bin/env bash
#
# Run OpenVINS serial timing benchmarks on EuRoC sequences.
# Produces per-component timing CSVs for analysis with ov_eval tools.
#
# Usage:
#   bash run_timing_benchmark.sh
#
# Prerequisites:
#   - Workspace built: colcon build --symlink-install
#   - EuRoC ROS2 bags in ~/datasets/euroc/

set -eo pipefail

# --- Configuration ---
DATASETS_DIR="$HOME/datasets/euroc"
RESULTS_DIR="$HOME/results/timing/x86/serial"
TIMING_TMP="/tmp/traj_timing.txt"
TIMING_TMP_CPU="/tmp/traj_timing_cpu.txt"
TIMING_TMP_THREAD="/tmp/traj_timing_thread.txt"
WS_DIR="$HOME/workspace/catkin_ws_ov"
CONFIG_DIR="$WS_DIR/src/open_vins/config/euroc_mav"
TMP_CONFIG="$CONFIG_DIR/estimator_config_timing.yaml"

SEQUENCES=(V1_01_easy MH_03_medium V1_03_difficult)

# --- Source ROS 2 ---
source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"

# --- Create temp config with timing enabled ---
cp "$CONFIG_DIR/estimator_config.yaml" "$TMP_CONFIG"
sed -i 's/^record_timing_information: false/record_timing_information: true/' "$TMP_CONFIG"
sed -i 's/^record_timing_cpu_time: false/record_timing_cpu_time: true/' "$TMP_CONFIG"
sed -i 's/^record_timing_thread_time: false/record_timing_thread_time: true/' "$TMP_CONFIG"
trap 'rm -f "$TMP_CONFIG"' EXIT

mkdir -p "$RESULTS_DIR/stereo" "$RESULTS_DIR/mono"

for mode in stereo mono; do
  if [ "$mode" = "stereo" ]; then
    CAM_ARGS="max_cameras:=2 use_stereo:=true"
  else
    CAM_ARGS="max_cameras:=1 use_stereo:=false"
  fi

  echo ""
  echo "======== $mode mode ========"
  for seq in "${SEQUENCES[@]}"; do
    OUT="$RESULTS_DIR/$mode/${seq}.txt"
    if [ -f "$OUT" ]; then
      echo "=== $seq ($mode) === SKIP (exists: $OUT)"
      continue
    fi

    echo "=== $seq ($mode) ==="
    rm -f "$TIMING_TMP" "$TIMING_TMP_CPU" "$TIMING_TMP_THREAD"

    ros2 launch ov_msckf serial.launch.py \
        config_path:="$TMP_CONFIG" \
        path_bag:="$DATASETS_DIR/$seq" \
        $CAM_ARGS

    if [ -f "$TIMING_TMP" ]; then
      ROWS=$(grep -cv '^#' "$TIMING_TMP" || true)
      cp "$TIMING_TMP" "$OUT"
      echo "  -> Saved $OUT ($ROWS frames)"
    else
      echo "  -> WARNING: no timing file produced for $seq ($mode)"
    fi
    if [ -f "$TIMING_TMP_CPU" ]; then
      cp "$TIMING_TMP_CPU" "${OUT%.txt}_cpu.txt"
      echo "  -> Saved ${OUT%.txt}_cpu.txt (process CPU time)"
    fi
    if [ -f "$TIMING_TMP_THREAD" ]; then
      cp "$TIMING_TMP_THREAD" "${OUT%.txt}_thread.txt"
      echo "  -> Saved ${OUT%.txt}_thread.txt (thread CPU time)"
    fi
  done
done

echo ""
echo "======== Done ========"
echo "Results in: $RESULTS_DIR"
echo ""
echo "Analyze with:"
echo "  ros2 run ov_eval timing_flamegraph $RESULTS_DIR/stereo/V1_01_easy.txt"
echo "  ros2 run ov_eval timing_comparison $RESULTS_DIR/stereo/V1_01_easy.txt $RESULTS_DIR/mono/V1_01_easy.txt"
echo "  ros2 run ov_eval timing_histogram $RESULTS_DIR/stereo/V1_01_easy.txt 50"
