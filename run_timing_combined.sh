#!/usr/bin/env bash
#
# Combined timing + accuracy + feature count benchmark for subscribe mode.
# Uses -d 5 bag play delay to fix DDS discovery race.
# Collects wall timing, feature counts, and trajectory estimates per run.
#
# Usage:
#   bash run_timing_combined.sh [sequence] [reps]
#   bash run_timing_combined.sh V2_02_medium 5

set -eo pipefail

# Kill any stale OpenVINS subscribe processes from previous runs.
# Stale nodes on the same DDS domain steal messages and corrupt results.
cleanup_stale_processes() {
  pkill -9 -f "run_subscribe_msckf" 2>/dev/null || true
  sleep 1
}
cleanup_stale_processes

SEQ="${1:-V2_02_medium}"
REPS="${2:-5}"
DATASETS_DIR="$HOME/datasets/euroc"
RESULTS_BASE="$HOME/results/timing/x86"
WS_DIR="$HOME/workspace/catkin_ws_ov"
CONFIG_DIR="$WS_DIR/src/open_vins/config/euroc_mav"
GT_REF="$WS_DIR/src/open_vins/ov_data/euroc_mav/${SEQ}.txt"
BAG_PLAY_DELAY=5  # seconds for DDS discovery before playback starts

WALL_TMP="/tmp/traj_timing.txt"
FEATS_TMP="/tmp/traj_features.txt"
EST_TMP="/tmp/ov_estimate.txt"
STD_TMP="/tmp/ov_estimate_std.txt"

source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"

make_config() {
  local THREADS="$1"
  local SLAM_DELAY="$2"
  local TMP="$CONFIG_DIR/estimator_config_combined.yaml"
  cp "$CONFIG_DIR/estimator_config.yaml" "$TMP"
  sed -i 's/^record_timing_information: false/record_timing_information: true/' "$TMP"
  sed -i 's/^record_feature_counts: false/record_feature_counts: true/' "$TMP"
  if [ "$THREADS" = "1" ]; then
    sed -i 's/^num_opencv_threads: 4/num_opencv_threads: 1/' "$TMP"
  fi
  if [ -n "$SLAM_DELAY" ] && [ "$SLAM_DELAY" != "1" ]; then
    sed -i "s/^dt_slam_delay: 1/dt_slam_delay: $SLAM_DELAY/" "$TMP"
  fi
  echo "$TMP"
}

cleanup_config() {
  rm -f "$CONFIG_DIR/estimator_config_combined.yaml"
}
trap cleanup_config EXIT

save_results() {
  local DIR="$1"
  local TAG="$2"
  mkdir -p "$DIR"
  [ -f "$WALL_TMP" ]  && cp "$WALL_TMP"  "$DIR/${TAG}_wall.txt"
  [ -f "$FEATS_TMP" ] && cp "$FEATS_TMP" "$DIR/${TAG}_feats.txt"
  [ -f "$EST_TMP" ]   && cp "$EST_TMP"   "$DIR/${TAG}_est.txt"
}

check_skip() {
  local DIR="$1"
  local TAG="$2"
  [ -f "$DIR/${TAG}_wall.txt" ] && [ -f "$DIR/${TAG}_feats.txt" ]
}

report_run() {
  local DIR="$1"
  local TAG="$2"
  local WALL="$DIR/${TAG}_wall.txt"
  local FEATS="$DIR/${TAG}_feats.txt"
  if [ -f "$WALL" ]; then
    local ROWS=$(grep -cv '^#' "$WALL" || true)
    local AVG_SLAM="?"
    if [ -f "$FEATS" ]; then
      AVG_SLAM=$(python3 -c "
import csv
v=[]
with open('$FEATS') as f:
    for r in csv.reader(f):
        if r[0].startswith('#'): continue
        v.append(int(r[1]))
print(f'{sum(v)/len(v):.1f}') if v else print('?')
" 2>/dev/null || echo "?")
    fi
    local HAS_EST="no"
    [ -f "$DIR/${TAG}_est.txt" ] && HAS_EST="yes"
    echo "  -> $ROWS frames, avg SLAM=$AVG_SLAM, trajectory=$HAS_EST"
  else
    echo "  -> WARNING: no timing file"
  fi
}

# ── Serial reference ──
run_serial() {
  local THREADS="$1"
  local DIR="$RESULTS_BASE/serial/v2"
  local TAG="${SEQ}_${THREADS}thr"
  if check_skip "$DIR" "$TAG"; then
    echo "SKIP serial ${THREADS}-thr"
    return
  fi
  local CFG=$(make_config "$THREADS")
  rm -f "$WALL_TMP" "$FEATS_TMP" "$EST_TMP" "$STD_TMP"
  echo "--- serial ${THREADS}-thr ---"
  ros2 launch ov_msckf serial.launch.py \
      config_path:="$CFG" \
      path_bag:="$DATASETS_DIR/$SEQ" \
      max_cameras:=2 use_stereo:=true \
      save_total_state:=true \
      filepath_est:="$EST_TMP" \
      filepath_std:="$STD_TMP" 2>&1 | tail -1
  save_results "$DIR" "$TAG"
  report_run "$DIR" "$TAG"
}

# ── Subscribe runs ──
run_subscribe() {
  local THREADS="$1"
  local SLAM_DELAY="$2"
  local DIR_SUFFIX="${3:-v2}"
  local DIR="$RESULTS_BASE/subscribe/$DIR_SUFFIX"
  local CFG=$(make_config "$THREADS" "$SLAM_DELAY")

  for i in $(seq 1 $REPS); do
    local TAG="${SEQ}_${THREADS}thr_run${i}"
    if check_skip "$DIR" "$TAG"; then
      echo "SKIP subscribe ${THREADS}-thr run $i"
      continue
    fi
    rm -f "$WALL_TMP" "$FEATS_TMP" "$EST_TMP" "$STD_TMP"
    echo "--- subscribe ${THREADS}-thr run $i / $REPS ---"

    ros2 launch ov_msckf subscribe.launch.py \
        config_path:="$CFG" \
        save_total_state:=true \
        filepath_est:="$EST_TMP" \
        filepath_std:="$STD_TMP" &>/dev/null &
    local OV_PID=$!

    # Wait for subscriber to be ready
    sleep 3

    # Play bag with delay for DDS discovery (full bag, no data skipped)
    ros2 bag play "$DATASETS_DIR/$SEQ" --rate 1.0 -d "$BAG_PLAY_DELAY" &>/dev/null

    # Wait for VIO to drain remaining queued messages
    sleep 5

    kill $OV_PID 2>/dev/null || true
    wait $OV_PID 2>/dev/null || true
    # Ensure no child processes survive (prevents zombie contamination)
    cleanup_stale_processes

    save_results "$DIR" "$TAG"
    report_run "$DIR" "$TAG"
  done
}

echo "========================================================"
echo "  Combined timing + accuracy benchmark"
echo "  Sequence: $SEQ, Reps: $REPS, Bag delay: ${BAG_PLAY_DELAY}s"
echo "========================================================"
echo ""

# Serial references (one run each, deterministic)
echo "========== Serial references =========="
run_serial 4
run_serial 1
echo ""

# Subscribe runs
echo "========== Subscribe 4-thread =========="
run_subscribe 4
echo ""
echo "========== Subscribe 1-thread =========="
run_subscribe 1
echo ""

# ── Analysis ──
echo "========================================================"
echo "  Analysis"
echo "========================================================"
echo ""

# Starting timestamps
echo "--- Starting timestamps (subscribe) ---"
for thr in 4 1; do
  echo "  ${thr}-thr:"
  for i in $(seq 1 $REPS); do
    F="$RESULTS_BASE/subscribe/v2/${SEQ}_${thr}thr_run${i}_wall.txt"
    [ -f "$F" ] || continue
    TS=$(head -2 "$F" | tail -1 | cut -d',' -f1)
    echo "    run $i: $TS"
  done
done

# Serial reference timestamp
for thr in 4 1; do
  F="$RESULTS_BASE/serial/v2/${SEQ}_${thr}thr_wall.txt"
  [ -f "$F" ] || continue
  TS=$(head -2 "$F" | tail -1 | cut -d',' -f1)
  echo "  serial ${thr}-thr: $TS"
done

echo ""

# Timing summary
echo "--- Timing totals ---"
for mode in serial subscribe; do
  DIR="$RESULTS_BASE/$mode/v2"
  for f in "$DIR"/${SEQ}_*_wall.txt; do
    [ -f "$f" ] || continue
    TAG=$(basename "$f" _wall.txt)
    TOTAL=$(ros2 run ov_eval timing_flamegraph "$f" 2>/dev/null | grep "(total)" | grep -oP 'mean_time = \K[0-9.]+')
    [ -n "$TOTAL" ] && TOTAL_MS=$(python3 -c "print(f'{float(\"$TOTAL\")*1000:.1f}')") || TOTAL_MS="?"
    echo "  $TAG: ${TOTAL_MS}ms"
  done
done

echo ""

# ATE if ground truth exists
if [ -f "$GT_REF" ]; then
  echo "--- ATE (posyaw alignment) ---"
  for mode in serial subscribe; do
    DIR="$RESULTS_BASE/$mode/v2"
    for f in "$DIR"/${SEQ}_*_est.txt; do
      [ -f "$f" ] || continue
      TAG=$(basename "$f" _est.txt)
      ATE_LINE=$(ros2 run ov_eval error_singlerun posyaw "$GT_REF" "$f" 2>/dev/null | grep "rmse_.*pos" | head -1)
      if [ -n "$ATE_LINE" ]; then
        echo "  $TAG: $ATE_LINE"
      else
        echo "  $TAG: FAILED (diverged)"
      fi
    done
  done
fi

echo ""
echo "========== Done =========="
