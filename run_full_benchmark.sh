#!/usr/bin/env bash
#
# Full benchmark: serial + subscribe on multiple EuRoC sequences.
# Validates timing determinism, accuracy, SLAM stability, and realtime feasibility.
#
# Runs:
#   - Serial: 3 sequences x {stereo 4-thr, stereo 1-thr} = 6 runs (deterministic, 1 rep each)
#   - Subscribe: 3 sequences x {stereo 4-thr, stereo 1-thr} x 3 reps = 18 runs
#   - V2_02 paper comparison: serial + subscribe, 4-thr and 1-thr (covered above)
#
# Total: 24 runs (~45 min serial + ~90 min subscribe = ~2.5 hours)
#
# Usage:
#   bash run_full_benchmark.sh [reps] [tag]
#   bash run_full_benchmark.sh 5 bench_v3

set -o pipefail

WS_DIR="$HOME/workspace/catkin_ws_ov"
DATASETS_DIR="$HOME/datasets/euroc"
RESULTS_BASE="$HOME/results/timing/x86"
CONFIG_DIR="$WS_DIR/src/open_vins/config/euroc_mav"
GT_DIR="$WS_DIR/src/open_vins/ov_data/euroc_mav"
BAG_PLAY_DELAY=5

WALL_TMP="/tmp/traj_timing.txt"
FEATS_TMP="/tmp/traj_features.txt"
EST_TMP="/tmp/ov_estimate.txt"
STD_TMP="/tmp/ov_estimate_std.txt"

SEQUENCES=(V1_01_easy MH_03_medium V2_02_medium)
SUBSCRIBE_REPS=${1:-5}
BENCH_TAG=${2:-bench}

source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"

# ── Zombie cleanup ──
cleanup_stale() {
  pkill -9 -f "run_subscribe_msckf" 2>/dev/null || true
  sleep 1
}
cleanup_stale

# ── Config helpers ──
make_config() {
  local THREADS="$1"
  local TMP="$CONFIG_DIR/estimator_config_bench.yaml"
  cp "$CONFIG_DIR/estimator_config.yaml" "$TMP"
  sed -i 's/^record_timing_information: false/record_timing_information: true/' "$TMP"
  sed -i 's/^record_timing_cpu_time: false/record_timing_cpu_time: true/' "$TMP"
  sed -i 's/^record_timing_thread_time: false/record_timing_thread_time: true/' "$TMP"
  sed -i 's/^record_feature_counts: false/record_feature_counts: true/' "$TMP"
  if [ "$THREADS" = "1" ]; then
    sed -i 's/^num_opencv_threads: 4/num_opencv_threads: 1/' "$TMP"
  fi
  echo "$TMP"
}
trap 'rm -f "$CONFIG_DIR/estimator_config_bench.yaml"' EXIT

CPU_TMP="/tmp/traj_timing_cpu.txt"
THREAD_TMP="/tmp/traj_timing_thread.txt"

save_results() {
  local DIR="$1" TAG="$2"
  mkdir -p "$DIR"
  [ -f "$WALL_TMP" ]   && cp "$WALL_TMP"   "$DIR/${TAG}_wall.txt"
  [ -f "$CPU_TMP" ]    && cp "$CPU_TMP"    "$DIR/${TAG}_cpu.txt"
  [ -f "$THREAD_TMP" ] && cp "$THREAD_TMP" "$DIR/${TAG}_thread.txt"
  [ -f "$FEATS_TMP" ]  && cp "$FEATS_TMP"  "$DIR/${TAG}_feats.txt"
  [ -f "$EST_TMP" ]    && cp "$EST_TMP"    "$DIR/${TAG}_est.txt"
}

get_slam_avg() {
  local F="$1"
  [ -f "$F" ] && python3 -c "
import csv; v=[int(r[1]) for r in csv.reader(open('$F')) if not r[0].startswith('#')]
print(f'{sum(v)/len(v):.1f}') if v else print('?')
" 2>/dev/null || echo "?"
}

skip_check() {
  [ -f "$1/${2}_wall.txt" ] && [ -f "$1/${2}_feats.txt" ]
}

echo "================================================================"
echo "  Full Benchmark Suite"
echo "  Sequences: ${SEQUENCES[*]}"
echo "  Subscribe reps: $SUBSCRIBE_REPS"
echo "  Date: $(date)"
echo "================================================================"
echo ""

# ══════════════════════════════════════════════════════════════════════
# SERIAL: one run per sequence x thread config (deterministic)
# ══════════════════════════════════════════════════════════════════════
echo "==================== SERIAL MODE ===================="
for seq in "${SEQUENCES[@]}"; do
  for thr in 4 1; do
    DIR="$RESULTS_BASE/serial/$BENCH_TAG"
    TAG="${seq}_${thr}thr"
    if skip_check "$DIR" "$TAG"; then
      echo "SKIP serial $seq ${thr}-thr"
      continue
    fi
    CFG=$(make_config "$thr")
    rm -f "$WALL_TMP" "$CPU_TMP" "$THREAD_TMP" "$FEATS_TMP" "$EST_TMP" "$STD_TMP"
    echo "--- serial $seq ${thr}-thr ---"
    ros2 launch ov_msckf serial.launch.py \
        config_path:="$CFG" \
        path_bag:="$DATASETS_DIR/$seq" \
        max_cameras:=2 use_stereo:=true \
        save_total_state:=true \
        filepath_est:="$EST_TMP" \
        filepath_std:="$STD_TMP" 2>&1 | tail -1
    save_results "$DIR" "$TAG"
    ROWS=$(grep -cv '^#' "$DIR/${TAG}_wall.txt" 2>/dev/null || echo 0)
    SLAM=$(get_slam_avg "$DIR/${TAG}_feats.txt")
    echo "  -> $ROWS frames, avg SLAM=$SLAM"
  done
done
echo ""

# ══════════════════════════════════════════════════════════════════════
# SUBSCRIBE: multiple reps per sequence x thread config
# ══════════════════════════════════════════════════════════════════════
echo "==================== SUBSCRIBE MODE ===================="
for seq in "${SEQUENCES[@]}"; do
  for thr in 4 1; do
    CFG=$(make_config "$thr")
    for i in $(seq 1 $SUBSCRIBE_REPS); do
      DIR="$RESULTS_BASE/subscribe/$BENCH_TAG"
      TAG="${seq}_${thr}thr_run${i}"
      if skip_check "$DIR" "$TAG"; then
        echo "SKIP subscribe $seq ${thr}-thr run $i"
        continue
      fi
      rm -f "$WALL_TMP" "$CPU_TMP" "$THREAD_TMP" "$FEATS_TMP" "$EST_TMP" "$STD_TMP"
      echo "--- subscribe $seq ${thr}-thr run $i / $SUBSCRIBE_REPS ---"

      ros2 launch ov_msckf subscribe.launch.py \
          config_path:="$CFG" \
          save_total_state:=true \
          filepath_est:="$EST_TMP" \
          filepath_std:="$STD_TMP" &>/dev/null &
      OV_PID=$!
      sleep 3
      ros2 bag play "$DATASETS_DIR/$seq" --rate 1.0 -d "$BAG_PLAY_DELAY" &>/dev/null
      sleep 5
      kill $OV_PID 2>/dev/null || true
      wait $OV_PID 2>/dev/null || true
      cleanup_stale

      save_results "$DIR" "$TAG"
      ROWS=$(grep -cv '^#' "$DIR/${TAG}_wall.txt" 2>/dev/null || echo 0)
      SLAM=$(get_slam_avg "$DIR/${TAG}_feats.txt")
      echo "  -> $ROWS frames, avg SLAM=$SLAM"
    done
  done
done
echo ""

# ══════════════════════════════════════════════════════════════════════
# ANALYSIS
# ══════════════════════════════════════════════════════════════════════
echo "================================================================"
echo "  RESULTS SUMMARY"
echo "================================================================"
echo ""

# ── Timing ──
echo "--- Per-component timing (serial, mean total in ms) ---"
for seq in "${SEQUENCES[@]}"; do
  echo "  $seq:"
  for thr in 4 1; do
    F="$RESULTS_BASE/serial/$BENCH_TAG/${seq}_${thr}thr_wall.txt"
    [ -f "$F" ] || continue
    TOTAL=$(ros2 run ov_eval timing_flamegraph "$F" 2>/dev/null | grep "(total)" | grep -oP 'mean_time = \K[0-9.]+')
    TOTAL_MS=$(python3 -c "print(f'{float(\"${TOTAL:-0}\")*1000:.1f}')")
    echo "    ${thr}-thr: ${TOTAL_MS}ms"
  done
done
echo ""

echo "--- Subscribe timing consistency (mean total in ms) ---"
for seq in "${SEQUENCES[@]}"; do
  echo "  $seq:"
  for thr in 4 1; do
    VALS=""
    for i in $(seq 1 $SUBSCRIBE_REPS); do
      F="$RESULTS_BASE/subscribe/$BENCH_TAG/${seq}_${thr}thr_run${i}_wall.txt"
      [ -f "$F" ] || continue
      T=$(ros2 run ov_eval timing_flamegraph "$F" 2>/dev/null | grep "(total)" | grep -oP 'mean_time = \K[0-9.]+')
      T_MS=$(python3 -c "print(f'{float(\"${T:-0}\")*1000:.1f}')")
      VALS="$VALS $T_MS"
    done
    echo "    ${thr}-thr:$VALS ms"
  done
done
echo ""

# ── SLAM health ──
echo "--- SLAM feature health (avg features in state) ---"
for seq in "${SEQUENCES[@]}"; do
  echo "  $seq:"
  # Serial
  for thr in 4 1; do
    SLAM=$(get_slam_avg "$RESULTS_BASE/serial/$BENCH_TAG/${seq}_${thr}thr_feats.txt")
    echo "    serial ${thr}-thr: $SLAM"
  done
  # Subscribe
  for thr in 4 1; do
    VALS=""
    for i in $(seq 1 $SUBSCRIBE_REPS); do
      SLAM=$(get_slam_avg "$RESULTS_BASE/subscribe/$BENCH_TAG/${seq}_${thr}thr_run${i}_feats.txt")
      VALS="$VALS $SLAM"
    done
    echo "    subscribe ${thr}-thr:$VALS"
  done
done
echo ""

# ── ATE ──
echo "--- ATE position RMSE (meters, posyaw alignment) ---"
for seq in "${SEQUENCES[@]}"; do
  GT="$GT_DIR/${seq}.txt"
  [ -f "$GT" ] || continue
  echo "  $seq:"
  # Serial
  for thr in 4 1; do
    F="$RESULTS_BASE/serial/$BENCH_TAG/${seq}_${thr}thr_est.txt"
    [ -f "$F" ] || continue
    ATE=$(ros2 run ov_eval error_singlerun posyaw "$GT" "$F" 2>/dev/null | grep "rmse_.*pos" | head -1 | grep -oP 'rmse_pos = \K[0-9.]+' || true)
    echo "    serial ${thr}-thr: ${ATE:-FAILED}"
  done
  # Subscribe
  for thr in 4 1; do
    VALS=""
    for i in $(seq 1 $SUBSCRIBE_REPS); do
      F="$RESULTS_BASE/subscribe/$BENCH_TAG/${seq}_${thr}thr_run${i}_est.txt"
      [ -f "$F" ] || continue
      ATE=$(ros2 run ov_eval error_singlerun posyaw "$GT" "$F" 2>/dev/null | grep "rmse_.*pos" | head -1 | grep -oP 'rmse_pos = \K[0-9.]+' || true)
      VALS="$VALS ${ATE:-FAIL}"
    done
    echo "    subscribe ${thr}-thr:$VALS"
  done
done
echo ""

# ── RPi5 projections ──
echo "--- RPi5 projections (serial x3.5, subscribe x1.85) ---"
for seq in "${SEQUENCES[@]}"; do
  F="$RESULTS_BASE/serial/$BENCH_TAG/${seq}_4thr_wall.txt"
  [ -f "$F" ] || continue
  T=$(ros2 run ov_eval timing_flamegraph "$F" 2>/dev/null | grep "(total)" | grep -oP 'mean_time = \K[0-9.]+')
  T_MS=$(python3 -c "print(f'{float(\"${T:-0}\")*1000:.1f}')")
  RPI_SER=$(python3 -c "print(f'{float(\"${T:-0}\")*1000*3.5:.0f}')")
  RPI_SUB=$(python3 -c "print(f'{float(\"${T:-0}\")*1000*3.5*1.85:.0f}')")
  echo "  $seq: x86=${T_MS}ms -> RPi5 serial ~${RPI_SER}ms, subscribe ~${RPI_SUB}ms (budget: 50ms @20Hz)"
done
echo ""

# ── Zombie check ──
ZOMBIES=$(ps aux | grep "run_subscribe_msckf" | grep -v grep | wc -l || echo 0)
echo "--- Zombie check: $ZOMBIES stale processes ---"
echo ""
echo "================================================================"
echo "  Done — $(date)"
echo "================================================================"
