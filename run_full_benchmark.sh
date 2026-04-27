#!/usr/bin/env bash
#
# Flexible benchmark orchestrator: serial + subscribe on the EuRoC dataset.
# Supersedes run_timing_benchmark.sh and run_timing_combined.sh — their
# scopes are covered by the CLI options below. See usage() for full flags.

set -u -o pipefail

# ── Defaults ──
WS_DIR="$HOME/workspace/catkin_ws_ov"
DATASETS_DIR="$HOME/datasets/euroc"
RESULTS_BASE="$HOME/results/timing/x86"
CONFIG_DIR="$WS_DIR/src/open_vins/config/euroc_mav"
GT_DIR="$WS_DIR/src/open_vins/ov_data/euroc_mav"
BAG_PLAY_DELAY=5

MODE="both"
SEQUENCES_CSV="V1_01_easy,MH_03_medium,V2_02_medium"
THREADS_CSV="4,1"
CAMERAS="stereo"
SUBSCRIBE_REPS=5
BENCH_TAG="bench_$(date +%Y%m%d_%H%M%S)"
SLAM_CHI2_RECOVERY=""  # empty = leave config value alone (shipping default is false)

usage() {
  cat <<'EOF'
Flexible benchmark orchestrator: serial + subscribe on the EuRoC dataset.

Default behavior (no options): all 3 sequences × {4-thr, 1-thr} × stereo,
serial (1 rep each) + subscribe (5 reps each). ~2.5 hours total.

Usage:
  bash run_full_benchmark.sh [OPTIONS]

Options:
  -m, --mode <serial|subscribe|both>   default: both
  -s, --sequences <csv>                default: V1_01_easy,MH_03_medium,V2_02_medium
  -t, --threads <csv>                  default: 4,1
  -c, --cameras <stereo|mono|both>     default: stereo
                                       (mono is only measured in serial mode;
                                        mono+subscribe is skipped)
  -r, --reps <N>                       default: 5 (subscribe reps; serial is always 1)
      --tag <name>                     default: bench_YYYYMMDD_HHMMSS
      --results-base <dir>             default: $HOME/results/timing/x86
      --slam-chi2-recovery <true|false>  override slam_chi2_recovery in the temp config
                                       (default: leave config's value alone)
      --quick                          shortcut: -m serial -s V1_01_easy -t 4 -c stereo -r 1
  -h, --help                           show this help and exit

Examples:
  # Full default suite
  bash run_full_benchmark.sh

  # Serial mode, 3 sequences, stereo+mono, 1 rep (old run_timing_benchmark.sh)
  bash run_full_benchmark.sh -m serial -c both -r 1

  # Subscribe mode, single sequence, 5 reps (old run_timing_combined.sh)
  bash run_full_benchmark.sh -m subscribe -s V2_02_medium -r 5

  # Quick smoke test
  bash run_full_benchmark.sh --quick
EOF
}

# ── CLI parsing ──
while [[ $# -gt 0 ]]; do
  case "$1" in
    -m|--mode)         MODE="$2"; shift 2 ;;
    -s|--sequences)    SEQUENCES_CSV="$2"; shift 2 ;;
    -t|--threads)      THREADS_CSV="$2"; shift 2 ;;
    -c|--cameras)      CAMERAS="$2"; shift 2 ;;
    -r|--reps)         SUBSCRIBE_REPS="$2"; shift 2 ;;
    --tag)             BENCH_TAG="$2"; shift 2 ;;
    --results-base)    RESULTS_BASE="$2"; shift 2 ;;
    --slam-chi2-recovery) SLAM_CHI2_RECOVERY="$2"; shift 2 ;;
    --quick)
      MODE="serial"; SEQUENCES_CSV="V1_01_easy"; THREADS_CSV="4"
      CAMERAS="stereo"; SUBSCRIBE_REPS=1
      BENCH_TAG="smoke_$(date +%Y%m%d_%H%M%S)"
      shift ;;
    -h|--help)         usage; exit 0 ;;
    *)
      echo "ERROR: unknown option: $1" >&2
      usage >&2
      exit 2 ;;
  esac
done

# ── Validate ──
case "$MODE" in serial|subscribe|both) ;; *) echo "ERROR: --mode must be serial|subscribe|both (got: $MODE)" >&2; exit 2 ;; esac
case "$CAMERAS" in stereo|mono|both) ;; *) echo "ERROR: --cameras must be stereo|mono|both (got: $CAMERAS)" >&2; exit 2 ;; esac
case "${SLAM_CHI2_RECOVERY:-}" in ""|true|false) ;; *) echo "ERROR: --slam-chi2-recovery must be true or false (got: $SLAM_CHI2_RECOVERY)" >&2; exit 2 ;; esac
# Subscribe mode only runs stereo. Fail fast on pure mono+subscribe; warn on both+subscribe.
if [[ "$MODE" == "subscribe" && "$CAMERAS" == "mono" ]]; then
  echo "ERROR: --cameras mono --mode subscribe is not supported (mono+subscribe is untested)." >&2
  echo "       Use --cameras stereo, or --mode serial if you need mono measurements." >&2
  exit 2
fi
if [[ "$CAMERAS" == "both" && ( "$MODE" == "subscribe" || "$MODE" == "both" ) ]]; then
  echo "NOTE: subscribe mode will run stereo only; mono+subscribe is skipped." >&2
fi

IFS=',' read -ra SEQUENCES <<< "$SEQUENCES_CSV"
IFS=',' read -ra THREADS <<< "$THREADS_CSV"

# Camera-config pairs as "name:max_cameras:use_stereo"
CAM_CONFIGS=()
case "$CAMERAS" in
  stereo) CAM_CONFIGS=("stereo:2:true") ;;
  mono)   CAM_CONFIGS=("mono:1:false") ;;
  both)   CAM_CONFIGS=("stereo:2:true" "mono:1:false") ;;
esac

WALL_TMP="/tmp/traj_timing.txt"
CPU_TMP="/tmp/traj_timing_cpu.txt"
THREAD_TMP="/tmp/traj_timing_thread.txt"
FEATS_TMP="/tmp/traj_features.txt"
EST_TMP="/tmp/ov_estimate.txt"
STD_TMP="/tmp/ov_estimate_std.txt"

# ROS setup scripts reference unset vars on first load; guard them from set -u.
set +u
for _distro in jazzy humble; do
  if [ -f "/opt/ros/$_distro/setup.bash" ]; then source "/opt/ros/$_distro/setup.bash"; break; fi
done
source "$WS_DIR/install/setup.bash"
set -u

cleanup_stale() {
  pkill -9 -f "run_subscribe_msckf" 2>/dev/null || true
  sleep 1
}
cleanup_stale

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
  if [ -n "${SLAM_CHI2_RECOVERY:-}" ]; then
    sed -i "s/^slam_chi2_recovery: .*/slam_chi2_recovery: ${SLAM_CHI2_RECOVERY} # overridden via --slam-chi2-recovery/" "$TMP"
  fi
  echo "$TMP"
}
trap 'rm -f "$CONFIG_DIR/estimator_config_bench.yaml"' EXIT

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

# Compose per-mode tag suffix. Stereo keeps the historical naming
# (no camera suffix) so earlier results directories continue to match.
tag_for() {
  local SEQ="$1" THR="$2" CAM_NAME="$3"
  if [ "$CAM_NAME" = "stereo" ]; then
    echo "${SEQ}_${THR}thr"
  else
    echo "${SEQ}_${THR}thr_${CAM_NAME}"
  fi
}

echo "================================================================"
echo "  OpenVINS Benchmark"
echo "  Mode:      $MODE"
echo "  Sequences: ${SEQUENCES[*]}"
echo "  Threads:   ${THREADS[*]}"
echo "  Cameras:   $CAMERAS"
echo "  Sub reps:  $SUBSCRIBE_REPS"
echo "  Tag:       $BENCH_TAG"
echo "  Date:      $(date)"
echo "================================================================"
echo ""

# ══════════════════════════════════════════════════════════════════════
# SERIAL: one run per sequence × thread × camera (deterministic)
# ══════════════════════════════════════════════════════════════════════
if [[ "$MODE" == "serial" || "$MODE" == "both" ]]; then
  echo "==================== SERIAL MODE ===================="
  for seq in "${SEQUENCES[@]}"; do
    for thr in "${THREADS[@]}"; do
      for cam_spec in "${CAM_CONFIGS[@]}"; do
        IFS=':' read -r CAM_NAME CAM_N CAM_STEREO <<< "$cam_spec"
        DIR="$RESULTS_BASE/serial/$BENCH_TAG"
        TAG="$(tag_for "$seq" "$thr" "$CAM_NAME")"
        if skip_check "$DIR" "$TAG"; then
          echo "SKIP serial $seq ${thr}-thr $CAM_NAME"
          continue
        fi
        CFG=$(make_config "$thr")
        rm -f "$WALL_TMP" "$CPU_TMP" "$THREAD_TMP" "$FEATS_TMP" "$EST_TMP" "$STD_TMP"
        echo "--- serial $seq ${thr}-thr $CAM_NAME ---"
        ros2 launch ov_msckf serial.launch.py \
            config_path:="$CFG" \
            path_bag:="$DATASETS_DIR/$seq" \
            max_cameras:="$CAM_N" use_stereo:="$CAM_STEREO" \
            save_total_state:=true \
            filepath_est:="$EST_TMP" \
            filepath_std:="$STD_TMP" 2>&1 | tail -1
        save_results "$DIR" "$TAG"
        ROWS=$(grep -cv '^#' "$DIR/${TAG}_wall.txt" 2>/dev/null || true)
        SLAM=$(get_slam_avg "$DIR/${TAG}_feats.txt")
        echo "  -> ${ROWS:-0} frames, avg SLAM=$SLAM"
      done
    done
  done
  echo ""
fi

# ══════════════════════════════════════════════════════════════════════
# SUBSCRIBE: N reps per sequence × thread (stereo only — mono untested)
# ══════════════════════════════════════════════════════════════════════
if [[ "$MODE" == "subscribe" || "$MODE" == "both" ]]; then
  if [[ "$CAMERAS" == "mono" ]]; then
    echo "NOTE: mono+subscribe is not supported by this script — skipping subscribe block."
    echo ""
  else
  echo "==================== SUBSCRIBE MODE ===================="
  for seq in "${SEQUENCES[@]}"; do
    for thr in "${THREADS[@]}"; do
      CFG=$(make_config "$thr")
      for i in $(seq 1 "$SUBSCRIBE_REPS"); do
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
            max_cameras:=2 use_stereo:=true \
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
        ROWS=$(grep -cv '^#' "$DIR/${TAG}_wall.txt" 2>/dev/null || true)
        SLAM=$(get_slam_avg "$DIR/${TAG}_feats.txt")
        echo "  -> ${ROWS:-0} frames, avg SLAM=$SLAM"
      done
    done
  done
  echo ""
  fi
fi

# ══════════════════════════════════════════════════════════════════════
# ANALYSIS
# ══════════════════════════════════════════════════════════════════════
echo "================================================================"
echo "  RESULTS SUMMARY ($BENCH_TAG)"
echo "================================================================"
echo ""

# ── Timing ──
if [[ "$MODE" == "serial" || "$MODE" == "both" ]]; then
  echo "--- Per-component timing (serial, mean total in ms) ---"
  for seq in "${SEQUENCES[@]}"; do
    echo "  $seq:"
    for thr in "${THREADS[@]}"; do
      for cam_spec in "${CAM_CONFIGS[@]}"; do
        IFS=':' read -r CAM_NAME _ _ <<< "$cam_spec"
        TAG="$(tag_for "$seq" "$thr" "$CAM_NAME")"
        F="$RESULTS_BASE/serial/$BENCH_TAG/${TAG}_wall.txt"
        [ -f "$F" ] || continue
        TOTAL=$(ros2 run ov_eval timing_flamegraph "$F" 2>/dev/null | grep "(total)" | grep -oP 'mean_time = \K[0-9.]+')
        TOTAL_MS=$(python3 -c "print(f'{float(\"${TOTAL:-0}\")*1000:.1f}')")
        echo "    ${thr}-thr ${CAM_NAME}: ${TOTAL_MS}ms"
      done
    done
  done
  echo ""
fi

if [[ "$MODE" == "subscribe" || "$MODE" == "both" ]]; then
  echo "--- Subscribe timing consistency (mean total in ms) ---"
  for seq in "${SEQUENCES[@]}"; do
    echo "  $seq:"
    for thr in "${THREADS[@]}"; do
      VALS=""
      for i in $(seq 1 "$SUBSCRIBE_REPS"); do
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
fi

# ── SLAM health ──
echo "--- SLAM feature health (avg features in state) ---"
for seq in "${SEQUENCES[@]}"; do
  echo "  $seq:"
  if [[ "$MODE" == "serial" || "$MODE" == "both" ]]; then
    for thr in "${THREADS[@]}"; do
      for cam_spec in "${CAM_CONFIGS[@]}"; do
        IFS=':' read -r CAM_NAME _ _ <<< "$cam_spec"
        TAG="$(tag_for "$seq" "$thr" "$CAM_NAME")"
        SLAM=$(get_slam_avg "$RESULTS_BASE/serial/$BENCH_TAG/${TAG}_feats.txt")
        echo "    serial ${thr}-thr ${CAM_NAME}: $SLAM"
      done
    done
  fi
  if [[ "$MODE" == "subscribe" || "$MODE" == "both" ]]; then
    for thr in "${THREADS[@]}"; do
      VALS=""
      for i in $(seq 1 "$SUBSCRIBE_REPS"); do
        SLAM=$(get_slam_avg "$RESULTS_BASE/subscribe/$BENCH_TAG/${seq}_${thr}thr_run${i}_feats.txt")
        VALS="$VALS $SLAM"
      done
      echo "    subscribe ${thr}-thr:$VALS"
    done
  fi
done
echo ""

# ── ATE ──
echo "--- ATE position RMSE (meters, posyaw alignment) ---"
for seq in "${SEQUENCES[@]}"; do
  GT="$GT_DIR/${seq}.txt"
  [ -f "$GT" ] || continue
  echo "  $seq:"
  if [[ "$MODE" == "serial" || "$MODE" == "both" ]]; then
    for thr in "${THREADS[@]}"; do
      for cam_spec in "${CAM_CONFIGS[@]}"; do
        IFS=':' read -r CAM_NAME _ _ <<< "$cam_spec"
        TAG="$(tag_for "$seq" "$thr" "$CAM_NAME")"
        F="$RESULTS_BASE/serial/$BENCH_TAG/${TAG}_est.txt"
        [ -f "$F" ] || continue
        ATE=$(ros2 run ov_eval error_singlerun posyaw "$GT" "$F" 2>/dev/null | grep "rmse_.*pos" | head -1 | grep -oP 'rmse_pos = \K[0-9.]+' || true)
        echo "    serial ${thr}-thr ${CAM_NAME}: ${ATE:-FAILED}"
      done
    done
  fi
  if [[ "$MODE" == "subscribe" || "$MODE" == "both" ]]; then
    for thr in "${THREADS[@]}"; do
      VALS=""
      for i in $(seq 1 "$SUBSCRIBE_REPS"); do
        F="$RESULTS_BASE/subscribe/$BENCH_TAG/${seq}_${thr}thr_run${i}_est.txt"
        [ -f "$F" ] || continue
        ATE=$(ros2 run ov_eval error_singlerun posyaw "$GT" "$F" 2>/dev/null | grep "rmse_.*pos" | head -1 | grep -oP 'rmse_pos = \K[0-9.]+' || true)
        VALS="$VALS ${ATE:-FAIL}"
      done
      echo "    subscribe ${thr}-thr:$VALS"
    done
  fi
done
echo ""

# ── Zombie check ──
ZOMBIES=$(pgrep -cf "run_subscribe_msckf" 2>/dev/null || true)
echo "--- Zombie check: ${ZOMBIES:-0} stale processes ---"
echo ""
echo "================================================================"
echo "  Done — $(date)"
echo "================================================================"
