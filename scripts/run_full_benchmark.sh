#!/usr/bin/env bash
#
# Flexible benchmark orchestrator: serial + subscribe on the EuRoC dataset.
# Supersedes run_timing_benchmark.sh and run_timing_combined.sh — their
# scopes are covered by the CLI options below. See usage() for full flags.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=bench_lib.sh
. "$SCRIPT_DIR/bench_lib.sh"

# ── Defaults ──
RESULTS_BASE="${RESULTS_BASE:-$(arch_results_base)}"
BAG_PLAY_DELAY=5  # seconds the script waits after `ros2 bag play` to flush queued msgs

MODE="both"
SEQUENCES_CSV="V1_01_easy,MH_03_medium,V2_02_medium"
THREADS_CSV="4,1"
CAMERAS="stereo"
SUBSCRIBE_REPS=5
RATES_CSV="1.0"
BENCH_TAG="bench_$(date +%Y%m%d_%H%M%S)"
SLAM_CHI2_RECOVERY=""  # empty = leave config value alone (shipping default is false)
DRY_RUN=0

usage() {
  cat <<'EOF'
Flexible benchmark orchestrator: serial + subscribe on the EuRoC dataset.

Default behavior (no options): all 3 sequences × {4-thr, 1-thr} × stereo,
serial (1 rep each) + subscribe (5 reps each). Wall-time on x86 (i7-1185G7):
~12 min serial + ~90 min subscribe ≈ 1h 45m. RPi5 is ~3.5× slower.

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
      --rate <csv>                     default: 1.0 (subscribe-mode bag-play rates)
                                       sweep dimension; e.g. --rate 1.0,2.0,5.0
                                       runs the matrix three times.
      --tag <name>                     default: bench_YYYYMMDD_HHMMSS
                                       routes output under
                                       <results-base>/{serial,subscribe}/<tag>/
      --results-base <dir>             default: $HOME/results/timing/<arch>
                                       (x86_64→x86, aarch64→rpi5; auto-detected)
      --slam-chi2-recovery <true|false>  override slam_chi2_recovery in the temp config
                                       (default: leave config's value alone)
      --dry-run                        print the cells that would run, no execution
      --quick                          shortcut: -m serial -s V1_01_easy -t 4 -c stereo -r 1
  -h, --help                           show this help and exit

Examples:
  # Full default suite
  bash scripts/run_full_benchmark.sh --tag bench_$(date +%Y%m%d)

  # Serial mode, 3 sequences, stereo+mono, 1 rep (paper Table II/III reproduction)
  bash scripts/run_full_benchmark.sh -m serial -c both -r 1 \
      -s V1_01_easy,V1_02_medium,V1_03_difficult,V2_01_easy,V2_02_medium,MH_01_easy,MH_02_easy,MH_03_medium,MH_04_difficult,MH_05_difficult \
      --tag rerun_paper

  # Subscribe mode, single sequence, 5 reps
  bash scripts/run_full_benchmark.sh -m subscribe -s V2_02_medium -r 5 --tag bench_v2_only

  # Subscribe rate-feasibility sweep (single invocation, 3 rates)
  bash scripts/run_full_benchmark.sh -m subscribe -s V1_01_easy --rate 1.0,2.0,5.0 --tag rate_sweep

  # Quick smoke test
  bash scripts/run_full_benchmark.sh --quick

Output naming:
  Serial:    <SEQ>_<N>thr[_mono]_{wall,cpu,thread,feats,est}.txt
  Subscribe: <SEQ>_<N>thr[_rate<R>]_run<R>_{wall,cpu,thread,feats,est}.txt
             (the _rate<R> segment is inserted only when R != 1.0, so existing
              rate=1.0 citations like <SEQ>_<N>thr_run<R>_*.txt are unchanged.)

Stale OpenVINS subscribe nodes (this user only) are TERM-then-KILL'd on
script start and after each subscribe rep.
EOF
}

# ── CLI parsing (supports both `--opt val` and `--opt=val`) ──
while [[ $# -gt 0 ]]; do
  case "$1" in
    -m=*|--mode=*)            MODE="${1#*=}"; shift ;;
    -m|--mode)                MODE="${2:-}"; shift 2 ;;
    -s=*|--sequences=*)       SEQUENCES_CSV="${1#*=}"; shift ;;
    -s|--sequences)           SEQUENCES_CSV="${2:-}"; shift 2 ;;
    -t=*|--threads=*)         THREADS_CSV="${1#*=}"; shift ;;
    -t|--threads)             THREADS_CSV="${2:-}"; shift 2 ;;
    -c=*|--cameras=*)         CAMERAS="${1#*=}"; shift ;;
    -c|--cameras)             CAMERAS="${2:-}"; shift 2 ;;
    -r=*|--reps=*)            SUBSCRIBE_REPS="${1#*=}"; shift ;;
    -r|--reps)                SUBSCRIBE_REPS="${2:-}"; shift 2 ;;
    --rate=*)                 RATES_CSV="${1#*=}"; shift ;;
    --rate)                   RATES_CSV="${2:-}"; shift 2 ;;
    --tag=*)                  BENCH_TAG="${1#*=}"; shift ;;
    --tag)                    BENCH_TAG="${2:-}"; shift 2 ;;
    --results-base=*)         RESULTS_BASE="${1#*=}"; shift ;;
    --results-base)           RESULTS_BASE="${2:-}"; shift 2 ;;
    --slam-chi2-recovery=*)   SLAM_CHI2_RECOVERY="${1#*=}"; shift ;;
    --slam-chi2-recovery)     SLAM_CHI2_RECOVERY="${2:-}"; shift 2 ;;
    --dry-run)                DRY_RUN=1; shift ;;
    --quick)
      MODE="serial"; SEQUENCES_CSV="V1_01_easy"; THREADS_CSV="4"
      CAMERAS="stereo"; SUBSCRIBE_REPS=1
      BENCH_TAG="smoke_$(date +%Y%m%d_%H%M%S)"
      shift ;;
    -h|--help)                usage; exit 0 ;;
    *)
      echo "ERROR: unknown option: $1" >&2
      usage >&2
      exit 2 ;;
  esac
done

# ── Validate ──
case "$MODE" in serial|subscribe|both) ;; *) echo "ERROR: --mode must be serial|subscribe|both (got: $MODE)" >&2; exit 2 ;; esac
case "$CAMERAS" in stereo|mono|both) ;; *) echo "ERROR: --cameras must be stereo|mono|both (got: $CAMERAS)" >&2; exit 2 ;; esac
validate_chi2_recovery "$SLAM_CHI2_RECOVERY" || exit 2
validate_positive_int "--reps" "$SUBSCRIBE_REPS" || exit 2
validate_nonempty "--tag" "$BENCH_TAG" || exit 2
require_dir "datasets" "$DATASETS_DIR" || exit 1
require_dir "config" "$CONFIG_DIR" || exit 1

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
IFS=',' read -ra RATES <<< "$RATES_CSV"

# Validate every requested sequence has a bag dir.
for seq in "${SEQUENCES[@]}"; do
  require_bag "$seq" || exit 1
done

# Validate every requested rate is a positive float.
for rate in "${RATES[@]}"; do
  [[ "$rate" =~ ^[0-9]+(\.[0-9]+)?$ ]] || {
    echo "ERROR: --rate token must be a positive float (got: $rate)" >&2
    exit 2
  }
done

# Camera-config pairs as "name:max_cameras:use_stereo"
CAM_CONFIGS=()
case "$CAMERAS" in
  stereo) CAM_CONFIGS=("stereo:2:true") ;;
  mono)   CAM_CONFIGS=("mono:1:false") ;;
  both)   CAM_CONFIGS=("stereo:2:true" "mono:1:false") ;;
esac

# ── Setup ──
TMP_CONFIG_4="$CONFIG_DIR/estimator_config_bench_4thr.yaml"
TMP_CONFIG_1="$CONFIG_DIR/estimator_config_bench_1thr.yaml"

cleanup() {
  rm -f "$TMP_CONFIG_4" "$TMP_CONFIG_1" 2>/dev/null || true
  kill_stale_subscribe_nodes
}
trap cleanup EXIT INT TERM

source_ros
kill_stale_subscribe_nodes

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

# Pick (or build) a temp config for the requested thread count.
get_config() {
  local THREADS="$1"
  local TMP
  if [ "$THREADS" = "1" ]; then TMP="$TMP_CONFIG_1"; else TMP="$TMP_CONFIG_4"; fi
  if [ ! -f "$TMP" ]; then
    make_bench_config "$TMP" "$SLAM_CHI2_RECOVERY" "$THREADS" || exit 1
  fi
  echo "$TMP"
}

save_results() {
  local DIR="$1" TAG="$2"
  mkdir -p "$DIR"
  [ -f "$TIMING_WALL_TMP" ]   && cp "$TIMING_WALL_TMP"   "$DIR/${TAG}_wall.txt"
  [ -f "$TIMING_CPU_TMP" ]    && cp "$TIMING_CPU_TMP"    "$DIR/${TAG}_cpu.txt"
  [ -f "$TIMING_THREAD_TMP" ] && cp "$TIMING_THREAD_TMP" "$DIR/${TAG}_thread.txt"
  [ -f "$FEATS_TMP" ]         && cp "$FEATS_TMP"         "$DIR/${TAG}_feats.txt"
  [ -f "$EST_TMP" ]           && cp "$EST_TMP"           "$DIR/${TAG}_est.txt"
}

get_slam_avg() {
  local F="$1"
  [ -f "$F" ] || { echo "?"; return; }
  python3 -c "
import csv
v=[int(r[1]) for r in csv.reader(open('$F')) if not r[0].startswith('#')]
print(f'{sum(v)/len(v):.1f}') if v else print('?')
" 2>/dev/null || echo "?"
}

echo "================================================================"
echo "  OpenVINS Benchmark"
echo "  Arch:      $(uname -m)  →  RESULTS_BASE=$RESULTS_BASE"
echo "  Mode:      $MODE"
echo "  Sequences: ${SEQUENCES[*]}"
echo "  Threads:   ${THREADS[*]}"
echo "  Rates:     ${RATES[*]}"
echo "  Cameras:   $CAMERAS"
echo "  Sub reps:  $SUBSCRIBE_REPS"
echo "  Tag:       $BENCH_TAG"
echo "  Date:      $(date)"
[ "$DRY_RUN" -eq 1 ] && echo "  DRY-RUN: planning only, no execution"
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
        if run_complete "$DIR" "$TAG"; then
          echo "SKIP serial $seq ${thr}-thr $CAM_NAME (already complete)"
          continue
        fi
        if [ "$DRY_RUN" -eq 1 ]; then
          echo "DRY-RUN serial $seq ${thr}-thr $CAM_NAME → $DIR/${TAG}_*.txt"
          continue
        fi
        CFG=$(get_config "$thr")
        rm -f "$TIMING_WALL_TMP" "$TIMING_CPU_TMP" "$TIMING_THREAD_TMP" "$FEATS_TMP" "$EST_TMP" "$STD_TMP"
        echo "--- serial $seq ${thr}-thr $CAM_NAME ---"
        ros2 launch ov_msckf serial.launch.py \
            config_path:="$CFG" \
            path_bag:="$DATASETS_DIR/$seq" \
            max_cameras:="$CAM_N" use_stereo:="$CAM_STEREO" \
            save_total_state:=true \
            filepath_est:="$EST_TMP" \
            filepath_std:="$STD_TMP" 2>&1 | tail -1 || {
          echo "  -> launch FAILED" >&2
          continue
        }
        save_results "$DIR" "$TAG"
        if ! run_complete "$DIR" "$TAG"; then
          echo "  -> WARNING: produced incomplete results (< $MIN_ROWS_FOR_COMPLETE_RUN frames)" >&2
        fi
        ROWS=$(grep -cv '^#' "$DIR/${TAG}_wall.txt" 2>/dev/null || echo 0)
        SLAM=$(get_slam_avg "$DIR/${TAG}_feats.txt")
        echo "  -> ${ROWS} frames, avg SLAM=$SLAM"
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
      for rate in "${RATES[@]}"; do
        # Rate=1.0 produces the historic filename layout <seq>_<thr>thr_run<N>_*;
        # other rates inject _rate<R> so existing rate=1.0 citations stay valid.
        if [ "$rate" = "1.0" ]; then
          rate_suffix=""
        else
          rate_suffix="_rate${rate}"
        fi
        for i in $(seq 1 "$SUBSCRIBE_REPS"); do
          DIR="$RESULTS_BASE/subscribe/$BENCH_TAG"
          TAG="${seq}_${thr}thr${rate_suffix}_run${i}"
          if run_complete "$DIR" "$TAG"; then
            echo "SKIP subscribe $seq ${thr}-thr rate=$rate run $i (already complete)"
            continue
          fi
          if [ "$DRY_RUN" -eq 1 ]; then
            echo "DRY-RUN subscribe $seq ${thr}-thr rate=$rate run $i → $DIR/${TAG}_*.txt"
            continue
          fi
          CFG=$(get_config "$thr")
          rm -f "$TIMING_WALL_TMP" "$TIMING_CPU_TMP" "$TIMING_THREAD_TMP" "$FEATS_TMP" "$EST_TMP" "$STD_TMP"
          echo "--- subscribe $seq ${thr}-thr rate=$rate run $i / $SUBSCRIBE_REPS ---"

          ros2 launch ov_msckf subscribe.launch.py \
              config_path:="$CFG" \
              max_cameras:=2 use_stereo:=true \
              save_total_state:=true \
              filepath_est:="$EST_TMP" \
              filepath_std:="$STD_TMP" &>/dev/null &
          OV_PID=$!
          sleep 3
          ros2 bag play "$DATASETS_DIR/$seq" --rate "$rate" -d "$BAG_PLAY_DELAY" &>/dev/null
          sleep 5
          kill "$OV_PID" 2>/dev/null || true
          wait "$OV_PID" 2>/dev/null || true
          kill_stale_subscribe_nodes

          save_results "$DIR" "$TAG"
          if ! run_complete "$DIR" "$TAG"; then
            echo "  -> WARNING: produced incomplete results (< $MIN_ROWS_FOR_COMPLETE_RUN frames)" >&2
          fi
          ROWS=$(grep -cv '^#' "$DIR/${TAG}_wall.txt" 2>/dev/null || echo 0)
          SLAM=$(get_slam_avg "$DIR/${TAG}_feats.txt")
          echo "  -> ${ROWS} frames, avg SLAM=$SLAM"
        done
      done
    done
  done
  echo ""
  fi
fi

[ "$DRY_RUN" -eq 1 ] && { echo "DRY-RUN complete — no data written."; exit 0; }

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
        TOTAL=$(ros2 run ov_eval timing_flamegraph "$F" 2>/dev/null | grep "(total)" | grep -oP 'mean_time = \K[0-9.]+' || true)
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
      for rate in "${RATES[@]}"; do
        if [ "$rate" = "1.0" ]; then rate_suffix=""; else rate_suffix="_rate${rate}"; fi
        VALS=""
        for i in $(seq 1 "$SUBSCRIBE_REPS"); do
          F="$RESULTS_BASE/subscribe/$BENCH_TAG/${seq}_${thr}thr${rate_suffix}_run${i}_wall.txt"
          [ -f "$F" ] || continue
          T=$(ros2 run ov_eval timing_flamegraph "$F" 2>/dev/null | grep "(total)" | grep -oP 'mean_time = \K[0-9.]+' || true)
          T_MS=$(python3 -c "print(f'{float(\"${T:-0}\")*1000:.1f}')")
          VALS="$VALS $T_MS"
        done
        if [ "$rate" = "1.0" ]; then
          echo "    ${thr}-thr:$VALS ms"
        else
          echo "    ${thr}-thr rate=${rate}:$VALS ms"
        fi
      done
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
      for rate in "${RATES[@]}"; do
        if [ "$rate" = "1.0" ]; then rate_suffix=""; else rate_suffix="_rate${rate}"; fi
        VALS=""
        for i in $(seq 1 "$SUBSCRIBE_REPS"); do
          SLAM=$(get_slam_avg "$RESULTS_BASE/subscribe/$BENCH_TAG/${seq}_${thr}thr${rate_suffix}_run${i}_feats.txt")
          VALS="$VALS $SLAM"
        done
        if [ "$rate" = "1.0" ]; then
          echo "    subscribe ${thr}-thr:$VALS"
        else
          echo "    subscribe ${thr}-thr rate=${rate}:$VALS"
        fi
      done
    done
  fi
done
echo ""

# ── ATE ──
GT_DIR="$WS_DIR/src/open_vins/ov_data/euroc_mav"
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
      for rate in "${RATES[@]}"; do
        if [ "$rate" = "1.0" ]; then rate_suffix=""; else rate_suffix="_rate${rate}"; fi
        VALS=""
        for i in $(seq 1 "$SUBSCRIBE_REPS"); do
          F="$RESULTS_BASE/subscribe/$BENCH_TAG/${seq}_${thr}thr${rate_suffix}_run${i}_est.txt"
          [ -f "$F" ] || continue
          ATE=$(ros2 run ov_eval error_singlerun posyaw "$GT" "$F" 2>/dev/null | grep "rmse_.*pos" | head -1 | grep -oP 'rmse_pos = \K[0-9.]+' || true)
          VALS="$VALS ${ATE:-FAIL}"
        done
        if [ "$rate" = "1.0" ]; then
          echo "    subscribe ${thr}-thr:$VALS"
        else
          echo "    subscribe ${thr}-thr rate=${rate}:$VALS"
        fi
      done
    done
  fi
done
echo ""

# ── Zombie check ──
ZOMBIES=$(pgrep -u "$USER" -cf "run_subscribe_msckf" 2>/dev/null || echo 0)
echo "--- Zombie check (this user only): ${ZOMBIES} stale processes ---"
echo ""
echo "================================================================"
echo "  Done — $(date)"
echo "================================================================"
