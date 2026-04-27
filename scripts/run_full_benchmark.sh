#!/usr/bin/env bash
#
# Flexible benchmark orchestrator: serial + subscribe on the EuRoC dataset.
# Single entry point — supersedes run_timing_subscribe.sh, run_pwt_benchmark_v2.sh,
# and run_pwt_final_ab.sh. Native by default; --docker <image> wraps each
# launch in a container. --rate <csv> is a sweep dimension; --slam-chi2-recovery
# overrides the YAML key in the temp config. See usage() for full flags.

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
DOCKER_IMAGE=""        # empty = run natively; set via --docker
DOCKER_FLAGS=""        # extra args passed to `docker run` (one quoted token, word-split)
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
      --docker <image>                 run the OpenVINS launches inside this Docker
                                       image (e.g. openvins-humble:latest). Native
                                       by default. Mounts $HOME/workspace,
                                       $HOME/datasets, $HOME/results.
      --docker-flags '<args>'          extra args passed through to `docker run`
                                       (one quoted token, word-split). Example:
                                       --docker-flags '--cap-add=SYS_NICE
                                                       --ulimit rtprio=99
                                                       --ulimit memlock=-1
                                                       --cpuset-cpus=0-3'
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
  Subscribe: <SEQ>_<N>thr[_rate<R>]_run<i>_{wall,cpu,thread,feats,est}.txt
             (N = threads, R = bag-play rate, i = rep index 1..reps. The
              _rate<R> segment is inserted only when R != 1.0, so existing
              rate=1.0 citations like <SEQ>_<N>thr_run<i>_*.txt are unchanged.)

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
    --docker=*)               DOCKER_IMAGE="${1#*=}"; shift ;;
    --docker)                 DOCKER_IMAGE="${2:-}"; shift 2 ;;
    --docker-flags=*)         DOCKER_FLAGS="${1#*=}"; shift ;;
    --docker-flags)           DOCKER_FLAGS="${2:-}"; shift 2 ;;
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

# Pre-flight when --docker is set: the daemon must be reachable AND the
# named image must exist locally. Catches "docker is down" and
# "image missing" before we burn N reps' worth of confusing errors.
if [ -n "$DOCKER_IMAGE" ]; then
  command -v docker >/dev/null 2>&1 || {
    echo "ERROR: --docker $DOCKER_IMAGE requested but docker is not in PATH" >&2
    exit 1
  }
  docker image inspect "$DOCKER_IMAGE" >/dev/null 2>&1 || {
    echo "ERROR: docker image '$DOCKER_IMAGE' not found locally" >&2
    echo "       (try: docker pull $DOCKER_IMAGE  or  docker build -t $DOCKER_IMAGE ...)" >&2
    exit 1
  }
fi

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

# Validate every requested rate is a positive float, then canonicalise to a
# `<int>.<frac>` form so that string comparisons against "1.0" later in the
# loop don't miss `--rate 1` / `--rate 01.00` / etc. The canonical form is
# also what gets injected into output filenames as `_rate<R>`.
for i in "${!RATES[@]}"; do
  rate="${RATES[$i]}"
  [[ "$rate" =~ ^[0-9]+(\.[0-9]+)?$ ]] || {
    echo "ERROR: --rate token must be a positive float (got: $rate)" >&2
    exit 2
  }
  awk -v r="$rate" 'BEGIN{exit !(r>0)}' || {
    echo "ERROR: --rate token must be > 0 (got: $rate)" >&2
    exit 2
  }
  RATES[$i]=$(awk -v r="$rate" 'BEGIN{printf "%g", r+0}')
  # `%g` collapses 1.0 → 1, 2.00 → 2, 5.0 → 5; we want a uniform 1-decimal
  # form so it matches the historical `_rate2.0` / `_rate5.0` filenames.
  case "${RATES[$i]}" in *.*) ;; *) RATES[$i]="${RATES[$i]}.0" ;; esac
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
  kill_stale_docker_containers
}
trap cleanup EXIT INT TERM

# Skip host-side ROS sourcing when --docker is set: the launches happen inside
# the container, which has its own /opt/ros_ws overlay. We still need
# `docker` and basic shell utilities, which are already in PATH.
if [ -z "$DOCKER_IMAGE" ]; then
  source_ros
fi
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
if [ -n "$DOCKER_IMAGE" ]; then
  echo "  Docker:    $DOCKER_IMAGE"
  [ -n "$DOCKER_FLAGS" ] && echo "  DkrFlags:  $DOCKER_FLAGS"
else
  echo "  Docker:    (native — no --docker)"
fi
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
        mkdir -p "$DIR"
        echo "--- serial $seq ${thr}-thr $CAM_NAME ---"
        # Run the launch + save in a single shell context so the cp commands
        # see the same /tmp filesystem as the launch (matters in --docker mode,
        # where /tmp is the container's /tmp, not the host's).
        if ! docker_wrap "
          rm -f '$TIMING_WALL_TMP' '$TIMING_CPU_TMP' '$TIMING_THREAD_TMP' '$FEATS_TMP' '$EST_TMP' '$STD_TMP'
          ros2 launch ov_msckf serial.launch.py \\
              config_path:='$CFG' \\
              path_bag:='$DATASETS_DIR/$seq' \\
              max_cameras:=$CAM_N use_stereo:=$CAM_STEREO \\
              save_total_state:=true \\
              filepath_est:='$EST_TMP' \\
              filepath_std:='$STD_TMP' 2>&1 | tail -1
          [ -f '$TIMING_WALL_TMP' ]   && cp '$TIMING_WALL_TMP'   '$DIR/${TAG}_wall.txt'
          [ -f '$TIMING_CPU_TMP' ]    && cp '$TIMING_CPU_TMP'    '$DIR/${TAG}_cpu.txt'
          [ -f '$TIMING_THREAD_TMP' ] && cp '$TIMING_THREAD_TMP' '$DIR/${TAG}_thread.txt'
          [ -f '$FEATS_TMP' ]         && cp '$FEATS_TMP'         '$DIR/${TAG}_feats.txt'
          [ -f '$EST_TMP' ]           && cp '$EST_TMP'           '$DIR/${TAG}_est.txt'
          true
        "; then
          echo "  -> launch FAILED" >&2
          continue
        fi
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
          mkdir -p "$DIR"
          echo "--- subscribe $seq ${thr}-thr rate=$rate run $i / $SUBSCRIBE_REPS ---"

          # Whole rep — launch + bag play + kill + save — runs in one shell
          # context so /tmp is consistent (host's in native mode, container's
          # in --docker mode; cps in the same shell see the same FS).
          docker_wrap "
            rm -f '$TIMING_WALL_TMP' '$TIMING_CPU_TMP' '$TIMING_THREAD_TMP' '$FEATS_TMP' '$EST_TMP' '$STD_TMP'
            ros2 launch ov_msckf subscribe.launch.py \\
                config_path:='$CFG' \\
                max_cameras:=2 use_stereo:=true \\
                save_total_state:=true \\
                filepath_est:='$EST_TMP' \\
                filepath_std:='$STD_TMP' >/dev/null 2>&1 &
            OV_PID=\$!
            sleep 3
            ros2 bag play '$DATASETS_DIR/$seq' --rate $rate -d $BAG_PLAY_DELAY >/dev/null 2>&1
            sleep 5
            kill \$OV_PID 2>/dev/null || true
            wait \$OV_PID 2>/dev/null || true
            [ -f '$TIMING_WALL_TMP' ]   && cp '$TIMING_WALL_TMP'   '$DIR/${TAG}_wall.txt'
            [ -f '$TIMING_CPU_TMP' ]    && cp '$TIMING_CPU_TMP'    '$DIR/${TAG}_cpu.txt'
            [ -f '$TIMING_THREAD_TMP' ] && cp '$TIMING_THREAD_TMP' '$DIR/${TAG}_thread.txt'
            [ -f '$FEATS_TMP' ]         && cp '$FEATS_TMP'         '$DIR/${TAG}_feats.txt'
            [ -f '$EST_TMP' ]           && cp '$EST_TMP'           '$DIR/${TAG}_est.txt'
            true
          " || echo "  -> WARNING: subscribe rep returned nonzero (container/launch error?)" >&2
          # In native mode, also reap host-side strays. In --docker mode the
          # container is --rm so it cleans itself up; the user-scoped pkill
          # is a no-op there but harmless.
          kill_stale_subscribe_nodes
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

# Frame-level stats from the trailing "total" column of an OpenVINS timing
# CSV: prints "mean std p99" (ms, space-separated). Empty on missing/empty
# file. Reads the CSV directly — avoids `ros2 run ov_eval timing_flamegraph`,
# which is a Qt/matplotlib GUI tool that aborts on headless hosts. Requires
# GNU awk (asort).
frame_stats() {
  local f="$1"
  [ -f "$f" ] || return 0
  awk -F, '
    !/^#/ && NF >= 2 { v[++n] = $NF }
    END {
      if (n == 0) exit
      s = 0; for (i=1;i<=n;i++) s += v[i]
      m = s/n
      ss = 0; for (i=1;i<=n;i++) ss += (v[i]-m)*(v[i]-m)
      sd = (n>1) ? sqrt(ss/(n-1)) : 0
      asort(v)
      idx = int(0.99 * n + 0.5); if (idx < 1) idx = 1; if (idx > n) idx = n
      printf "%.1f %.1f %.1f", m*1000, sd*1000, v[idx]*1000
    }' "$f"
}

# Format a frame_stats triple ("mean std p99") as "mean±std (p99: X)".
# Empty input yields a literal em dash for the cell.
fmt_frame_cell() {
  if [ -z "$1" ]; then echo "—"; return; fi
  read -r m s p <<<"$1"
  printf '%s±%s (p99: %s)' "$m" "$s" "$p"
}

# Mean ± std of a whitespace-separated series. Single value → "mean±0.0";
# empty → "--".
mean_std() {
  awk '{
    n = NF
    if (n == 0) { printf "--"; exit }
    s = 0; for (i=1;i<=n;i++) s += $i
    m = s/n
    ss = 0; for (i=1;i<=n;i++) ss += ($i-m)*($i-m)
    sd = (n>1) ? sqrt(ss/(n-1)) : 0
    printf "%.1f±%.1f", m, sd
  }' <<<"$*"
}

# Find the first rep index that has a populated wall.txt for a subscribe cell.
# Echoes the index (1..reps) or empty if none.
first_complete_rep() {
  local dir="$1" prefix="$2"
  local i
  for i in $(seq 1 "$SUBSCRIBE_REPS"); do
    if [ -f "$dir/${prefix}_run${i}_wall.txt" ]; then
      echo "$i"; return
    fi
  done
}

# Print where output files for this run landed. Indented for nesting in summary.
print_save_locations() {
  if [[ "$MODE" == "serial" || "$MODE" == "both" ]]; then
    echo "  Serial:    $RESULTS_BASE/serial/$BENCH_TAG/"
  fi
  if [[ ( "$MODE" == "subscribe" || "$MODE" == "both" ) && "$CAMERAS" != "mono" ]]; then
    echo "  Subscribe: $RESULTS_BASE/subscribe/$BENCH_TAG/"
  fi
}

echo "================================================================"
echo "  RESULTS SUMMARY ($BENCH_TAG)"
echo "================================================================"
echo ""
echo "Saved results:"
print_save_locations
echo ""

# ── Serial timing table (3-clock totals) ──
if [[ "$MODE" == "serial" || "$MODE" == "both" ]]; then
  echo "--- Serial timing — frame-level total (ms) ---"
  printf "  %-14s %-3s %-6s %22s %22s %22s\n" "sequence" "thr" "cam" \
    "wall (mean±std, p99)" "cpu (mean±std, p99)" "thread (mean±std, p99)"
  for seq in "${SEQUENCES[@]}"; do
    for thr in "${THREADS[@]}"; do
      for cam_spec in "${CAM_CONFIGS[@]}"; do
        IFS=':' read -r CAM_NAME _ _ <<< "$cam_spec"
        TAG="$(tag_for "$seq" "$thr" "$CAM_NAME")"
        DIR="$RESULTS_BASE/serial/$BENCH_TAG"
        FS_WALL=$(frame_stats "$DIR/${TAG}_wall.txt")
        [ -z "$FS_WALL" ] && continue
        FS_CPU=$(frame_stats  "$DIR/${TAG}_cpu.txt")
        FS_THR=$(frame_stats  "$DIR/${TAG}_thread.txt")
        printf "  %-14s %-3s %-6s %22s %22s %22s\n" \
          "$seq" "$thr" "$CAM_NAME" \
          "$(fmt_frame_cell "$FS_WALL")" \
          "$(fmt_frame_cell "$FS_CPU")" \
          "$(fmt_frame_cell "$FS_THR")"
      done
    done
  done
  echo ""
fi

# ── Subscribe timing tables: frame-level (one rep) + across-reps aggregate ──
if [[ ( "$MODE" == "subscribe" || "$MODE" == "both" ) && "$CAMERAS" != "mono" ]]; then
  # Table 1: frame-level (rep N picked per cell — first complete one).
  echo "--- Subscribe timing — frame-level total (ms; rep marked per row) ---"
  printf "  %-14s %-3s %-5s %5s %22s %22s %22s\n" "sequence" "thr" "rate" "rep" \
    "wall (mean±std, p99)" "cpu (mean±std, p99)" "thread (mean±std, p99)"
  DIR="$RESULTS_BASE/subscribe/$BENCH_TAG"
  for seq in "${SEQUENCES[@]}"; do
    for thr in "${THREADS[@]}"; do
      for rate in "${RATES[@]}"; do
        if [ "$rate" = "1.0" ]; then rate_suffix=""; else rate_suffix="_rate${rate}"; fi
        prefix="${seq}_${thr}thr${rate_suffix}"
        REP=$(first_complete_rep "$DIR" "$prefix")
        [ -z "$REP" ] && continue
        BASE="$DIR/${prefix}_run${REP}"
        FS_WALL=$(frame_stats "${BASE}_wall.txt")
        [ -z "$FS_WALL" ] && continue
        FS_CPU=$(frame_stats  "${BASE}_cpu.txt")
        FS_THR=$(frame_stats  "${BASE}_thread.txt")
        printf "  %-14s %-3s %-5s %5s %22s %22s %22s\n" \
          "$seq" "$thr" "$rate" "$REP" \
          "$(fmt_frame_cell "$FS_WALL")" \
          "$(fmt_frame_cell "$FS_CPU")" \
          "$(fmt_frame_cell "$FS_THR")"
      done
    done
  done
  echo ""

  # Table 2: across-reps aggregate. Each cell shows
  #   mean±std (of per-rep frame-mean) / mean±std (of per-rep frame-p99)
  echo "--- Subscribe timing — across $SUBSCRIBE_REPS reps (mean±std of per-rep frame-mean / frame-p99, ms) ---"
  printf "  %-14s %-3s %-5s %22s %22s %22s\n" "sequence" "thr" "rate" "wall" "cpu" "thread"
  for seq in "${SEQUENCES[@]}"; do
    for thr in "${THREADS[@]}"; do
      for rate in "${RATES[@]}"; do
        if [ "$rate" = "1.0" ]; then rate_suffix=""; else rate_suffix="_rate${rate}"; fi
        WM="" WP="" CM="" CP="" TM="" TP=""
        for i in $(seq 1 "$SUBSCRIBE_REPS"); do
          BASE="$DIR/${seq}_${thr}thr${rate_suffix}_run${i}"
          [ -f "${BASE}_wall.txt" ] || continue
          read -r m _ p <<< "$(frame_stats "${BASE}_wall.txt")"
          [ -n "$m" ] && WM+=" $m"; [ -n "$p" ] && WP+=" $p"
          read -r m _ p <<< "$(frame_stats "${BASE}_cpu.txt")"
          [ -n "$m" ] && CM+=" $m"; [ -n "$p" ] && CP+=" $p"
          read -r m _ p <<< "$(frame_stats "${BASE}_thread.txt")"
          [ -n "$m" ] && TM+=" $m"; [ -n "$p" ] && TP+=" $p"
        done
        [ -z "${WM// }" ] && continue
        printf "  %-14s %-3s %-5s %22s %22s %22s\n" \
          "$seq" "$thr" "$rate" \
          "$(mean_std "$WM") / $(mean_std "$WP")" \
          "$(mean_std "$CM") / $(mean_std "$CP")" \
          "$(mean_std "$TM") / $(mean_std "$TP")"
      done
    done
  done
  echo ""
fi

# ── SLAM feature health table ──
echo "--- SLAM feature health (avg features in state) ---"
printf "  %-14s %-12s %-3s %-5s %14s\n" "sequence" "mode" "thr" "rate" "avg_features"
for seq in "${SEQUENCES[@]}"; do
  if [[ "$MODE" == "serial" || "$MODE" == "both" ]]; then
    for thr in "${THREADS[@]}"; do
      for cam_spec in "${CAM_CONFIGS[@]}"; do
        IFS=':' read -r CAM_NAME _ _ <<< "$cam_spec"
        TAG="$(tag_for "$seq" "$thr" "$CAM_NAME")"
        SLAM=$(get_slam_avg "$RESULTS_BASE/serial/$BENCH_TAG/${TAG}_feats.txt")
        if [ "$SLAM" = "?" ]; then continue; fi
        printf "  %-14s %-14s %-3s %-5s %14s\n" "$seq" "serial/$CAM_NAME" "$thr" "—" "$SLAM"
      done
    done
  fi
  if [[ ( "$MODE" == "subscribe" || "$MODE" == "both" ) && "$CAMERAS" != "mono" ]]; then
    for thr in "${THREADS[@]}"; do
      for rate in "${RATES[@]}"; do
        if [ "$rate" = "1.0" ]; then rate_suffix=""; else rate_suffix="_rate${rate}"; fi
        VALS=""
        for i in $(seq 1 "$SUBSCRIBE_REPS"); do
          S=$(get_slam_avg "$RESULTS_BASE/subscribe/$BENCH_TAG/${seq}_${thr}thr${rate_suffix}_run${i}_feats.txt")
          [ "$S" = "?" ] && continue
          VALS+=" $S"
        done
        [ -z "${VALS// }" ] && continue
        printf "  %-14s %-14s %-3s %-5s %14s\n" "$seq" "subscribe" "$thr" "$rate" "$(mean_std "$VALS")"
      done
    done
  fi
done
echo ""

# ── ATE table ──
GT_DIR="$WS_DIR/src/open_vins/ov_data/euroc_mav"
echo "--- ATE position RMSE (m, posyaw alignment) ---"
printf "  %-14s %-12s %-3s %-5s %14s\n" "sequence" "mode" "thr" "rate" "rmse_pos"
for seq in "${SEQUENCES[@]}"; do
  GT="$GT_DIR/${seq}.txt"
  [ -f "$GT" ] || continue
  if [[ "$MODE" == "serial" || "$MODE" == "both" ]]; then
    for thr in "${THREADS[@]}"; do
      for cam_spec in "${CAM_CONFIGS[@]}"; do
        IFS=':' read -r CAM_NAME _ _ <<< "$cam_spec"
        TAG="$(tag_for "$seq" "$thr" "$CAM_NAME")"
        F="$RESULTS_BASE/serial/$BENCH_TAG/${TAG}_est.txt"
        [ -f "$F" ] || continue
        ATE=$(QT_QPA_PLATFORM=offscreen timeout 30 ros2 run ov_eval error_singlerun posyaw "$GT" "$F" 2>/dev/null | grep "rmse_.*pos" | head -1 | grep -oP 'rmse_pos = \K[0-9.]+' || true)
        printf "  %-14s %-14s %-3s %-5s %14s\n" "$seq" "serial/$CAM_NAME" "$thr" "—" "${ATE:-FAIL}"
      done
    done
  fi
  if [[ ( "$MODE" == "subscribe" || "$MODE" == "both" ) && "$CAMERAS" != "mono" ]]; then
    for thr in "${THREADS[@]}"; do
      for rate in "${RATES[@]}"; do
        if [ "$rate" = "1.0" ]; then rate_suffix=""; else rate_suffix="_rate${rate}"; fi
        VALS=""
        for i in $(seq 1 "$SUBSCRIBE_REPS"); do
          F="$RESULTS_BASE/subscribe/$BENCH_TAG/${seq}_${thr}thr${rate_suffix}_run${i}_est.txt"
          [ -f "$F" ] || continue
          ATE=$(QT_QPA_PLATFORM=offscreen timeout 30 ros2 run ov_eval error_singlerun posyaw "$GT" "$F" 2>/dev/null | grep "rmse_.*pos" | head -1 | grep -oP 'rmse_pos = \K[0-9.]+' || true)
          [ -n "$ATE" ] && VALS+=" $ATE"
        done
        [ -z "${VALS// }" ] && continue
        printf "  %-14s %-14s %-3s %-5s %14s\n" "$seq" "subscribe" "$thr" "$rate" "$(mean_std "$VALS")"
      done
    done
  fi
done
echo ""

# ── Zombie check ──
ZOMBIES=$( { pgrep -u "$USER" -f "run_subscribe_msckf" 2>/dev/null || :; } | wc -l)
echo "--- Zombie check (this user only): ${ZOMBIES} stale processes ---"
echo ""
echo "Saved results:"
print_save_locations
echo ""
echo "Per-component breakdown (paper-style, mean±std and p99 per component):"
if [[ "$MODE" == "serial" || "$MODE" == "both" ]]; then
  echo "  python3 $WS_DIR/scripts/parse_results.py $RESULTS_BASE/serial/$BENCH_TAG --per-component"
fi
if [[ ( "$MODE" == "subscribe" || "$MODE" == "both" ) && "$CAMERAS" != "mono" ]]; then
  echo "  python3 $WS_DIR/scripts/parse_results.py $RESULTS_BASE/subscribe/$BENCH_TAG --per-component"
fi
echo ""
echo "================================================================"
echo "  Done — $(date)"
echo "================================================================"
