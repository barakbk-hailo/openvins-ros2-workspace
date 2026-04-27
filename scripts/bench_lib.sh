# Shared helpers for benchmark orchestrator scripts.
# Source this from a sibling script with `. "$(dirname "$0")/bench_lib.sh"`.
# Sourced (not executed); do not run directly.

# ── Path constants (overridable via env) ──
WS_DIR="${WS_DIR:-$HOME/workspace/catkin_ws_ov}"
DATASETS_DIR="${DATASETS_DIR:-$HOME/datasets/euroc}"
CONFIG_DIR="${CONFIG_DIR:-$WS_DIR/src/open_vins/config/euroc_mav}"
BASE_CONFIG="${BASE_CONFIG:-$CONFIG_DIR/estimator_config.yaml}"

# Default results base by host arch. Map x86_64 → x86 and aarch64 → rpi5 so the
# tag layout in data-provenance.md (results/timing/{x86,rpi5}/...) is preserved.
# Other archs land under `results/timing/$(uname -m)/`. Callers can override via
# the RESULTS_BASE env var or a script-level --results-base flag.
arch_results_base() {
  case "$(uname -m)" in
    x86_64)  echo "$HOME/results/timing/x86" ;;
    aarch64) echo "$HOME/results/timing/rpi5" ;;
    *)       echo "$HOME/results/timing/$(uname -m)" ;;
  esac
}

# Temp file paths the OpenVINS nodes write to (matches launch defaults).
TIMING_WALL_TMP="/tmp/traj_timing.txt"
TIMING_CPU_TMP="/tmp/traj_timing_cpu.txt"
TIMING_THREAD_TMP="/tmp/traj_timing_thread.txt"
FEATS_TMP="/tmp/traj_features.txt"
EST_TMP="/tmp/ov_estimate.txt"
STD_TMP="/tmp/ov_estimate_std.txt"

# ── ROS distro detection ──
# Match install.sh's UBUNTU_CODENAME-driven choice (noble→jazzy, jammy→humble).
# Sources both ROS and the workspace overlay. Caller must invoke before any `ros2 ...`.
source_ros() {
  local ros_distro
  case "$(. /etc/os-release && echo "${UBUNTU_CODENAME:-}")" in
    noble) ros_distro=jazzy ;;
    *)     ros_distro=humble ;;
  esac
  [ -f "/opt/ros/${ros_distro}/setup.bash" ] || {
    echo "ERROR: /opt/ros/${ros_distro}/setup.bash not found — is ROS 2 installed?" >&2
    return 1
  }
  # ROS setup scripts touch unset vars on first source; relax `set -u` around them.
  set +u
  # shellcheck disable=SC1090
  source "/opt/ros/${ros_distro}/setup.bash"
  if [ -f "$WS_DIR/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$WS_DIR/install/setup.bash"
  else
    echo "WARNING: $WS_DIR/install/setup.bash not found — did you `colcon build`?" >&2
  fi
  set -u
}

# ── Config templating ──
# make_bench_config <output_path> [chi2_recovery] [num_threads]
# Copies BASE_CONFIG, enables all timing/feature CSV recording, optionally
# overrides slam_chi2_recovery and num_opencv_threads. Verifies each sed
# substitution actually fired so silent no-ops on YAML drift become errors.
make_bench_config() {
  local out="$1" chi2="${2:-}" threads="${3:-}"
  cp "$BASE_CONFIG" "$out"

  local k
  for k in record_timing_information record_timing_cpu_time record_timing_thread_time record_feature_counts; do
    sed -i "s/^${k}: false/${k}: true/" "$out"
    grep -q "^${k}: true" "$out" || {
      echo "ERROR: failed to enable ${k} in $out — has the upstream YAML changed?" >&2
      return 1
    }
  done

  if [ -n "$threads" ]; then
    sed -i "s/^num_opencv_threads: [0-9]*/num_opencv_threads: ${threads}/" "$out"
    grep -q "^num_opencv_threads: ${threads}\b" "$out" || {
      echo "ERROR: failed to set num_opencv_threads=${threads} in $out" >&2
      return 1
    }
  fi

  if [ -n "$chi2" ]; then
    sed -i "s/^slam_chi2_recovery: .*/slam_chi2_recovery: ${chi2} # overridden via --slam-chi2-recovery/" "$out"
    grep -q "^slam_chi2_recovery: ${chi2}\b" "$out" || {
      echo "ERROR: failed to override slam_chi2_recovery=${chi2} in $out" >&2
      return 1
    }
  fi
}

# ── Validators ──
validate_chi2_recovery() {
  case "${1:-}" in
    ""|true|false) return 0 ;;
    *) echo "ERROR: --slam-chi2-recovery must be true or false (got: $1)" >&2; return 2 ;;
  esac
}

validate_positive_int() { # <name> <value>
  [[ "${2:-}" =~ ^[1-9][0-9]*$ ]] || {
    echo "ERROR: $1 must be a positive integer (got: ${2:-<empty>})" >&2
    return 2
  }
}

validate_nonempty() { # <name> <value>
  [ -n "${2:-}" ] || {
    echo "ERROR: $1 must not be empty" >&2
    return 2
  }
}

require_dir() { # <label> <path>
  [ -d "$2" ] || {
    echo "ERROR: $1 directory not found: $2" >&2
    return 1
  }
}

require_bag() { # <seq>
  local seq="$1"
  [ -d "$DATASETS_DIR/$seq" ] || {
    echo "ERROR: bag not found: $DATASETS_DIR/$seq" >&2
    return 1
  }
}

# ── Process management ──
# User-scoped TERM-then-KILL of stray run_subscribe_msckf nodes from prior runs.
# Avoids -9-everywhere harshness and won't touch other users' processes.
kill_stale_subscribe_nodes() {
  pkill -u "$USER" -TERM -f 'run_subscribe_msckf' 2>/dev/null || true
  sleep 0.5
  pkill -u "$USER" -KILL -f 'run_subscribe_msckf' 2>/dev/null || true
}

# Stable label used to tag containers spawned by this orchestrator process so
# the EXIT/INT/TERM trap can kill only its own children (vs every container
# from $DOCKER_IMAGE on the host, which would harm a coworker running the
# same image in another session).
ov_bench_label() { echo "ov_bench_pid=$$"; }

# Kill containers tagged by this orchestrator process. Used by the
# EXIT/INT/TERM trap so a Ctrl-C doesn't leak a running container.
kill_stale_docker_containers() {
  [ -n "${DOCKER_IMAGE:-}" ] || return 0
  command -v docker >/dev/null 2>&1 || return 0
  local stale
  stale=$(docker ps -q --filter "label=$(ov_bench_label)" 2>/dev/null || true)
  [ -n "$stale" ] && docker kill $stale >/dev/null 2>&1 || true
}

# ── Native / Docker dispatcher ──
# Run a block of shell code either natively (eval) or inside a docker container.
# Set DOCKER_IMAGE (typically via the orchestrator's --docker flag) to switch
# modes. DOCKER_FLAGS (a single space-separated string) is word-split and
# passed to `docker run` so callers can layer on `--cap-add`, `--ulimit`, etc.
#
# Mounts: $HOME/workspace, $HOME/datasets, and $HOME/results are all bind-
# mounted at their host paths so any path the orchestrator passes (CFG, bag
# dir, results dir) resolves to the same filesystem location in both modes.
# HOME=/tmp is exported in the container so `~/.ros/log` writes don't fail
# when the container user has no writable home.
#
# In docker mode the inner code sources `/opt/ros_ws/install/setup.bash` (the
# image's prebuilt OpenVINS), NOT the host workspace overlay — the image is
# the source of truth for the binary being measured.
docker_wrap() { # <bash_code>
  if [ -z "${DOCKER_IMAGE:-}" ]; then
    eval "$1"
    return $?
  fi
  command -v docker >/dev/null 2>&1 || {
    echo "ERROR: --docker $DOCKER_IMAGE requested but docker is not in PATH" >&2
    return 1
  }
  local -a docker_flags_array=()
  if [ -n "${DOCKER_FLAGS:-}" ]; then
    # Intentional word-splitting on user-supplied flags.
    # shellcheck disable=SC2206
    docker_flags_array=( $DOCKER_FLAGS )
  fi
  docker run --rm --network host \
    --user "$(id -u):$(id -g)" \
    --label "$(ov_bench_label)" \
    -v "$HOME/workspace:$HOME/workspace" \
    -v "$HOME/datasets:$HOME/datasets" \
    -v "$HOME/results:$HOME/results" \
    -e HOME=/tmp \
    "${docker_flags_array[@]}" \
    "$DOCKER_IMAGE" bash -c "set -eo pipefail; source /opt/ros_ws/install/setup.bash; $1"
}

# ── Output validation ──
# A run "completed" iff its wall-clock CSV has more than this many data rows.
# 100 catches "init only" partial runs; every EuRoC bag produces >2000 frames.
MIN_ROWS_FOR_COMPLETE_RUN="${MIN_ROWS_FOR_COMPLETE_RUN:-100}"

run_complete() { # <results_dir> <tag>
  local f="$1/${2}_wall.txt"
  [ -f "$f" ] || return 1
  [ -f "$1/${2}_feats.txt" ] || return 1
  local rows
  rows=$(grep -cv '^#' "$f" 2>/dev/null || echo 0)
  [ "$rows" -gt "$MIN_ROWS_FOR_COMPLETE_RUN" ]
}
