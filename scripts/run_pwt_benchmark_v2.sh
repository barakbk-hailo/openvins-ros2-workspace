#!/usr/bin/env bash
# PWT benchmark v2: 3-clock timing + feature counts + accuracy
#
# RPi5 / Docker only — runs OpenVINS inside a prebuilt container; not portable
# to x86 or non-Docker hosts. For x86 use run_full_benchmark.sh / run_timing_*.sh.
#
# Usage:
#   bash run_pwt_benchmark_v2.sh <results_subdir> <image_tag> [num_subscribe_reps] [extra_docker_flags...]
#
# Example (baseline):
#   bash run_pwt_benchmark_v2.sh pwt_baseline openvins-humble-pwt 10
#
# Example (Docker RT flags):
#   bash run_pwt_benchmark_v2.sh pwt_rtflags openvins-humble-pwt 10 \
#       --cap-add=SYS_NICE --ulimit rtprio=99 --ulimit memlock=-1 --cpuset-cpus=0-3
#
# Example (max-interval sync):
#   bash run_pwt_benchmark_v2.sh pwt_maxinterval openvins-humble-maxinterval 10

set -euo pipefail

SUBDIR="${1:?Usage: $0 <results_subdir> <image_tag> [reps] [extra_docker_flags...]}"
IMAGE="${2:?Missing image tag}"
REPS="${3:-10}"
shift 3 || true
EXTRA_DOCKER=("$@")

RESULTS="$HOME/workspace/catkin_ws_ov/results/rpi5/$SUBDIR"
SEQ=V1_01_easy

mkdir -p "$RESULTS"

# Kill any stray containers from this image on interrupt/exit so the orchestrator
# doesn't leak a running container if aborted mid-rep.
cleanup() {
  local stale
  stale=$(docker ps -q --filter "ancestor=$IMAGE" 2>/dev/null || true)
  [ -n "$stale" ] && docker kill $stale >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

echo "==========================================================="
echo "  PWT benchmark v2"
echo "  Subdir: $SUBDIR"
echo "  Image:  $IMAGE"
echo "  Reps:   $REPS subscribe (2 serial)"
echo "  Extra:  ${EXTRA_DOCKER[*]:-<none>}"
echo "  Out:    $RESULTS"
echo "==========================================================="

run_in_container() {
  local MODE=$1      # serial | subscribe
  local TAG=$2       # e.g. serial_run1, sub_run3

  docker run --rm --network host --privileged \
    --user "$(id -u):$(id -g)" \
    -v /dev:/dev -v ~/workspace:/workspace -v ~/datasets:/datasets \
    "${EXTRA_DOCKER[@]}" \
    "$IMAGE" bash -c '
    set -e
    source /opt/ros_ws/install/setup.bash
    CONFIG_DIR=/workspace/catkin_ws_ov/src/open_vins/config/euroc_mav
    TMP_CONFIG="$CONFIG_DIR/estimator_config_pwt_v2.yaml"
    GT=/opt/ros_ws/src/open_vins/ov_data/euroc_mav/'$SEQ'.txt

    # Enable all timing/feature instrumentation
    cp "$CONFIG_DIR/estimator_config.yaml" "$TMP_CONFIG"
    sed -i "s/record_timing_information: false/record_timing_information: true/" "$TMP_CONFIG"
    sed -i "s/record_timing_cpu_time: false/record_timing_cpu_time: true/" "$TMP_CONFIG"
    sed -i "s/record_timing_thread_time: false/record_timing_thread_time: true/" "$TMP_CONFIG"
    sed -i "s/record_feature_counts: false/record_feature_counts: true/" "$TMP_CONFIG"

    rm -f /tmp/traj_timing.txt /tmp/traj_timing_cpu.txt /tmp/traj_timing_thread.txt \
          /tmp/traj_features.txt /tmp/est.txt /tmp/std.txt

    if [ "'$MODE'" = "serial" ]; then
      ros2 run ov_msckf ros2_serial_msckf --ros-args \
        -p config_path:=$TMP_CONFIG \
        -p path_bag:=/datasets/euroc/'$SEQ' \
        -p path_gt:=$GT \
        -p max_cameras:=2 -p use_stereo:=true \
        -p save_total_state:=true \
        -p filepath_est:=/tmp/est.txt -p filepath_std:=/tmp/std.txt > /dev/null 2>&1
    else
      ros2 run ov_msckf run_subscribe_msckf --ros-args \
        -p config_path:=$TMP_CONFIG \
        -p path_gt:=$GT \
        -p max_cameras:=2 -p use_stereo:=true \
        -p save_total_state:=true \
        -p filepath_est:=/tmp/est.txt -p filepath_std:=/tmp/std.txt > /dev/null 2>&1 &
      sleep 3
      ros2 bag play /datasets/euroc/'$SEQ' --rate 1.0 > /dev/null 2>&1
      sleep 5
      kill %1 2>/dev/null; wait 2>/dev/null
    fi

    OUT=/workspace/catkin_ws_ov/results/rpi5/'$SUBDIR'
    [ -f /tmp/traj_timing.txt ]         && cp /tmp/traj_timing.txt         $OUT/'$TAG'_wall.txt
    [ -f /tmp/traj_timing_cpu.txt ]     && cp /tmp/traj_timing_cpu.txt     $OUT/'$TAG'_cpu.txt
    [ -f /tmp/traj_timing_thread.txt ]  && cp /tmp/traj_timing_thread.txt  $OUT/'$TAG'_thread.txt
    [ -f /tmp/traj_features.txt ]       && cp /tmp/traj_features.txt       $OUT/'$TAG'_feats.txt

    if [ -f /tmp/est.txt ]; then
      echo "# timestamp(s) tx ty tz qx qy qz qw" > $OUT/'$TAG'_pose.txt
      awk "!/^#/ {printf \"%.5f %.9f %.9f %.9f %.9f %.9f %.9f %.9f\n\", \$1, \$6, \$7, \$8, \$2, \$3, \$4, \$5}" /tmp/est.txt >> $OUT/'$TAG'_pose.txt
    fi

    # Quick summary for this run
    ROWS=$(grep -cv "^#" $OUT/'$TAG'_wall.txt 2>/dev/null || echo 0)
    RMSE=$(ros2 run ov_eval error_singlerun posyaw $GT $OUT/'$TAG'_pose.txt 2>&1 | grep "^rmse_ori" | head -1 | tr -d "\x1b[0m" | sed "s/\[0m//")
    echo "SUMMARY frames=$ROWS $RMSE"

    rm -f "$TMP_CONFIG"
  ' 2>&1 | grep -E "SUMMARY"
}

echo ""
echo "======== SERIAL RUNS (determinism check) ========"
for i in 1 2; do
  echo "--- serial run $i ---"
  run_in_container serial "serial_run$i"
done

echo ""
echo "======== SUBSCRIBE RUNS (variance check) ========"
for i in $(seq 1 $REPS); do
  echo "--- subscribe run $i/$REPS ---"
  run_in_container subscribe "sub_run$i"
done

echo ""
echo "======== DONE ========"
echo "Results in: $RESULTS"
ls -la "$RESULTS"
