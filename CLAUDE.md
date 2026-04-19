# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build

```bash
source /opt/ros/humble/setup.bash
cd ~/workspace/catkin_ws_ov
colcon build --symlink-install
source install/setup.bash
```

Rebuild a single package:
```bash
colcon build --symlink-install --packages-select ov_msckf
```

Dependencies: Eigen3, OpenCV 3/4, Boost, Ceres, libglog, libgflags, libatlas, libsuitesparse, ROS 2 Humble.

One-shot setup from scratch: `./install.sh`

## Running VIO

**Serial mode** (deterministic, for benchmarking — reads bag directly, no `ros2 bag play`):
```bash
ros2 launch ov_msckf serial.launch.py \
    config_path:=/path/to/estimator_config.yaml \
    path_bag:=/path/to/euroc_bag \
    max_cameras:=2 use_stereo:=true
```

**Subscribe mode** (real-time, message-driven):
```bash
ros2 launch ov_msckf subscribe.launch.py config_path:=/path/to/estimator_config.yaml &
ros2 bag play /path/to/bag --rate 1.0
```

EuRoC bags live in `~/datasets/euroc/` (V1_01_easy, MH_03_medium, V2_02_medium are the benchmark sequences).

## Evaluation Tools

```bash
# Per-component timing statistics (mean, std, p99, max)
ros2 run ov_eval timing_flamegraph /tmp/traj_timing.txt

# Compare timing across runs
ros2 run ov_eval timing_comparison file1.txt file2.txt

# ATE/RPE error (posyaw alignment)
ros2 run ov_eval error_singlerun posyaw groundtruth.txt estimate.txt [8 16 24 32 40]
```

Timing files are CSV: `timestamp,tracking,propagation,msckf_update,slam_update,slam_delayed,retri_marg,total`

## Benchmarking Scripts

| Script | Purpose |
|--------|---------|
| `run_full_benchmark.sh` | Complete suite: serial + subscribe, 3 sequences, 5 reps (~2.5 hrs) |
| `run_timing_benchmark.sh` | Serial-only baseline: 3 sequences x stereo/mono |
| `run_timing_combined.sh` | Subscribe timing + accuracy + feature counts with multi-rep |
| `run_timing_subscribe.sh` | Subscribe-mode timing with configurable threads/SLAM delay |
| `run_timing_sweep.sh` | Config sensitivity: thread count, feature density, SLAM delay |

Results go to `~/results/timing/x86/{serial,subscribe}/`. File naming: `{SEQUENCE}_{THREADS}thr_run{N}_{wall|cpu|thread|feats|est}.txt`

## Formatting

Uses `.clang-format` (LLVM-based, 140 column limit, 2-space indent).

## Architecture

### Package Dependency Graph

```
ov_core          (CV primitives, type system, feature tracking, camera models)
  ├── ov_init    (static/dynamic VIO initialization, Ceres optimization)
  ├── ov_eval    (ATE/RPE metrics, timing analysis tools)
  └── ov_msckf   (MSCKF filter, ROS 2 nodes, launch files, configs)
        uses ov_core + ov_init
```

`ov_data` holds ground-truth trajectories (no code).

### VIO Pipeline (in VioManager)

IMU + stereo images flow through 6 timed stages:
1. **Tracking** — KLT optical flow + stereo matching (`ov_core/src/track/`)
2. **Propagation** — RK4 IMU integration (`ov_msckf/src/state/Propagator.h`)
3. **MSCKF Update** — lost-feature triangulation + nullspace EKF update (`ov_msckf/src/update/UpdaterMSCKF`)
4. **SLAM Update** — persistent landmark EKF update (`ov_msckf/src/update/UpdaterSLAM`)
5. **SLAM Delayed Init** — new landmark initialization into state
6. **Re-triangulation & Marginalization** — sliding window maintenance

Central classes: `VioManager` (orchestrator), `State` (filter state + covariance), `Propagator`, `UpdaterMSCKF`, `UpdaterSLAM`.

### ROS 2 Nodes

- `run_subscribe_msckf` — standard real-time node with MultiThreadedExecutor, subscribes to `/imu0`, `/cam0/image_raw`, `/cam1/image_raw`
- `ros2_serial_msckf` — **fork addition**: reads bags sequentially with blocking VIO updates, deterministic (bit-identical across runs)

Both use `ROS2Visualizer` to publish odometry, point clouds, and TF.

### Fork-Specific Additions

- **Serial VIO node** (`ov_msckf/src/ros2_serial_msckf.cpp`) — deterministic offline bag processing for reproducible benchmarking
- **SLAM recovery** — when SLAM features drop below 25% of `max_slam`, chi-squared gate relaxes 3x to prevent irrecoverable feature collapse in subscribe mode
- **3-clock timing** — wall clock, process CPU, and thread CPU instrumentation in `VioManager::track_image_and_update()`
- **Feature count recording** — per-frame MSCKF/SLAM/tracking feature counts

## Configuration

Dataset configs live in `src/open_vins/config/{dataset}/` with 3 files each:
- `estimator_config.yaml` — algorithm parameters (features, SLAM, threads, timing toggles)
- `kalibr_imucam_chain.yaml` — camera-IMU extrinsics
- `kalibr_imu_chain.yaml` — IMU noise parameters

Key tuning knobs in `estimator_config.yaml`:
- `num_pts` (feature density, default 200), `max_slam` (SLAM features, default 50)
- `max_clones` (sliding window size, default 11), `max_cameras` (1=mono, 2=stereo)
- `record_timing_information`, `record_timing_cpu_time`, `record_timing_thread_time`, `record_feature_counts`
- `multi_threading_subs` (OpenCV parallel threads, default 4)

Primary benchmark config: `euroc_mav`. Custom config: `barak_mav` (for live RPi5 camera).
