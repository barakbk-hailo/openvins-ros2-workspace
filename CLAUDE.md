# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build

```bash
# Auto-detect ROS distro: humble on Ubuntu 22.04 (Jammy), jazzy on 24.04 (Noble)
. /etc/os-release && source /opt/ros/$([ "$UBUNTU_CODENAME" = "noble" ] && echo jazzy || echo humble)/setup.bash
cd ~/workspace/catkin_ws_ov
colcon build --symlink-install
source install/setup.bash
```

Rebuild a single package:
```bash
colcon build --symlink-install --packages-select ov_msckf
```

Dependencies: Eigen3, OpenCV 3/4, Boost, Ceres, libglog, libgflags, libatlas, libsuitesparse, ROS 2 Humble (Ubuntu 22.04) or Jazzy (Ubuntu 24.04).

One-shot setup from scratch: `./install.sh`

## Running VIO

Full walk-through in [docs/running.md](docs/running.md). One-liners:

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
| `run_full_benchmark.sh` | Flexible orchestrator: serial/subscribe × sequences × threads × cameras × reps. `--quick` for 1-rep smoke, `--help` for options. |
| `run_timing_subscribe.sh` | Subscribe mode at configurable bag-playback rates (1×/2×/5× realtime feasibility — Phase 3) |
| `run_timing_sweep.sh` | Config sensitivity sweep: thread count, feature density, SLAM delay (Phase 2) |

Example: targeted subscribe benchmark on a single sequence
```bash
bash run_full_benchmark.sh -m subscribe -s V2_02_medium -r 5 --tag bench_v2
```

Results go to `~/results/timing/x86/{serial,subscribe}/<tag>/`. File naming:
`{SEQUENCE}_{THREADS}thr_run{N}_{wall|cpu|thread|feats|est}.txt` (subscribe),
`{SEQUENCE}_{THREADS}thr_{wall|cpu|thread|feats|est}.txt` (serial). Mono runs
append `_mono` to the tag.

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
- **Persistent worker thread** (`ov_msckf/src/ros/ROS2Visualizer.{h,cpp}`) — replaces the upstream per-frame `detach()` dispatch with a single long-lived worker. Eliminates TOCTOU race, dangling-reference UB, and non-deterministic IMU triggering. Reduces subscribe overhead from 2× serial to 1× serial. Details in [docs/determinism.md](docs/determinism.md).
- **SLAM recovery** — when SLAM features drop below 25% of `max_slam`, chi-squared gate relaxes 3× to admit features in spite of elevated residuals. Off by default (`slam_chi2_recovery: false`) since 2026-04-26 because the always-on behavior interfered with stereo init on dark sequences (MH_05_difficult); opt-in via the YAML knob or the `--slam-chi2-recovery <true|false>` flag on the orchestrator scripts. See [docs/determinism.md §4](docs/determinism.md#4-optional-safety-net-slam-recovery-mechanism).
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
- `num_opencv_threads` (OpenCV parallel threads, default 4) — controls KLT parallelism
- `multi_threading_subs` (default `true`) — **fork addition**: async vs inline VIO dispatch in subscribe mode; previously hardcoded, now settable from YAML
- `record_timing_information` (wall-clock CSV), `record_timing_cpu_time`, `record_timing_thread_time` (process-CPU and thread-CPU CSVs — fork additions), `record_feature_counts` (per-frame feature counts — fork addition)

Primary benchmark config: `euroc_mav`. Custom config: `barak_mav` (for live RPi5 camera).

## Docs guidelines

- Docs in `docs/` follow a linear flow: install → run → evaluate →
  determinism → timing → benchmark-analysis → rpi5-setup → rpi5-benchmarking.
  See [README.md](README.md) for the full index and [docs/running.md](docs/running.md) as the entry point.
- Every results/data table carries a `*Source: ...*` citation line pointing
  at the CSV(s) under `results/` that produced it.
  Example: `*Source: results/timing/x86/serial/stereo/V1_01_easy.txt*`
- RPi5-specific content lives in `docs/rpi5-*.md` only. Don't add RPi5
  sections to x86-scoped docs (`timing.md`, `benchmark-analysis.md`,
  `evaluation.md`).
- When adding a new benchmark table, include the `run_full_benchmark.sh`
  invocation (with flags) that produced it — this keeps the docs
  reproducible as the CLI evolves.
