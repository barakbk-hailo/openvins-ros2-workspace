# OpenVINS ROS 2 Workspace

This repo ([barakbk-hailo/openvins-ros2-workspace](https://github.com/barakbk-hailo/openvins-ros2-workspace))
is a deployment workspace for our fork of OpenVINS
([barakbk-hailo/open_vins](https://github.com/barakbk-hailo/open_vins)) running with ROS 2 Humble
on Ubuntu 22.04. No GPU required — OpenVINS is a CPU-based MSCKF/EKF algorithm using OpenCV and Eigen.

Our fork adds:
- **Serial deterministic VIO node** (`ros2_serial_msckf`) — ROS 2 port of the ROS 1 serial reader. Processes bag frames sequentially with blocking updates, eliminating message drops from ROS 2 middleware. Produces bit-identical results between runs on the same platform.
- **3-clock timing instrumentation** — process CPU (`CLOCK_PROCESS_CPUTIME_ID`), thread CPU (`CLOCK_THREAD_CPUTIME_ID`), and wall-clock timing recorded per frame in separate CSVs. Also records per-frame feature counts (SLAM, MSCKF, delayed init). All disabled by default.
- **Persistent worker thread** — replaces the upstream per-frame `detach()` dispatch in subscribe mode with a single persistent processing thread. Fixes a TOCTOU race, dangling-reference UB, and non-deterministic IMU triggering. Reduces subscribe-mode overhead from 2x serial to 1x serial.
- **SLAM recovery mechanism** — relaxes the chi-squared gate for delayed feature init when SLAM features drop below 25% of max, preventing an irrecoverable empty-state feedback loop (defense-in-depth).
- **Configurable `multi_threading_subs`** — moved from hardcoded to a YAML parameter, allowing async vs inline VIO dispatch without recompiling.
- **Custom RPE segment lengths** — `error_singlerun` accepts optional segment lengths from the command line.
- **Launch file improvements** — `filepath_est`/`filepath_std` args for configurable output paths; `on_exit=Shutdown()` for clean exit in automated benchmark loops.
- **Docker images** for ROS 2 Humble (RPi5 / Debian Trixie) and ROS 2 Jazzy (WIP).
- **Benchmarking scripts** — orchestrate serial/subscribe runs across sequences, config sweeps, and 3-clock collection. Stored results (CSV timing + trajectory estimates) for x86 and RPi5.

## Quick install

```bash
git clone --recursive git@github.com:barakbk-hailo/openvins-ros2-workspace.git ~/workspace/catkin_ws_ov
cd ~/workspace/catkin_ws_ov
bash install.sh
```

## Documentation

| Guide | Description |
|---|---|
| [Installation](docs/installation.md) | Native build on Ubuntu 22.04 (ROS 2 Humble) |
| [Running EuRoC](docs/running-euroc.md) | Download dataset, launch OpenVINS, visualize in RViz |
| [Evaluation](docs/evaluation.md) | ATE/RPE benchmarks, paper comparison, reproduction script |
| [Determinism](docs/determinism.md) | Subscribe-mode determinism: persistent worker thread root-cause fix + SLAM recovery safety net + other fork changes |
| [Benchmark Analysis](docs/benchmark-analysis.md) | Paper comparison, 3-clock timing, accuracy, consistency, RPi5 projections (latest data) |
| [Timing](docs/timing.md) | Per-component timing breakdown, config sensitivity, realtime feasibility, RPi5 projections |
| [RPi5 Setup](docs/rpi5-setup.md) | Build on RPi5 (Docker or native) + run the EuRoC benchmark; live-sensor deployment is WIP |
| [RPi5 Benchmarking](docs/rpi5-benchmarking.md) | Phase 4 RPi5 timing, accuracy vs x86, config sweeps, subscribe mode, WIP list |
