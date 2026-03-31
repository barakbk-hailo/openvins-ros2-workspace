# OpenVINS ROS 2 Workspace

This repo ([barakbk-hailo/openvins-ros2-workspace](https://github.com/barakbk-hailo/openvins-ros2-workspace))
is a deployment workspace for our fork of OpenVINS
([barakbk-hailo/open_vins](https://github.com/barakbk-hailo/open_vins)) running with ROS 2 Humble
on Ubuntu 22.04. No GPU required — OpenVINS is a CPU-based MSCKF/EKF algorithm using OpenCV and Eigen.

Our fork adds:
- A ROS 2 port of the deterministic **serial VIO node** (`ros2_serial_msckf`) which processes
  bag frames sequentially and is CPU-speed-independent (eliminates message-drop errors under ROS 2)
- **Docker images** for ROS 2 Humble (RPi5 / Debian Trixie) and ROS 2 Jazzy (WIP — requires
  upstream code changes for the Jazzy migration, not yet complete)
- A minimal RViz config and updated launch files

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
| [Docker (RPi5)](docs/docker.md) | Containerized build for Raspberry Pi 5 / Debian Trixie |
| [RPi5 Deployment](docs/rpi5-deployment.md) | Camera, IMU, calibration, live VIO on RPi5 |
