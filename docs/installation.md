# Installation (Ubuntu 22.04 / ROS 2 Humble)

## Prerequisites

- Ubuntu 22.04 (Jammy)
- UTF-8 locale (any, e.g. `en_US.UTF-8`, `en_IL.UTF-8`)
- Internet access + sudo

## Quick install

Clone this repo (with the `open_vins` submodule), then run the install script:

```bash
git clone --recursive git@github.com:barakbk-hailo/openvins-ros2-workspace.git ~/workspace/catkin_ws_ov
cd ~/workspace/catkin_ws_ov
bash install.sh
```

## Manual steps

### 0. Clone this workspace repo (with submodule)

```bash
git clone --recursive git@github.com:barakbk-hailo/openvins-ros2-workspace.git ~/workspace/catkin_ws_ov
```

If you already cloned without `--recursive`:
```bash
cd ~/workspace/catkin_ws_ov
git submodule update --init --recursive
```

### 1. Add ROS 2 Humble apt repository

```bash
sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu jammy main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update
```

### 2. Install ROS 2 Humble + all dependencies

```bash
sudo apt install -y \
  ros-humble-desktop \
  python3-colcon-common-extensions \
  libeigen3-dev cmake \
  libgoogle-glog-dev libgflags-dev libatlas-base-dev libsuitesparse-dev libceres-dev \
  python3-dev python3-matplotlib python3-numpy python3-psutil python3-tk \
  build-essential gcc g++ gdb clang
```

### 3. Build the workspace

```bash
cd ~/workspace/catkin_ws_ov
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

Expected output (warnings about deprecated tf2/image_transport headers are harmless):
```
Summary: 5 packages finished [~5min]
  2 packages had stderr output: ov_core ov_msckf
```

### 4. Source the workspace (every new terminal)

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/catkin_ws_ov/install/setup.bash
```

## Notes

- The deprecated-header warnings from `ov_core` and `ov_msckf` are benign — ROS 2 Humble ships
  slightly outdated `.h` wrappers for tf2_geometry_msgs and image_transport. They work fine.
- No CUDA or OpenGL is needed to build or run the estimator. RViz (included in `ros-humble-desktop`)
  requires a display for rendering but the estimator itself runs headless.
- If you want a lighter install without RViz, replace `ros-humble-desktop` with `ros-humble-ros-base`.
- The `display.rviz` config has been updated from ROS 1 to ROS 2 plugin names (`rviz_default_plugins/`
  and `rviz_common/` namespaces) and simplified — this was the main fix for rviz2 crashes on Intel
  integrated graphics.
