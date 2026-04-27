# Installation (Ubuntu 24.04 / ROS 2 Jazzy, Ubuntu 22.04 / ROS 2 Humble)

## Prerequisites

- Ubuntu 24.04 (Noble) → ROS 2 **Jazzy**, or Ubuntu 22.04 (Jammy) → ROS 2 **Humble**
- UTF-8 locale (any, e.g. `en_US.UTF-8`, `en_IL.UTF-8`)
- Internet access + sudo

> **Distro auto-detect:** the commands below derive `$ROS_DISTRO` from
> `/etc/os-release` so the same copy-paste block works on both Ubuntu 22.04
> (→ humble) and Ubuntu 24.04 (→ jazzy). Run this once at the top of your
> terminal:
>
> ```bash
> . /etc/os-release
> case "$VERSION_CODENAME" in
>   noble) export ROS_DISTRO=jazzy ;;
>   jammy) export ROS_DISTRO=humble ;;
>   *) echo "Unsupported Ubuntu: $VERSION_CODENAME" >&2 ;;
> esac
> echo "ROS_DISTRO=$ROS_DISTRO  UBUNTU=$VERSION_CODENAME"
> ```

## Quick install

Clone this repo (with the `open_vins` submodule), then run the install script:

```bash
git clone --recursive git@github.com:NadavHHailo/openvins-ros2-workspace.git ~/workspace/catkin_ws_ov
cd ~/workspace/catkin_ws_ov
bash install.sh
```

## Manual steps

### 0. Clone this workspace repo (with submodule)

```bash
git clone --recursive git@github.com:NadavHHailo/openvins-ros2-workspace.git ~/workspace/catkin_ws_ov
```

If you already cloned without `--recursive`:
```bash
cd ~/workspace/catkin_ws_ov
git submodule update --init --recursive
```

### 1. Add ROS 2 apt repository

Uses `$VERSION_CODENAME` from the prerequisites block above (`noble` or
`jammy`):

```bash
sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $VERSION_CODENAME main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update
```

### 2. Install ROS 2 + all dependencies

Uses `$ROS_DISTRO` from the prerequisites block above (`jazzy` or `humble`):

```bash
sudo apt install -y \
  ros-${ROS_DISTRO}-desktop \
  python3-colcon-common-extensions \
  libeigen3-dev cmake \
  libgoogle-glog-dev libgflags-dev libatlas-base-dev libsuitesparse-dev libceres-dev \
  python3-dev python3-matplotlib python3-numpy python3-psutil python3-tk \
  build-essential gcc g++ gdb clang
```

### 3. Build the workspace

```bash
cd ~/workspace/catkin_ws_ov
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --symlink-install
```

Expected output (warnings about deprecated tf2/image_transport headers are harmless):
```
Summary: 5 packages finished [~5min]
  2 packages had stderr output: ov_core ov_msckf
```

### 4. Source the workspace (every new terminal)

Auto-detects the installed distro from `/opt/ros/`:

```bash
. /etc/os-release && source /opt/ros/$([ "$UBUNTU_CODENAME" = "noble" ] && echo jazzy || echo humble)/setup.bash
source ~/workspace/catkin_ws_ov/install/setup.bash
```

## Notes

- The deprecated-header warnings from `ov_core` and `ov_msckf` are benign — ROS 2 ships
  slightly outdated `.h` wrappers for tf2_geometry_msgs and image_transport. They work fine.
- No CUDA or OpenGL is needed to build or run the estimator. RViz (included in `ros-${ROS_DISTRO}-desktop`)
  requires a display for rendering but the estimator itself runs headless.
- If you want a lighter install without RViz, replace `ros-${ROS_DISTRO}-desktop` with `ros-${ROS_DISTRO}-ros-base`.
- The `display.rviz` config has been updated from ROS 1 to ROS 2 plugin names (`rviz_default_plugins/`
  and `rviz_common/` namespaces) and simplified — this was the main fix for rviz2 crashes on Intel
  integrated graphics.
