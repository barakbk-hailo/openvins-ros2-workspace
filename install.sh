#!/usr/bin/env bash
# One-shot setup for the OpenVINS ROS 2 workspace.
# Run from the repo root after cloning:
#   git clone --recursive git@github.com:barakbk-hailo/openvins-ros2-workspace.git ~/workspace/catkin_ws_ov
#   cd ~/workspace/catkin_ws_ov && bash install.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Ensure submodule is initialised (in case cloned without --recursive)
if [ ! -f "$SCRIPT_DIR/src/open_vins/CMakeLists.txt" ]; then
  echo "=== [0/4] Initialising open_vins submodule ==="
  git -C "$SCRIPT_DIR" submodule update --init --recursive
fi

echo "=== [1/4] Adding ROS 2 Humble apt repository ==="
sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu jammy main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update

echo "=== [2/4] Installing ROS 2 Humble + dependencies ==="
sudo apt install -y \
  ros-humble-desktop python3-colcon-common-extensions \
  libeigen3-dev cmake \
  libgoogle-glog-dev libgflags-dev libatlas-base-dev libsuitesparse-dev libceres-dev \
  python3-dev python3-matplotlib python3-numpy python3-psutil python3-tk \
  build-essential gcc g++ gdb clang

echo "=== [3/4] Building the workspace ==="
source /opt/ros/humble/setup.bash
cd "$SCRIPT_DIR"
colcon build --symlink-install

echo ""
echo "Done. Source the workspace in each new terminal with:"
echo "  source /opt/ros/humble/setup.bash"
echo "  source $SCRIPT_DIR/install/setup.bash"
