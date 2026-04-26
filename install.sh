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

# Select ROS 2 distro based on the host Ubuntu release.
UBUNTU_CODENAME="$(. /etc/os-release && echo "${UBUNTU_CODENAME:-}")"
case "$UBUNTU_CODENAME" in
  noble)  ROS_DISTRO="jazzy" ;;
  jammy)  ROS_DISTRO="humble" ;;
  *)
    echo "Unsupported Ubuntu codename: '${UBUNTU_CODENAME:-unknown}'" >&2
    echo "Supported: jammy (ROS 2 Humble), noble (ROS 2 Jazzy)." >&2
    exit 1
    ;;
esac
echo "=== Target: ROS 2 ${ROS_DISTRO} on Ubuntu ${UBUNTU_CODENAME} ==="

echo "=== [1/4] Adding ROS 2 ${ROS_DISTRO} apt repository ==="
sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME} main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list

# Some noble images (notably Raspberry Pi OS-for-desktop) ship with only
# `noble` + `noble-security` enabled. The security pocket pulls in newer
# runtime libs (libbz2-1.0, libdrm2, libicu74, …) while the matching -dev
# headers live in `noble-updates`; without that pocket apt can't satisfy
# libbz2-dev/libdrm-dev/etc. below and aborts with "unmet dependencies".
UBUNTU_SOURCES="/etc/apt/sources.list.d/ubuntu.sources"
if [ "$UBUNTU_CODENAME" = "noble" ] && [ -f "$UBUNTU_SOURCES" ] \
    && ! grep -qE '^Suites:.*\bnoble-updates\b' "$UBUNTU_SOURCES"; then
  echo "=== Enabling noble-updates pocket in ${UBUNTU_SOURCES} ==="
  sudo sed -i 's/^Suites: noble$/Suites: noble noble-updates/' "$UBUNTU_SOURCES"
fi

sudo apt update

echo "=== [2/4] Installing ROS 2 ${ROS_DISTRO} + dependencies ==="
sudo apt install -y \
  ros-${ROS_DISTRO}-desktop python3-colcon-common-extensions \
  libeigen3-dev cmake \
  libgoogle-glog-dev libgflags-dev libatlas-base-dev libsuitesparse-dev libceres-dev \
  python3-dev python3-matplotlib python3-numpy python3-psutil python3-tk \
  build-essential gcc g++ gdb clang \
  ccache \
  unzip

echo "=== [3/4] Building the workspace ==="
# Jazzy's setup scripts reference unset vars; disable nounset around sourcing.
set +u
source /opt/ros/${ROS_DISTRO}/setup.bash
set -u
cd "$SCRIPT_DIR"

# Throttle parallelism on memory-constrained hosts (Raspberry Pi 5).
# Boost math template instantiations in StateHelper.cpp can push a single
# cc1plus over 1.5 GB; 4-way parallel builds OOM-kill on an 8 GB Pi with
# no swap. Cap to one compile at a time when total RAM <= 8 GB.
TOTAL_MEM_KB=$(awk '/^MemTotal:/ {print $2}' /proc/meminfo)
COLCON_EXTRA=()
if [ "${TOTAL_MEM_KB:-0}" -le $((8 * 1024 * 1024)) ]; then
  echo "=== Low RAM detected ($((TOTAL_MEM_KB / 1024)) MB) — serialising build to avoid OOM ==="
  export MAKEFLAGS="-j1"
  COLCON_EXTRA=(--parallel-workers 1 --executor sequential)
fi

colcon build --symlink-install "${COLCON_EXTRA[@]}" \
  --cmake-args -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache

echo ""
echo "Done. Source the workspace in each new terminal with:"
echo "  source /opt/ros/${ROS_DISTRO}/setup.bash"
echo "  source $SCRIPT_DIR/install/setup.bash"
