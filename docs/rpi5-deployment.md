# Deployment on Raspberry Pi 5 (Raspicam 2 + Cube Orange+ / PX4)

**Target OS:** Debian 13 (Trixie) — the default Raspberry Pi OS for RPi5 as of 2025.

## Overview

OpenVINS requires two ROS 2 topics:

| Topic | Type | Rate |
|---|---|---|
| `/imu0` | `sensor_msgs/Imu` | >= 200 Hz |
| `/cam0/image_raw` | `sensor_msgs/Image` | 20-30 Hz |

Three calibration YAML files are also needed (see [Calibration](#4-calibration) below).

For mono camera mode, launch with:
```bash
ros2 launch ov_msckf subscribe.launch.py config:=<your_config> max_cameras:=1 use_stereo:=false
```

---

## 1. Install ROS 2 on Raspberry Pi 5 (Debian Trixie)

The ROS apt repo at `packages.ros.org/ros2/ubuntu` has a `trixie` distribution, but it only
ships build tooling (`ros-build-essential`, `ros-dev-tools`, `python3-colcon-*`) — **no prebuilt
`ros-humble-*` or `ros-jazzy-*` binary packages**. The two practical options are:

### Option A: Build ROS 2 from source (native, recommended for production)

This uses the official `trixie` repo for build tools, then compiles ROS 2 from source.

```bash
# Add the ROS apt repo (trixie — build tools only)
sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu trixie main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update

# Install build tools
sudo apt install -y \
  ros-dev-tools \
  python3-colcon-common-extensions \
  libeigen3-dev cmake \
  libgoogle-glog-dev libgflags-dev libatlas-base-dev libsuitesparse-dev libceres-dev \
  python3-dev python3-numpy build-essential gcc g++

# Download ROS 2 Jazzy source (latest LTS)
mkdir -p ~/ros2_jazzy/src && cd ~/ros2_jazzy
vcs import src < \
  <(curl -s https://raw.githubusercontent.com/ros2/ros2/jazzy/ros2.repos)

# Install rosdep dependencies
sudo rosdep init || true
rosdep update
rosdep install --from-paths src --ignore-src -y --skip-keys \
  "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"

# Build (takes ~1-2 hours on RPi5)
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

source ~/ros2_jazzy/install/setup.bash
```

Then clone and build OpenVINS against this ROS 2:
```bash
mkdir -p ~/workspace/catkin_ws_ov/src && cd ~/workspace/catkin_ws_ov/src
git clone https://github.com/barakbk-hailo/open_vins/
cd ~/workspace/catkin_ws_ov
source ~/ros2_jazzy/install/setup.bash
colcon build --symlink-install
```

### Option B: Docker (simplest, no source build required)

```bash
# Install Docker
curl -fsSL https://get.docker.com | sh
sudo usermod -aG docker $USER   # re-login after this

# Pull ROS 2 Jazzy image on Ubuntu 24.04 Noble (arm64)
# Noble is the correct match for Debian Trixie (same glibc generation)
docker pull ros:jazzy-ros-base-noble

# Run with host networking and device access (for serial/camera)
docker run -it --rm \
  --network host \
  --privileged \
  -v /dev:/dev \
  -v ~/workspace:/workspace \
  ros:jazzy-ros-base-noble bash
```

Inside the container, install deps and build:
```bash
apt update && apt install -y \
  python3-colcon-common-extensions \
  libeigen3-dev cmake \
  libopencv-dev \
  libboost-system-dev libboost-filesystem-dev libboost-thread-dev libboost-date-time-dev \
  libgoogle-glog-dev libgflags-dev libatlas-base-dev libsuitesparse-dev libceres-dev \
  python3-dev python3-numpy build-essential gcc g++
mkdir -p /workspace/catkin_ws_ov/src && cd /workspace/catkin_ws_ov/src
git clone https://github.com/barakbk-hailo/open_vins/
cd /workspace/catkin_ws_ov
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

To make the container start on boot, use `--restart unless-stopped` with a named container.

---

## 2. Camera — Raspicam 2 (libcamera)

Use the `camera_ros` package which wraps libcamera and publishes `sensor_msgs/Image`:

```bash
sudo apt install -y ros-jazzy-camera-ros libcamera-dev
```

Launch the camera (publishes to `/camera/image_raw` by default):
```bash
ros2 run camera_ros camera_node --ros-args \
  -p width:=640 -p height:=400 -p framerate:=30.0 \
  -r image_raw:=/cam0/image_raw \
  -r camera_info:=/cam0/camera_info
```

Verify images are arriving:
```bash
ros2 topic hz /cam0/image_raw   # should show ~30 Hz
```

---

## 3. IMU — Cube Orange+ with PX4 (MicroXRCE-DDS, recommended)

> **Why not MAVROS?** MAVROS IMU output is limited to ~50 Hz due to MAVLink bandwidth — too slow
> for VIO. The MicroXRCE-DDS bridge publishes raw PX4 IMU data at 200+ Hz over a serial/USB link.

### 3a. Install MicroXRCE-DDS agent on RPi5

```bash
sudo apt install -y ros-jazzy-micro-ros-agent
```

### 3b. Connect Cube Orange+ to RPi5

Connect via USB or UART (Cube Orange+ TELEM2 -> RPi5 UART). PX4 must have `XRCE_DDS_*` parameters
enabled (set `XRCE_DDS_CFG=1002` for UART or `1000` for USB in QGroundControl).

Start the agent:
```bash
# USB:
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 921600
# UART (TELEM2):
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyAMA0 -b 921600
```

### 3c. Bridge px4_msgs -> sensor_msgs/Imu

PX4 publishes `px4_msgs/SensorImu` (or `px4_msgs/VehicleImu`). OpenVINS expects
`sensor_msgs/Imu`. Write a small bridge node:

```bash
pip install px4-msgs  # or build px4_msgs from source
```

Minimal bridge (save as `px4_imu_bridge.py`):
```python
import rclpy
from rclpy.node import Node
from px4_msgs.msg import SensorImu
from sensor_msgs.msg import Imu

class ImuBridge(Node):
    def __init__(self):
        super().__init__('px4_imu_bridge')
        self.pub = self.create_publisher(Imu, '/imu0', 10)
        self.sub = self.create_subscription(SensorImu, '/fmu/out/sensor_imu', self.cb, 10)

    def cb(self, msg):
        out = Imu()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'imu0'
        out.angular_velocity.x = msg.gyro_rad[0]
        out.angular_velocity.y = msg.gyro_rad[1]
        out.angular_velocity.z = msg.gyro_rad[2]
        out.linear_acceleration.x = msg.accelerometer_m_s2[0]
        out.linear_acceleration.y = msg.accelerometer_m_s2[1]
        out.linear_acceleration.z = msg.accelerometer_m_s2[2]
        self.pub.publish(out)

def main():
    rclpy.init()
    rclpy.spin(ImuBridge())
```

Run it:
```bash
source /opt/ros/jazzy/setup.bash
python3 px4_imu_bridge.py
```

Verify:
```bash
ros2 topic hz /imu0   # should show ~200 Hz
```

---

## 4. Calibration

Three YAML files are required — use [Kalibr](https://github.com/ethz-asl/kalibr) to generate them:

| File | Content |
|---|---|
| `camchain.yaml` | Camera intrinsics (focal length, distortion) + T_cam_imu extrinsics |
| `imu.yaml` | IMU noise parameters (gyro/accel noise density, random walk) |
| `estimator_config.yaml` | OpenVINS estimator settings |

### 4a. Collect calibration data

With a printed AprilGrid target, record a bag moving the camera slowly in all axes:
```bash
ros2 bag record /cam0/image_raw /imu0 -o calib_bag
```

### 4b. Run Kalibr (Docker recommended)

```bash
docker run -it --rm -v $(pwd):/data kalibr/kalibr:latest \
  kalibr_calibrate_cameras \
    --bag /data/calib_bag \
    --topics /cam0/image_raw \
    --models pinhole-radtan \
    --target /data/april_grid.yaml
```

Then run IMU-camera calibration:
```bash
kalibr_calibrate_imu_camera \
  --bag /data/calib_bag \
  --cam /data/camchain.yaml \
  --imu /data/imu.yaml \
  --target /data/april_grid.yaml
```

### 4c. Create OpenVINS config

Copy `ov_data/euroc_mav/` as a template and paste your Kalibr output into the YAML files.
Set `max_cameras: 1` and `use_stereo: false` in `estimator_config.yaml`.

---

## 5. Run OpenVINS on RPi5

Once camera, IMU bridge, and calibration are ready, launch in three terminals:

**Terminal 1 — MicroXRCE-DDS agent:**
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 921600
```

**Terminal 2 — IMU bridge + camera:**
```bash
source /opt/ros/jazzy/setup.bash
python3 px4_imu_bridge.py &
ros2 run camera_ros camera_node --ros-args \
  -p width:=640 -p height:=400 -p framerate:=30.0 \
  -r image_raw:=/cam0/image_raw \
  -r camera_info:=/cam0/camera_info
```

**Terminal 3 — OpenVINS:**
```bash
cd ~/workspace/catkin_ws_ov
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch ov_msckf subscribe.launch.py \
  config:=<your_config_name> \
  max_cameras:=1 \
  use_stereo:=false
```

The estimator pose is published on `/ov_msckf/poseimu` (`PoseWithCovarianceStamped`).
