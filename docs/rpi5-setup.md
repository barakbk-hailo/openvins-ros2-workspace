# RPi5 Setup

How to build OpenVINS on a Raspberry Pi 5 and run the EuRoC MAV benchmark.
Live-sensor deployment (camera, IMU, calibration) is sketched in the final
section but is still WIP.

**Target OS:** Debian 13 (Trixie) — the default Raspberry Pi OS for RPi5
as of 2025.

**Required ROS 2 topics (subscribe mode):**

| Topic | Type | Rate |
|---|---|---|
| `/imu0` | `sensor_msgs/Imu` | ≥ 200 Hz |
| `/cam0/image_raw` | `sensor_msgs/Image` | 20-30 Hz |
| `/cam1/image_raw` | `sensor_msgs/Image` (stereo only) | 20-30 Hz |

Three calibration YAMLs are also needed — see §4 below.

---

## 1. Build option A: Docker (recommended)

The ROS apt repo for Debian Trixie ships only build tools, not `ros-humble-*`
binaries, so a native install requires building ROS from source (~1-2 h). The
simpler path is Docker with a pre-built ROS image.

A `Dockerfile_ros2_humble_jammy` is committed in the fork. All the benchmark
results under `results/timing/rpi5/` and `results/rpi5/rerun_2026_04_26_*/`
were produced inside this container.

### 1a. Install Docker on RPi5

```bash
curl -fsSL https://get.docker.com | sh
sudo usermod -aG docker $USER   # re-login after this
```

### 1b. Build the image

The Dockerfile clones the OpenVINS fork into `/opt/ros_ws/src/open_vins/`
and builds it inside the image, so the resulting `openvins-humble:latest`
ships a pre-compiled `/opt/ros_ws/install/` ready to run.

```bash
git clone git@github.com:NadavHHailo/openvins-ros2-workspace.git ~/workspace/catkin_ws_ov
cd ~/workspace/catkin_ws_ov
git submodule update --init --recursive

cd src/open_vins
docker build -t openvins-humble -f Dockerfile_ros2_humble_jammy .
```

Build takes ~25-40 min on RPi5 ARM; the resulting image is ~7 GB. Reclaim
the transient build cache afterwards if the rootfs is tight:

```bash
docker builder prune -af   # frees ~5-10 GB
```

### 1c. Run the container — interactive shell

For development or one-off runs:

```bash
docker run -it --rm \
  --network host --privileged \
  --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -v /dev:/dev \
  -v ~/workspace:/workspace \
  -v ~/datasets:/datasets \
  openvins-humble
```

Two flags that are **not optional** for runtime correctness:

- `--user "$(id -u):$(id -g)"` — files written to host-mounted volumes
  (`/workspace`, `/datasets`) come back owned by the host user, not by root.
  Without this, every benchmark output ends up `root:root` on the host and
  later runs hit "Permission denied" on `~/workspace/catkin_ws_ov/results/`.
- `-e HOME=/tmp` — `--user` makes the container user have no entry in
  `/etc/passwd`, so `HOME` defaults to empty. ros2's `rcl_logging_spdlog`
  then fails to create `~/.ros/log` (resolves to `/.ros/log`) and `ros2 run`
  exits 1 immediately. Setting `HOME=/tmp` lets logging initialise. (The
  `/tmp` choice is arbitrary — anything writable works.)

Inside the container, ROS 2 + the **baked-in** workspace are sourced:

```bash
echo $AMENT_PREFIX_PATH
# /opt/ros_ws/install/<pkg>/...:/opt/ros/humble/...
```

### 1d. Run the container — non-interactive (benchmark scripts)

The benchmark orchestrators (`run_pwt_benchmark_v2.sh`,
`run_full_benchmark.sh`) invoke docker non-interactively. The call pattern
they use, mirrored here for ad-hoc runs:

```bash
docker run --rm --network host --privileged \
  --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -v /dev:/dev \
  -v ~/workspace:/workspace \
  -v ~/datasets:/datasets \
  openvins-humble bash -c '
    set -e
    source /opt/ros_ws/install/setup.bash

    # Use a temp config in the SAME DIRECTORY as the source estimator_config.yaml
    # — the YAML uses relative paths to kalibr_imu_chain.yaml and
    # kalibr_imucam_chain.yaml, so a temp config under /tmp would fail to find
    # those siblings.
    CFG=/workspace/catkin_ws_ov/src/open_vins/config/euroc_mav/estimator_config.yaml
    TMP=/workspace/catkin_ws_ov/src/open_vins/config/euroc_mav/estimator_config_run.yaml
    cp $CFG $TMP
    sed -i "s/record_timing_information: false/record_timing_information: true/" $TMP
    sed -i "s/record_timing_cpu_time: false/record_timing_cpu_time: true/" $TMP
    sed -i "s/record_timing_thread_time: false/record_timing_thread_time: true/" $TMP

    ros2 run ov_msckf ros2_serial_msckf --ros-args \
      -p config_path:=$TMP \
      -p path_bag:=/datasets/euroc/V1_01_easy \
      -p max_cameras:=2 -p use_stereo:=true \
      -p save_total_state:=true \
      -p filepath_est:=/tmp/est.txt -p filepath_std:=/tmp/std.txt

    # Copy outputs to the host-mounted volume
    cp /tmp/traj_timing.txt /workspace/catkin_ws_ov/results/rpi5/run1_wall.txt
    cp /tmp/est.txt          /workspace/catkin_ws_ov/results/rpi5/run1_est.txt

    rm -f $TMP
  '
```

Add Docker flags between `--rm` and `openvins-humble` to mirror specific
benchmark variants:

| Variant | Extra Docker flags |
|---|---|
| `pwt_baseline` | (none beyond the defaults above) |
| `pwt_rtflags` (RT scheduling, pinned cores) | `--cap-add=SYS_NICE --ulimit rtprio=99 --ulimit memlock=-1 --cpuset-cpus=0-3` |

Under `master-candidate`, persistent worker thread + max-interval are baked
into a single image, so `pwt_baseline ≡ pwt_maxinterval` and
`pwt_rtflags ≡ pwt_combined`. See
[determinism.md §6](determinism.md#6-rpi5-follow-up-why-accuracy-variance-doesnt-transfer)
for the cross-run variability tables.

### 1e. `/opt/ros_ws` vs `/workspace` — which wins for what

The image has a workspace baked in at `/opt/ros_ws/`, and the benchmark
flow also bind-mounts the host's workspace at `/workspace/`. The split:

| Path | Source | Authoritative for | Notes |
|---|---|---|---|
| `/opt/ros_ws/install/` | image (built at `docker build` time) | `ros2 run`, `ros2 launch` — these find binaries here via `AMENT_PREFIX_PATH` | Rebuild the image to pick up new C++ code (slow on RPi5 ARM) |
| `/opt/ros_ws/src/open_vins/` | image (cloned at `docker build` time) | the *baked* config + ground-truth files | `git rev-parse HEAD` here tells you which submodule commit the binary was built from |
| `/workspace/catkin_ws_ov/` | host (bind-mounted) | YAML configs the benchmark scripts pass to the binary; output destination | The benchmark scripts read `config_path:=/workspace/catkin_ws_ov/src/open_vins/config/...`, so the *host* submodule's YAML controls runtime parameters even though the binary is from `/opt/ros_ws` |
| `/datasets/euroc/` | host (bind-mounted) | the EuRoC bags | `gdown`/`unzip` happens host-side |

**The split that matters most:** for runtime knobs (e.g.
`slam_chi2_recovery`, `num_pts`, `record_timing_information`), edit the
host's `~/workspace/catkin_ws_ov/src/open_vins/config/euroc_mav/estimator_config.yaml`
— no rebuild needed. For C++ code changes, rebuild the image.

### 1f. Persistent setup (optional)

`pipx` packages and any container-local state do not persist across
`docker run --rm` invocations. For a persistent setup:

- Use `--name openvins` instead of `--rm` and restart with
  `docker start -ai openvins`, or
- Bake extra tools (e.g. `gdown`) into the Dockerfile.

> **WIP:** `Dockerfile_ros2_jazzy_noble` also exists in the fork but requires
> upstream OpenVINS code changes for the ROS 2 Jazzy migration that are not
> yet complete.

---

## 2. Build option B: Native (ROS 2 from source)

Use this if you want to avoid Docker. Takes ~1-2 h on an RPi5.

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
git clone https://github.com/NadavHHailo/open_vins/
cd ~/workspace/catkin_ws_ov
source ~/ros2_jazzy/install/setup.bash
colcon build --symlink-install
```

### Which option should you pick?

| Concern | Docker | Native |
|---|---|---|
| Time to first build | ~10 min | ~1-2 h |
| Isolation from host | strong | none |
| GPU / V4L2 device access | via `--privileged -v /dev:/dev` | direct |
| Match for our published benchmarks | identical | may differ (ROS 2 Jazzy vs Humble) |
| Persistence of tools between runs | requires setup (§1d) | native |

For reproducing our `results/timing/rpi5/` numbers, use **Docker**. For
long-term deployment with live sensors, **native** avoids the Docker passthrough
overhead.

---

## 3. Run the EuRoC benchmark on RPi5

With the workspace built (either option), you can reproduce the x86 flow from
[running.md](running.md) on RPi5. The commands are identical once ROS 2 is
sourced.

### 3a. Download the dataset

Inside the Docker container (or natively):
```bash
sudo apt update && sudo apt install -y pipx
pipx ensurepath && source ~/.bashrc
pipx install gdown

mkdir -p /datasets/euroc && cd /datasets/euroc
gdown 1LFrdiMU6UBjtFfXPHzjJ4L7iDIXcdhvh -O V1_01_easy.zip && unzip V1_01_easy.zip
# Repeat for MH_03_medium, V1_03_difficult, V2_02_medium as needed.
```

### 3b. Serial mode (recommended for benchmarking)

Inside the Docker container — the binary lives in `/opt/ros_ws/install/`
(baked) but the YAML configs come from the host-mounted submodule:

```bash
# (already in the Docker container started per §1c)
source /opt/ros_ws/install/setup.bash   # baked-in workspace, NOT /workspace/...

CFG=/workspace/catkin_ws_ov/src/open_vins/config/euroc_mav/estimator_config.yaml
ros2 launch ov_msckf serial.launch.py \
    config_path:=$CFG \
    path_bag:=/datasets/euroc/V1_01_easy \
    max_cameras:=2 use_stereo:=true
```

> The image's `.bashrc` already runs `source /opt/ros/humble/setup.bash` and
> `source /opt/ros_ws/install/setup.bash` when you open an interactive shell,
> so the `source` line above is redundant for `docker run -it`. It's only
> needed when you spawn a `bash -c '...'` non-interactively (where `.bashrc`
> isn't sourced) — that's why the benchmark script `run_pwt_benchmark_v2.sh`
> always sources it explicitly inside its `docker run ... bash -c '...'`
> invocation.

To reproduce the Phase 4 RPi5 timing numbers, use `run_pwt_benchmark_v2.sh`
from the **host** (not from inside the container — the script itself spawns
its own docker containers per rep):

```bash
# On the RPi5 host shell, NOT inside docker:
cd ~/workspace/catkin_ws_ov
bash run_pwt_benchmark_v2.sh my_baseline openvins-humble 10 -e HOME=/tmp
```

The first three positional args are `<results_subdir> <image_tag> <num_subscribe_reps>`;
extra args after that are forwarded as docker flags. **Always include `-e HOME=/tmp`** —
without it, the container's `--user $(id -u):$(id -g)` strips `HOME` and ros2 fails
to create `~/.ros/log` on first launch.

### 3c. Subscribe mode (realtime test)

Inside the Docker container, terminal 1:
```bash
source /opt/ros_ws/install/setup.bash
CFG=/workspace/catkin_ws_ov/src/open_vins/config/euroc_mav/estimator_config.yaml
ros2 launch ov_msckf subscribe.launch.py config_path:=$CFG
```

Terminal 2 (host or another container shell with the same dataset mount):
```bash
ros2 bag play /datasets/euroc/V1_01_easy --rate 1.0
```

With the persistent worker thread fix ([determinism.md](determinism.md)),
subscribe mode runs at serial speed on RPi5 — stereo baseline is ~24 ms per
frame, comfortably within the 50 ms @ 20 Hz budget. See
[rpi5-benchmarking.md](rpi5-benchmarking.md) for the full measurements.

### 3d. Evaluate accuracy

Inside the Docker container (the host doesn't have `ov_eval` installed
unless you also did the native build of §2):

```bash
source /opt/ros_ws/install/setup.bash
GT_DIR=/opt/ros_ws/src/open_vins/ov_data/euroc_mav
ros2 run ov_eval error_singlerun posyaw \
  $GT_DIR/V1_01_easy.txt state_estimate.txt 8 16 24 32 40
```

> **Format note:** `serial.launch.py` with `save_total_state:=true` writes
> the state-dump format (`timestamp qx qy qz qw px py pz vx ...`, JPL
> quaternion convention), but `error_singlerun` expects TUM
> (`timestamp px py pz qx qy qz qw`). Convert with `awk` first:
> ```bash
> awk '!/^#/ && NF>=8 {print $1,$6,$7,$8,$2,$3,$4,$5}' state_estimate.txt > pose.tum
> ros2 run ov_eval error_singlerun posyaw $GT_DIR/V1_01_easy.txt pose.tum 8 16 24 32 40
> ```
> See [determinism.md §3](determinism.md#3-root-cause-fix-persistent-worker-thread)
> "Format note" for details. Skipping this conversion produces wildly
> inflated ATE (~50× the correct value).

Expected RPi5 numbers for V1_01_easy stereo (serial, default config):
rmse_pos ≈ 0.044 m, rmse_ori ≈ 0.686°. See
[rpi5-benchmarking.md](rpi5-benchmarking.md) §3 for the full table.

---

## 4. Live-sensor deployment (future work / WIP)

The sections below describe wiring a live Raspicam 2 and Cube Orange+ / PX4
IMU to OpenVINS. **This path has not been validated end-to-end** — no results
under `results/` come from a live-sensor run. Treat these as a starting
recipe, not a production guide.

### 4a. Camera — Raspicam 2 (libcamera)

Use the `camera_ros` package which wraps libcamera and publishes
`sensor_msgs/Image`:

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

### 4b. IMU — Cube Orange+ with PX4 (MicroXRCE-DDS, recommended)

> **Why not MAVROS?** MAVROS IMU output is limited to ~50 Hz due to MAVLink
> bandwidth — too slow for VIO. The MicroXRCE-DDS bridge publishes raw PX4
> IMU data at 200+ Hz over a serial/USB link.

Install the agent on RPi5:
```bash
sudo apt install -y ros-jazzy-micro-ros-agent
```

Connect Cube Orange+ via USB or UART (Cube Orange+ TELEM2 → RPi5 UART). PX4
must have `XRCE_DDS_*` parameters enabled (set `XRCE_DDS_CFG=1002` for UART
or `1000` for USB in QGroundControl).

Start the agent:
```bash
# USB:
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 921600
# UART (TELEM2):
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyAMA0 -b 921600
```

PX4 publishes `px4_msgs/SensorImu` (or `px4_msgs/VehicleImu`). OpenVINS expects
`sensor_msgs/Imu`. A minimal bridge (save as `px4_imu_bridge.py`):

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

### 4c. Calibration

Three YAML files are required — use [Kalibr](https://github.com/ethz-asl/kalibr)
to generate them:

| File | Content |
|---|---|
| `camchain.yaml` | Camera intrinsics (focal length, distortion) + T_cam_imu extrinsics |
| `imu.yaml` | IMU noise parameters (gyro/accel noise density, random walk) |
| `estimator_config.yaml` | OpenVINS estimator settings |

Collect calibration data with a printed AprilGrid target; record a bag moving
the camera slowly in all axes:
```bash
ros2 bag record /cam0/image_raw /imu0 -o calib_bag
```

Run Kalibr (Docker recommended):
```bash
docker run -it --rm -v $(pwd):/data kalibr/kalibr:latest \
  kalibr_calibrate_cameras \
    --bag /data/calib_bag \
    --topics /cam0/image_raw \
    --models pinhole-radtan \
    --target /data/april_grid.yaml

kalibr_calibrate_imu_camera \
  --bag /data/calib_bag \
  --cam /data/camchain.yaml \
  --imu /data/imu.yaml \
  --target /data/april_grid.yaml
```

Copy `ov_data/euroc_mav/` as a template and paste your Kalibr output into
the YAML files. Set `max_cameras: 1` and `use_stereo: false` in
`estimator_config.yaml` for mono setups.

### 4d. Live run

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

> **Status of this section:** items pending end-to-end validation —
> live-camera timing at 30 Hz, live-IMU sync latency, and the Kalibr
> workflow on RPi5. Tracked in [rpi5-benchmarking.md](rpi5-benchmarking.md) §WIP.
