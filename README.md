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

For a one-command setup see [`install.sh`](#quick-install) below.

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

## Running the EuRoC MAV Example

Reference: https://docs.openvins.com/gs-tutorial.html

### 1. Download the dataset

Install `gdown` to download from Google Drive:

```bash
pip install gdown
# In Docker (where pip packages may not persist), use pipx instead:
#   sudo apt install -y pipx && pipx ensurepath && pipx install gdown
```

Download and extract the EuRoC V1_01_easy ROS 2 bag (~900 MB, already converted):

```bash
mkdir -p ~/datasets/euroc && cd ~/datasets/euroc
gdown 1LFrdiMU6UBjtFfXPHzjJ4L7iDIXcdhvh -O V1_01_easy.zip
unzip V1_01_easy.zip
```

### 2. Terminal 1 — launch OpenVINS

```bash
cd ~/workspace/catkin_ws_ov
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ov_msckf subscribe.launch.py config:=euroc_mav
```

### 3. Terminal 2 — play the bag

```bash
source /opt/ros/humble/setup.bash
cd ~/datasets/euroc
ros2 bag play V1_01_easy
```

### 4. (Optional) Visualize in RViz

Two one-time setup steps are required on Intel integrated graphics:

**1. Force Ogre to use the GL3Plus render system:**

```bash
mkdir -p ~/.rviz2 && cat > ~/.rviz2/ogre.cfg << 'EOF'
[General]
Render System=OpenGL 3+ Rendering Subsystem

[OpenGL 3+ Rendering Subsystem]
FSAA=0
Full Screen=No
RTT Preferred Mode=FBO
Video Mode=1280 x 720
sRGB Gamma Conversion=No
EOF
```

**2.** The bundled `display.rviz` config has been updated to use ROS 2 plugin names — no action needed,
it is already in the repo.

Then in Terminal 3:

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/catkin_ws_ov/install/setup.bash
ros2 run rqt_image_view rqt_image_view &
rviz2 -d ~/workspace/catkin_ws_ov/src/open_vins/ov_msckf/launch/display.rviz
```

`rqt_image_view` launches in the background; select `/ov_msckf/trackhist` from its dropdown to see
feature tracks. The rviz2 window shows the 3D trajectory and point clouds.

Once all three terminals are running, you will see the green VIO path growing in rviz2 as OpenVINS
processes the dataset frames.

> **Note:** The rviz2 `Image` display plugin crashes on Intel integrated graphics with GL3Plus. Use
> `rqt_image_view` for image topics instead.

## Evaluation (ATE / RPE)

### Real-time dependency and reproducibility

The ROS 2 `subscribe` node processes messages as they arrive from `ros2 bag play`.
If OpenVINS cannot keep up, IMU or image messages are dropped from the subscription
queue, degrading accuracy. The paper's results used the ROS 1 `serial` node, which
reads bag frames sequentially — blocking until each frame is processed before
reading the next — and is entirely CPU-speed-independent.

This workspace includes a ROS 2 port of that serial node: `ros2_serial_msckf`.
It reads the bag directory directly (no `ros2 bag play` needed), processes every
message regardless of CPU speed, and is the recommended way to run benchmarks.

> **Note on reproducibility:** the serial node produces bit-identical results
> between runs. This is because (a) messages are processed in a fixed order, and
> (b) since June 2021 (commit `fae7144`, post-paper), OpenVINS calls
> `cv::setRNGSeed(0)` at startup, making RANSAC sampling deterministic.
>
> The ICRA 2020 paper predates this fix — at that time RANSAC used an unseeded
> (time-based) RNG, so even the ROS 1 serial node produced different results on
> each run. This is why the paper averaged over 10 runs. With the current codebase,
> 10-run averaging is only needed if you use Option B (`ros2 bag play`), where
> timing-induced message drops shift the RNG call sequence.

#### Option A — Serial node (recommended)

**Terminal 1 — start the recorder first** (so no poses are missed):

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/results && cd ~/results
python3 ~/workspace/catkin_ws_ov/record_poses.py
```

**Terminal 2 — run OpenVINS** (reads the bag directly, no `ros2 bag play` needed):

```bash
cd ~/workspace/catkin_ws_ov
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch ov_msckf serial.launch.py \
    config:=euroc_mav \
    path_bag:=$HOME/datasets/euroc/V1_01_easy
```

Press Ctrl+C in Terminal 1 after OpenVINS finishes (it exits on its own).
This saves `state_estimate.txt` and `state_groundtruth.txt` in `~/results`.

#### Option B — Subscribe node with reduced rate (legacy)

**Terminal 1 — start the recorder first:**

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/results && cd ~/results
python3 ~/workspace/catkin_ws_ov/record_poses.py
```

**Terminal 2 — launch OpenVINS:**

```bash
cd ~/workspace/catkin_ws_ov
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch ov_msckf subscribe.launch.py config:=euroc_mav
```

**Terminal 3 — play the bag at reduced rate** (minimises message drops):

```bash
source /opt/ros/humble/setup.bash
cd ~/datasets/euroc
ros2 bag play V1_01_easy --rate 0.1   # 10× slower than real-time
```

Press Ctrl+C in Terminal 1 after the bag finishes.

To check for timestamp gaps in a recorded estimate (which would indicate dropped
messages), skip the comment header line:

```bash
awk '!/^#/{if(prev && $1-prev>0.1) print NR, $1-prev"s gap"; prev=$1}' state_estimate.txt
```

No output means no gaps.

### 2. Compute ATE and RPE

The fork ships pre-formatted ground truth `.txt` files in `ov_data/euroc_mav/`
(no conversion needed). Pass the six segment lengths from the paper's Table III:

```bash
source ~/workspace/catkin_ws_ov/install/setup.bash
cd ~/results
GT_DIR=~/workspace/catkin_ws_ov/src/open_vins/ov_data/euroc_mav
ros2 run ov_eval error_singlerun posyaw $GT_DIR/V1_01_easy.txt state_estimate.txt 8 16 24 32 40 48
```

### Reference results (V1_01_easy, stereo, Intel Iris Xe)

Both runs are single-run results on this machine. Serial mode is deterministic
(bit-identical across repeated runs); subscribe mode at `--rate 0.1` is not.

#### Serial node (`ros2_serial_msckf`)

```
======================================
Absolute Trajectory Error
======================================
rmse_ori = 0.569 | rmse_pos = 0.038
mean_ori = 0.501 | mean_pos = 0.034
min_ori  = 0.093 | min_pos  = 0.006
max_ori  = 1.609 | max_pos  = 0.078
std_ori  = 0.270 | std_pos  = 0.017
======================================
Relative Pose Error
======================================
seg  8 - median_ori = 0.528 | median_pos = 0.057 (2361 samples)
seg 16 - median_ori = 0.368 | median_pos = 0.051 (2058 samples)
seg 24 - median_ori = 0.467 | median_pos = 0.047 (1793 samples)
seg 32 - median_ori = 0.565 | median_pos = 0.051 (1401 samples)
seg 40 - median_ori = 0.600 | median_pos = 0.038 (1079 samples)
```

#### Subscribe node (`run_subscribe_msckf`, `--rate 0.1`)

```
======================================
Absolute Trajectory Error
======================================
rmse_ori = 0.731 | rmse_pos = 0.051
mean_ori = 0.653 | mean_pos = 0.048
min_ori  = 0.046 | min_pos  = 0.009
max_ori  = 2.146 | max_pos  = 0.097
std_ori  = 0.330 | std_pos  = 0.019
======================================
Relative Pose Error
======================================
seg  8 - median_ori = 0.608 | median_pos = 0.059 (2379 samples)
seg 16 - median_ori = 0.588 | median_pos = 0.058 (2074 samples)
seg 24 - median_ori = 0.623 | median_pos = 0.072 (1807 samples)
seg 32 - median_ori = 0.952 | median_pos = 0.092 (1412 samples)
seg 40 - median_ori = 0.847 | median_pos = 0.086 (1087 samples)
```

> **Note on sample counts:** the subscribe node reports slightly more RPE samples
> than the serial node (e.g. 2379 vs 2361 for seg 8). The serial stereo-sync
> algorithm discards camera frames where no partner is found within ±0.02 s,
> while the subscribe node's `message_filters::ApproximateTime` policy is slightly
> more permissive. The difference is small (~1 %) and does not affect comparability.

#### Comparison with the paper (Geneva et al. ICRA 2020, Table II / III)

The paper reports the **mean ATE over 10 runs** (Table II); our figures are single-run RMSEs.
The paper's RPE (Table III) is averaged over **all EuRoC datasets** (not V1_01_easy alone),
so RPE figures are not directly comparable to our per-sequence results.

**ATE — V1_01_easy, stereo (Table II)**

| Metric | Serial (1 run) | Subscribe 0.1x (1 run) | Paper (10-run mean) |
|---|---|---|---|
| Orientation (deg) | **0.569** | 0.731 | 0.905 |
| Position (m) | **0.038** | 0.051 | 0.061 |

**RPE — stereo_ov_vio (Table III, all-dataset average)**

| Segment | Serial — deg / m | Subscribe 0.1x — deg / m | Paper — deg / m |
|---|---|---|---|
| 8 m | **0.528 / 0.057** | 0.608 / 0.059 | 0.722 / 0.068 |
| 16 m | **0.368 / 0.051** | 0.588 / 0.058 | 0.892 / 0.077 |
| 24 m | **0.467 / 0.047** | 0.623 / 0.072 | 1.089 / 0.087 |
| 32 m | **0.565 / 0.051** | 0.952 / 0.092 | 1.218 / 0.088 |
| 40 m | **0.600 / 0.038** | 0.847 / 0.086 | 1.342 / 0.101 |

The serial node outperforms the subscribe node on every metric. The improvement is
most pronounced on longer RPE segments (32 m, 40 m), where the subscribe node's
residual message drops accumulate into larger drift errors.

Our single-sequence results beat the paper's all-dataset averages because
V1_01_easy is the easiest sequence; harder sequences (V1_03, MH_04, MH_05)
will pull the average up.

> **Note on mono vs stereo orientation ATE:** the paper shows `mono_ov_vio` with a
> lower orientation ATE on V1_01_easy (0.642 °) than stereo (0.905 °), even though
> stereo wins on every RPE segment. RPE measures local drift and is the more reliable
> indicator of systematic performance; the ATE orientation inversion is a statistical
> artefact of a 10-sample average on a short, easy sequence.

### Reproducing the 10-run mean

> **Note:** the serial node is deterministic — it produces bit-identical results
> across runs (see note above). The 10-run average is only meaningful when using
> the subscribe node (Option B), where RANSAC timing jitter causes run-to-run variance.

Run the subscribe node ten times, saving the estimate under a different name each time.
Restart OpenVINS between each run:

```bash
source /opt/ros/humble/setup.bash && source ~/workspace/catkin_ws_ov/install/setup.bash
mkdir -p ~/results

for i in $(seq -w 1 10); do
  # Start recording in the background
  cd ~/results
  python3 ~/workspace/catkin_ws_ov/record_poses.py &
  RECORD_PID=$!

  # Launch OpenVINS subscribe node in the background
  ros2 launch ov_msckf subscribe.launch.py config:=euroc_mav &
  OV_PID=$!

  # Play the bag at reduced rate
  cd ~/datasets/euroc
  ros2 bag play V1_01_easy --rate 0.1

  # Stop recorder and OpenVINS
  kill $RECORD_PID $OV_PID 2>/dev/null; wait $RECORD_PID $OV_PID 2>/dev/null
  mv ~/results/state_estimate.txt ~/results/state_estimate_run${i}.txt
done
```

Then compute the ATE scalar for each run and average:

```bash
GT_DIR=~/workspace/catkin_ws_ov/src/open_vins/ov_data/euroc_mav
cd ~/results
for i in $(seq -w 1 10); do
  ros2 run ov_eval error_singlerun posyaw $GT_DIR/V1_01_easy.txt state_estimate_run${i}.txt \
    | grep rmse_pos
done
```

The mean of the ten `rmse_pos` values is the number comparable to Table II of the paper.

### Full EuRoC benchmark (all sequences)

The paper (Table II) reports ATE over the 5 Vicon room sequences (V2_03 excluded
due to some algorithms failing on it). Download all Vicon bags:

```bash
mkdir -p ~/datasets/euroc && cd ~/datasets/euroc
pip install gdown   # or: sudo apt install pipx && pipx install gdown

# EuRoC MAV ROS 2 bags from the OpenVINS docs (Google Drive)
gdown 1LFrdiMU6UBjtFfXPHzjJ4L7iDIXcdhvh -O V1_01_easy.zip
gdown 1rlGSy7h38ucm8jr8ssH-sJPX84JfkBtX -O V1_02_medium.zip
gdown 1Gy1zc4LaMlwsLpXBqOIci6Y3cV_5r-0k -O V1_03_difficult.zip
gdown 1KAkE8Ptq3eSQlXMozJgzNIAVUBH3h0FP -O V2_01_easy.zip
gdown 1Gj4psmvcAwYwCp4T4CQH-d2ZVJ09d3x2 -O V2_02_medium.zip

for f in *.zip; do unzip -o "$f"; done
```

Run the serial node over every sequence and compute ATE:

```bash
source /opt/ros/humble/setup.bash && source ~/workspace/catkin_ws_ov/install/setup.bash
export MPLBACKEND=Agg   # prevent error_singlerun from blocking on plt.show()
GT_DIR=~/workspace/catkin_ws_ov/src/open_vins/ov_data/euroc_mav
mkdir -p ~/results

SEQUENCES=(V1_01_easy V1_02_medium V1_03_difficult V2_01_easy V2_02_medium)

for seq in "${SEQUENCES[@]}"; do
  echo "=== $seq ==="

  # Start recorder in the background
  cd ~/results
  python3 ~/workspace/catkin_ws_ov/record_poses.py &
  RECORD_PID=$!
  sleep 2

  # Run serial node (blocks until bag is fully processed)
  ros2 launch ov_msckf serial.launch.py \
      config:=euroc_mav \
      path_bag:=$HOME/datasets/euroc/$seq

  # Give recorder time to flush, then stop it
  sleep 2
  kill $RECORD_PID 2>/dev/null
  wait $RECORD_PID 2>/dev/null
  mv ~/results/state_estimate.txt ~/results/estimate_${seq}.txt

  # Evaluate
  ros2 run ov_eval error_singlerun posyaw $GT_DIR/${seq}.txt ~/results/estimate_${seq}.txt \
    | grep rmse
  echo ""
done
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

---

## Docker installation (RPi5 / Debian Trixie)

For Raspberry Pi 5 running Debian 13 (Trixie), the easiest path is Docker with
the pre-built ROS 2 Humble image. A `Dockerfile_ros2_humble_jammy` is included in
the fork.

### 1. Install Docker on RPi5

```bash
curl -fsSL https://get.docker.com | sh
sudo usermod -aG docker $USER   # re-login after this
```

### 2. Build the image

```bash
cd ~/workspace
git clone git@github.com:barakbk-hailo/open_vins.git
cd open_vins
docker build -t openvins-humble -f Dockerfile_ros2_humble_jammy .
```

### 3. Run the container

```bash
docker run -it --rm \
  --network host --privileged \
  -v /dev:/dev \
  -v ~/workspace:/workspace \
  -v ~/datasets:/datasets \
  openvins-humble
```

Inside the container, ROS 2 and the workspace are already sourced.

### 4. Download datasets inside the container

`gdown` is not baked into the image (to keep it small). Install it with `pipx`:

```bash
sudo apt update && sudo apt install -y pipx
pipx ensurepath
source ~/.bashrc   # pick up the new PATH
pipx install gdown

mkdir -p /datasets/euroc && cd /datasets/euroc
gdown 1LFrdiMU6UBjtFfXPHzjJ4L7iDIXcdhvh -O V1_01_easy.zip && unzip V1_01_easy.zip
```

> **Note:** `pipx` packages are installed per-container and do not persist across
> `docker run` invocations unless you mount or commit the container. For a persistent
> setup, add the `pipx install gdown` step to the Dockerfile or use a named container
> (`docker run --name openvins ...` instead of `--rm`).

A `Dockerfile_ros2_jazzy_noble` also exists in the fork but is **WIP** — it requires
upstream OpenVINS code changes for the ROS 2 Jazzy migration that are not yet complete.

---

## Deployment on Raspberry Pi 5 (Raspicam 2 + Cube Orange+ / PX4)

**Target OS:** Debian 13 (Trixie) — the default Raspberry Pi OS for RPi5 as of 2025.

### Overview

OpenVINS requires two ROS 2 topics:

| Topic | Type | Rate |
|---|---|---|
| `/imu0` | `sensor_msgs/Imu` | ≥ 200 Hz |
| `/cam0/image_raw` | `sensor_msgs/Image` | 20–30 Hz |

Three calibration YAML files are also needed (see [Calibration](#calibration) below).

For mono camera mode, launch with:
```bash
ros2 launch ov_msckf subscribe.launch.py config:=<your_config> max_cameras:=1 use_stereo:=false
```

---

### 1. Install ROS 2 on Raspberry Pi 5 (Debian Trixie)

The ROS apt repo at `packages.ros.org/ros2/ubuntu` has a `trixie` distribution, but it only
ships build tooling (`ros-build-essential`, `ros-dev-tools`, `python3-colcon-*`) — **no prebuilt
`ros-humble-*` or `ros-jazzy-*` binary packages**. The two practical options are:

#### Option A: Build ROS 2 from source (native, recommended for production)

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

#### Option B: Docker (simplest, no source build required)

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

### 2. Camera — Raspicam 2 (libcamera)

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

### 3. IMU — Cube Orange+ with PX4 (MicroXRCE-DDS, recommended)

> **Why not MAVROS?** MAVROS IMU output is limited to ~50 Hz due to MAVLink bandwidth — too slow
> for VIO. The MicroXRCE-DDS bridge publishes raw PX4 IMU data at 200+ Hz over a serial/USB link.

#### 3a. Install MicroXRCE-DDS agent on RPi5

```bash
sudo apt install -y ros-jazzy-micro-ros-agent
```

#### 3b. Connect Cube Orange+ to RPi5

Connect via USB or UART (Cube Orange+ TELEM2 → RPi5 UART). PX4 must have `XRCE_DDS_*` parameters
enabled (set `XRCE_DDS_CFG=1002` for UART or `1000` for USB in QGroundControl).

Start the agent:
```bash
# USB:
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 921600
# UART (TELEM2):
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyAMA0 -b 921600
```

#### 3c. Bridge px4_msgs → sensor_msgs/Imu

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

### 4. Calibration

Three YAML files are required — use [Kalibr](https://github.com/ethz-asl/kalibr) to generate them:

| File | Content |
|---|---|
| `camchain.yaml` | Camera intrinsics (focal length, distortion) + T_cam_imu extrinsics |
| `imu.yaml` | IMU noise parameters (gyro/accel noise density, random walk) |
| `estimator_config.yaml` | OpenVINS estimator settings |

#### 4a. Collect calibration data

With a printed AprilGrid target, record a bag moving the camera slowly in all axes:
```bash
ros2 bag record /cam0/image_raw /imu0 -o calib_bag
```

#### 4b. Run Kalibr (Docker recommended)

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

#### 4c. Create OpenVINS config

Copy `ov_data/euroc_mav/` as a template and paste your Kalibr output into the YAML files.
Set `max_cameras: 1` and `use_stereo: false` in `estimator_config.yaml`.

---

### 5. Run OpenVINS on RPi5

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
