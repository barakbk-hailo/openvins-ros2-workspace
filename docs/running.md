# Running OpenVINS

> **ROS 2 distro:** commands below use `jazzy` (Ubuntu 24.04 / Noble) as the default.
> On Ubuntu 22.04 / Jammy, substitute `humble`:
> `source /opt/ros/jazzy/setup.bash` instead of `source /opt/ros/jazzy/setup.bash`.

Two ways to run the VIO pipeline:

- **Serial mode** (`ros2_serial_msckf`) — reads a bag file directly, processes
  frames sequentially with blocking updates. Deterministic (bit-identical
  across runs on the same platform) and CPU-speed-independent. **Recommended
  for benchmarking and paper reproduction.**
- **Subscribe mode** (`run_subscribe_msckf`) — subscribes to ROS 2 topics,
  processes frames as they arrive. What real hardware deployments use.

This doc shows both on the EuRoC MAV dataset. For the full benchmark
workflow (recorder, ATE/RPE evaluation, paper comparison) see
[evaluation.md](evaluation.md). For RPi5 deployment see
[rpi5-setup.md](rpi5-setup.md).

Reference: https://docs.openvins.com/gs-tutorial.html

## 1. Download the dataset

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

## 2. Serial mode (recommended for benchmarking)

Reads the bag directly — no `ros2 bag play` needed, no frame drops, and
produces identical output on every run for a given platform.

```bash
cd ~/workspace/catkin_ws_ov
source /opt/ros/jazzy/setup.bash && source install/setup.bash

ros2 launch ov_msckf serial.launch.py \
    config_path:=$PWD/src/open_vins/config/euroc_mav/estimator_config.yaml \
    path_bag:=$HOME/datasets/euroc/V1_01_easy \
    max_cameras:=2 use_stereo:=true
```

The launcher blocks until the bag finishes processing, then exits.

To record the estimate for accuracy evaluation, start `record_poses.py` in a
separate terminal **before** launching OpenVINS — see
[evaluation.md](evaluation.md) §"Option A — Serial node" for the complete
recorder-first flow.

For per-frame timing CSVs, set `record_timing_information: true` in the
`estimator_config.yaml` (or use the benchmark orchestrator
`run_full_benchmark.sh` from the workspace root, which handles the temp-config
dance automatically).

## 3. Subscribe mode (realtime)

What actual robots run. Launch OpenVINS and play the bag in separate
terminals.

**Terminal 1 — launch OpenVINS:**

```bash
cd ~/workspace/catkin_ws_ov
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch ov_msckf subscribe.launch.py config:=euroc_mav
```

**Terminal 2 — play the bag:**

```bash
source /opt/ros/jazzy/setup.bash
cd ~/datasets/euroc
ros2 bag play V1_01_easy
```

With the persistent worker thread fix (see [determinism.md](determinism.md)),
subscribe mode now runs at serial speed with no frame drops and no SLAM
collapse. If you want to reproduce the pre-fix failure modes, check out a
commit before the persistent-worker-thread branch was merged.

## 4. (Optional) Visualize in RViz

### Prerequisite: a reachable display

RViz and `rqt_image_view` are Qt GUI apps and need `$DISPLAY` (or Wayland)
pointing at a running compositor. If you are SSH'd in without X forwarding
you'll see:

```
qt.qpa.xcb: could not connect to display
This application failed to start because no Qt platform plugin could be initialized.
```

Pick **one** of the following, depending on where you want the window:

**A) Show the GUI on the machine's own monitor** (most common — user already
logged into GNOME/Wayland on the laptop, SSH'd in from elsewhere):

```bash
# Confirm a session exists for your UID (look for an Xwayland or Xorg process
# and an X socket owned by you)
ls /tmp/.X11-unix/                           # expect X0 owned by your user
pgrep -af 'Xwayland|Xorg'                    # note the -auth <path> argument

export DISPLAY=:0
export XAUTHORITY=$(pgrep -af Xwayland | grep -oE '/run/user/[0-9]+/\.mutter-Xwaylandauth\.[^ ]+' | head -1)
```

The `XAUTHORITY` path is regenerated each login session, so re-run the export
line after a reboot or logout.

**B) Forward the GUI to your SSH client** (only works if the client runs an
X server — XQuartz on macOS, VcXsrv/WSLg on Windows, a Linux desktop, etc.):

```bash
ssh -X hailo@<host>      # or -Y for trusted forwarding
# $DISPLAY is now set automatically; skip the exports above
```

Don't mix A and B — pick one per shell. Once either is set, `echo $DISPLAY`
should print a non-empty value before you run any GUI command below.

### Ogre / rviz2 config

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
source /opt/ros/jazzy/setup.bash
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
