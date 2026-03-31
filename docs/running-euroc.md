# Running the EuRoC MAV Example

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

## 2. Terminal 1 — launch OpenVINS

```bash
cd ~/workspace/catkin_ws_ov
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ov_msckf subscribe.launch.py config:=euroc_mav
```

## 3. Terminal 2 — play the bag

```bash
source /opt/ros/humble/setup.bash
cd ~/datasets/euroc
ros2 bag play V1_01_easy
```

## 4. (Optional) Visualize in RViz

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
