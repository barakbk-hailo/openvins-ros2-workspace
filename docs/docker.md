# Docker Installation (RPi5 / Debian Trixie)

For Raspberry Pi 5 running Debian 13 (Trixie), the easiest path is Docker with
the pre-built ROS 2 Humble image. A `Dockerfile_ros2_humble_jammy` is included in
the fork.

## 1. Install Docker on RPi5

```bash
curl -fsSL https://get.docker.com | sh
sudo usermod -aG docker $USER   # re-login after this
```

## 2. Build the image

```bash
cd ~/workspace
git clone git@github.com:barakbk-hailo/open_vins.git
cd open_vins
docker build -t openvins-humble -f Dockerfile_ros2_humble_jammy .
```

## 3. Run the container

```bash
docker run -it --rm \
  --network host --privileged \
  -v /dev:/dev \
  -v ~/workspace:/workspace \
  -v ~/datasets:/datasets \
  openvins-humble
```

Inside the container, ROS 2 and the workspace are already sourced.

## 4. Download datasets inside the container

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
