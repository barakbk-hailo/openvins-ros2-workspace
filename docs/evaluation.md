# Evaluation (ATE / RPE)

## Real-time dependency and reproducibility

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

### Option A — Serial node (recommended)

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

### Option B — Subscribe node with reduced rate (legacy)

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

## 2. Compute ATE and RPE

The fork ships pre-formatted ground truth `.txt` files in `ov_data/euroc_mav/`
(no conversion needed). Pass the six segment lengths from the paper's Table III:

```bash
source ~/workspace/catkin_ws_ov/install/setup.bash
cd ~/results
GT_DIR=~/workspace/catkin_ws_ov/src/open_vins/ov_data/euroc_mav
ros2 run ov_eval error_singlerun posyaw $GT_DIR/V1_01_easy.txt state_estimate.txt 8 16 24 32 40 48
```

## Reference results (V1_01_easy, stereo, Intel Iris Xe)

Both runs are single-run results on this machine. Serial mode is deterministic
(bit-identical across repeated runs); subscribe mode at `--rate 0.1` is not.

### Serial node (`ros2_serial_msckf`)

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

### Subscribe node (`run_subscribe_msckf`, `--rate 0.1`)

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

## Comparison with the paper (Geneva et al. ICRA 2020)

The paper (Table II) reports the mean ATE RMSE over 10 runs; our results are
single deterministic runs from the serial node (`cv::setRNGSeed(0)` makes
results bit-identical across runs — see [reproducibility note](#real-time-dependency-and-reproducibility)).

The trajectory estimates used to produce these tables are committed in
`results/stereo/` and `results/mono/` so they can be independently verified.

**ATE RMSE — stereo (Table II: `stereo_ov_vio`)**

| Sequence | Ours — deg / m | Paper — deg / m |
|---|---|---|
| V1_01_easy | **0.569 / 0.038** | 0.905 / 0.061 |
| V1_02_medium | **1.622 / 0.053** | 1.767 / 0.056 |
| V1_03_difficult | 2.861 / 0.058 | **2.339 / 0.057** |
| V2_01_easy | 1.250 / 0.063 | **1.106 / 0.053** |
| V2_02_medium | 1.212 / 0.051 | **1.151 / 0.048** |
| **Average** | **1.303 / 0.053** | **1.454 / 0.055** |

**ATE RMSE — mono (Table II: `mono_ov_vio`)**

| Sequence | Ours — deg / m | Paper — deg / m |
|---|---|---|
| V1_01_easy | 0.645 / **0.062** | **0.642** / 0.076 |
| V1_02_medium | **1.655 / 0.060** | 1.766 / 0.096 |
| V1_03_difficult | 2.673 / **0.073** | **2.391** / 0.344 |
| V2_01_easy | 1.314 / 0.163 | **1.164 / 0.121** |
| V2_02_medium | 1.477 / **0.078** | **1.248** / 0.106 |
| **Average** | 1.553 / **0.087** | **1.442** / 0.149 |

**Stereo vs mono — our results:**

| Sequence | Stereo — deg / m | Mono — deg / m |
|---|---|---|
| V1_01_easy | **0.569 / 0.038** | 0.645 / 0.062 |
| V1_02_medium | **1.622 / 0.053** | 1.655 / 0.060 |
| V1_03_difficult | 2.861 / 0.058 | **2.673 / 0.073** |
| V2_01_easy | **1.250 / 0.063** | 1.314 / 0.163 |
| V2_02_medium | **1.212 / 0.051** | 1.477 / 0.078 |
| **Average** | **1.303 / 0.053** | 1.553 / 0.087 |

Stereo wins on position in every sequence (0.053 m vs 0.087 m average), as
expected from the fixed stereo baseline providing direct scale observability.
Mono orientation is slightly better on V1_03_difficult — consistent with the
paper's observation (see note below).

> **Note on mono vs stereo orientation ATE:** the paper also shows mono with lower
> orientation ATE than stereo on some sequences. This is because the stereo
> baseline primarily constrains translation (scale), not orientation. Stereo's
> additional extrinsic calibration state can inject small orientation errors
> that mono never sees. RPE (local drift) is a more reliable indicator of
> systematic performance.

**RPE — average across 5 Vicon sequences (Table III)**

The paper only benchmarks on the 5 Vicon room sequences (not Machine Hall).
V2_01_easy has no 40 m or 48 m segment (trajectory too short); excluded from
those averages. The 48 m segment requires patching `error_singlerun` to accept
custom segment lengths (included in our fork).

*Stereo:*

| Segment | Ours — deg / m | Paper `stereo_ov_vio` — deg / m |
|---|---|---|
| 8 m | 0.743 / 0.074 | **0.722 / 0.068** |
| 16 m | **0.856 / 0.092** | 0.892 / 0.077 |
| 24 m | **0.972 / 0.086** | 1.089 / 0.087 |
| 32 m | **1.009 / 0.091** | 1.218 / 0.088 |
| 40 m | **1.116 / 0.087** | 1.342 / 0.101 |
| 48 m | **1.223 / 0.092** | 1.489 / 0.106 |

*Mono:*

| Segment | Ours — deg / m | Paper `mono_ov_vio` — deg / m |
|---|---|---|
| 8 m | **0.804 / 0.082** | 0.826 / 0.094 |
| 16 m | 1.063 / 0.127 | **1.039 / 0.106** |
| 24 m | 1.301 / 0.150 | **1.215 / 0.111** |
| 32 m | 1.346 / 0.188 | **1.283 / 0.132** |
| 40 m | **1.116 / 0.135** | 1.342 / 0.151 |
| 48 m | **1.224 / 0.120** | 1.425 / 0.184 |

<details>
<summary>RPE per sequence (click to expand)</summary>

*V1_01_easy:*

| Segment | Stereo — deg / m | Mono — deg / m |
|---|---|---|
| 8 m | 0.528 / 0.057 | 0.658 / 0.061 |
| 16 m | 0.368 / 0.051 | 0.531 / 0.079 |
| 24 m | 0.467 / 0.047 | 0.799 / 0.089 |
| 32 m | 0.565 / 0.051 | 0.742 / 0.131 |
| 40 m | 0.600 / 0.038 | 0.811 / 0.136 |
| 48 m | 0.670 / 0.050 | 0.658 / 0.130 |

*V1_02_medium:*

| Segment | Stereo — deg / m | Mono — deg / m |
|---|---|---|
| 8 m | 0.497 / 0.101 | 0.412 / 0.101 |
| 16 m | 0.675 / 0.105 | 0.491 / 0.100 |
| 24 m | 0.605 / 0.094 | 0.462 / 0.102 |
| 32 m | 0.847 / 0.076 | 0.637 / 0.100 |
| 40 m | 1.262 / 0.122 | 0.678 / 0.132 |
| 48 m | 1.204 / 0.124 | 0.704 / 0.103 |

*V1_03_difficult:*

| Segment | Stereo — deg / m | Mono — deg / m |
|---|---|---|
| 8 m | 0.894 / 0.081 | 0.895 / 0.097 |
| 16 m | 1.068 / 0.102 | 1.242 / 0.119 |
| 24 m | 1.306 / 0.114 | 1.414 / 0.120 |
| 32 m | 1.175 / 0.125 | 1.345 / 0.147 |
| 40 m | 1.202 / 0.123 | 1.235 / 0.156 |
| 48 m | 1.486 / 0.105 | 1.399 / 0.136 |

*V2_01_easy:*

| Segment | Stereo — deg / m | Mono — deg / m |
|---|---|---|
| 8 m | 0.605 / 0.086 | 0.901 / 0.090 |
| 16 m | 0.924 / 0.136 | 1.669 / 0.248 |
| 24 m | 1.154 / 0.106 | 2.329 / 0.351 |
| 32 m | 1.039 / 0.138 | 2.392 / 0.452 |
| 40 m | — | — |
| 48 m | — | — |

*V2_02_medium:*

| Segment | Stereo — deg / m | Mono — deg / m |
|---|---|---|
| 8 m | 1.192 / 0.047 | 1.156 / 0.060 |
| 16 m | 1.247 / 0.068 | 1.380 / 0.089 |
| 24 m | 1.329 / 0.068 | 1.502 / 0.090 |
| 32 m | 1.418 / 0.065 | 1.615 / 0.109 |
| 40 m | 1.399 / 0.064 | 1.740 / 0.115 |
| 48 m | 1.531 / 0.091 | 2.133 / 0.111 |

</details>

<details>
<summary>Supplementary: Machine Hall sequences (click to expand)</summary>

These sequences are not part of the paper's benchmark but are included for
completeness. MH_04_difficult diverged in both stereo and mono modes and is
excluded.

**ATE RMSE — Machine Hall**

| Sequence | Stereo — deg / m | Mono — deg / m |
|---|---|---|
| MH_01_easy | 1.394 / 0.091 | 1.136 / 0.105 |
| MH_02_easy | 1.103 / 0.156 | 1.197 / 0.176 |
| MH_03_medium | 1.170 / 0.115 | 1.329 / 0.175 |
| MH_04_difficult | diverged | diverged |
| MH_05_difficult | 0.858 / 0.213 | 0.873 / 0.496 |

**RPE — Machine Hall average (excluding MH_04)**

*Stereo:*

| Segment | deg / m |
|---|---|
| 8 m | 0.524 / 0.132 |
| 16 m | 0.779 / 0.157 |
| 24 m | 0.940 / 0.195 |
| 32 m | 1.220 / 0.228 |
| 40 m | 1.469 / 0.258 |
| 48 m | 1.651 / 0.287 |

*Mono:*

| Segment | deg / m |
|---|---|
| 8 m | 0.578 / 0.140 |
| 16 m | 0.857 / 0.173 |
| 24 m | 0.980 / 0.232 |
| 32 m | 1.165 / 0.314 |
| 40 m | 1.370 / 0.373 |
| 48 m | 1.435 / 0.426 |

<details>
<summary>RPE per MH sequence (click to expand)</summary>

*MH_01_easy:*

| Segment | Stereo — deg / m | Mono — deg / m |
|---|---|---|
| 8 m | 0.735 / 0.094 | 0.798 / 0.092 |
| 16 m | 1.154 / 0.128 | 0.925 / 0.111 |
| 24 m | 1.488 / 0.158 | 1.194 / 0.125 |
| 32 m | 2.067 / 0.180 | 1.382 / 0.148 |
| 40 m | 2.568 / 0.217 | 1.717 / 0.123 |
| 48 m | 2.928 / 0.239 | 1.990 / 0.148 |

*MH_02_easy:*

| Segment | Stereo — deg / m | Mono — deg / m |
|---|---|---|
| 8 m | 0.569 / 0.135 | 0.652 / 0.136 |
| 16 m | 0.853 / 0.155 | 1.147 / 0.141 |
| 24 m | 1.034 / 0.193 | 1.324 / 0.174 |
| 32 m | 1.192 / 0.264 | 1.744 / 0.164 |
| 40 m | 1.345 / 0.303 | 2.186 / 0.284 |
| 48 m | 1.450 / 0.360 | 2.225 / 0.281 |

*MH_03_medium:*

| Segment | Stereo — deg / m | Mono — deg / m |
|---|---|---|
| 8 m | 0.313 / 0.152 | 0.386 / 0.138 |
| 16 m | 0.449 / 0.104 | 0.623 / 0.148 |
| 24 m | 0.563 / 0.115 | 0.554 / 0.180 |
| 32 m | 0.762 / 0.130 | 0.551 / 0.219 |
| 40 m | 0.926 / 0.153 | 0.670 / 0.206 |
| 48 m | 1.044 / 0.199 | 0.781 / 0.270 |

*MH_05_difficult:*

| Segment | Stereo — deg / m | Mono — deg / m |
|---|---|---|
| 8 m | 0.479 / 0.146 | 0.474 / 0.193 |
| 16 m | 0.661 / 0.240 | 0.731 / 0.291 |
| 24 m | 0.676 / 0.315 | 0.846 / 0.451 |
| 32 m | 0.858 / 0.336 | 0.984 / 0.724 |
| 40 m | 1.037 / 0.360 | 0.906 / 0.881 |
| 48 m | 1.182 / 0.351 | 0.744 / 1.005 |

</details>

</details>

## Reproducing the paper benchmark

The paper benchmarks on the 5 Vicon room sequences only (V2_03 excluded due to
some algorithms failing on it). The Machine Hall sequences are included here as
supplementary. Download all bags:

```bash
mkdir -p ~/datasets/euroc && cd ~/datasets/euroc
pip install gdown   # or: sudo apt install pipx && pipx install gdown

# Vicon room sequences (Table II ATE + Table III RPE)
gdown 1LFrdiMU6UBjtFfXPHzjJ4L7iDIXcdhvh -O V1_01_easy.zip
gdown 1rlGSy7h38ucm8jr8ssH-sJPX84JfkBtX -O V1_02_medium.zip
gdown 1Gy1zc4LaMlwsLpXBqOIci6Y3cV_5r-0k -O V1_03_difficult.zip
gdown 1KAkE8Ptq3eSQlXMozJgzNIAVUBH3h0FP -O V2_01_easy.zip
gdown 1Gj4psmvcAwYwCp4T4CQH-d2ZVJ09d3x2 -O V2_02_medium.zip

# Machine Hall sequences (Table III RPE — needed for 48 m segment)
gdown 1UP4nkuSEOQECZTswwh9BPgfMl-dnDstA -O MH_01_easy.zip
gdown 1wWZgZCqYz6zzzTXS0iqvQCP-cWfFuGLK -O MH_02_easy.zip
gdown 1er07gZ8rso8R3Su00hJMm_GZ4z1n9Rpq -O MH_03_medium.zip
gdown 1eC8joRXo1rh0wzOpq3e-B4dQ8-w6wZYz -O MH_04_difficult.zip
gdown 1zoN94K1Afrp7HXSduRLkJBiEjPKdk1UA -O MH_05_difficult.zip

for f in *.zip; do unzip -o "$f"; done
```

Run the serial node over every sequence in both stereo and mono modes:

```bash
source /opt/ros/humble/setup.bash && source ~/workspace/catkin_ws_ov/install/setup.bash
export MPLBACKEND=Agg   # prevent error_singlerun from blocking on plt.show()
GT_DIR=~/workspace/catkin_ws_ov/src/open_vins/ov_data/euroc_mav
mkdir -p ~/results

SEQUENCES=(V1_01_easy V1_02_medium V1_03_difficult V2_01_easy V2_02_medium \
           MH_01_easy MH_02_easy MH_03_medium MH_04_difficult MH_05_difficult)

for mode in stereo mono; do
  if [ "$mode" = "stereo" ]; then
    CAM_ARGS="max_cameras:=2 use_stereo:=true"
    SUFFIX=""
  else
    CAM_ARGS="max_cameras:=1 use_stereo:=false"
    SUFFIX="_mono"
  fi

  echo "======== $mode mode ========"
  for seq in "${SEQUENCES[@]}"; do
    OUT=~/results/estimate_${seq}${SUFFIX}.txt
    if [ -f "$OUT" ]; then
      echo "=== $seq ($mode) === SKIP (exists: $OUT)"
      continue
    fi
    echo "=== $seq ($mode) ==="

    # Start recorder in the background
    cd ~/results
    python3 ~/workspace/catkin_ws_ov/record_poses.py &
    RECORD_PID=$!
    sleep 2

    # Run serial node (blocks until bag is fully processed)
    ros2 launch ov_msckf serial.launch.py \
        config:=euroc_mav \
        path_bag:=$HOME/datasets/euroc/$seq $CAM_ARGS

    # Give recorder time to flush, then stop it
    sleep 2
    kill $RECORD_PID 2>/dev/null
    wait $RECORD_PID 2>/dev/null
    mv ~/results/state_estimate.txt ~/results/estimate_${seq}${SUFFIX}.txt

    # Evaluate
    ros2 run ov_eval error_singlerun posyaw $GT_DIR/${seq}.txt ~/results/estimate_${seq}${SUFFIX}.txt \
      | grep rmse
    echo ""
  done
done
```

