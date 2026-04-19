# Timing Benchmarks (Realtime Performance)

| | |
|---|---|
| **Date** | 2026-03-31 |
| **Host** | Dell Latitude 5420, Intel Core i7-1185G7 (4C/8T, 3.0-4.8 GHz), Intel Iris Xe |
| **OS** | Ubuntu 22.04, ROS 2 Humble |
| **Build** | `colcon build --symlink-install` (compiler flags: `-O3 -fsee -fomit-frame-pointer -g3`) |
| **Dataset** | EuRoC MAV — see details below |
| **Reference** | https://docs.openvins.com/eval-timing.html |

### Dataset: EuRoC MAV

The [EuRoC MAV dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)
was collected on an AscTec Firefly hex-rotor MAV in two indoor environments (Vicon
room and machine hall) at ETH Zurich. It is the standard benchmark for visual-inertial
odometry research, used in the original OpenVINS ICRA 2020 paper.

| Sensor | Details |
|--------|---------|
| **Cameras** | 2x MT9V034 global-shutter, grayscale, 752x480 px, 20 Hz, hardware-synced stereo pair (~11 cm baseline) |
| **IMU** | ADIS16448, 6-axis (3 gyro + 3 accel), 200 Hz |
| **Ground truth** | Vicon motion capture at 100 Hz (Vicon room sequences) or Leica MS50 laser tracker (machine hall sequences) |

11 sequences total, split into two environments:

| Environment | Sequences | Difficulty levels |
|-------------|-----------|-------------------|
| Vicon room (V1/V2) | 5 sequences | easy, medium, difficult |
| Machine hall (MH) | 6 sequences | easy (x2), medium, difficult (x2) |

Pre-converted ROS 2 bags are stored in `~/datasets/euroc/` (see
[evaluation](evaluation.md) for download instructions).

## Goal

Understand which parts of the OpenVINS VIO algorithm take how long, what can be
optimized, and whether realtime operation is feasible on Raspberry Pi 5. This is the
x86 baseline — the same scripts will be reused on RPi5 hardware in Phase 4.

## Background: the two runners

OpenVINS provides two ways to run the VIO pipeline:

- **`ros2_serial_msckf`** (our ROS 2 port, in this fork): Reads a ROS 2 bag file
  directly, loads all messages into memory, and processes them one by one with
  `use_multi_threading_subs=false`. The VIO update blocks until complete before
  the next frame is read. This is deterministic (bit-identical across runs) and
  CPU-speed-independent — no frames are ever dropped regardless of how slow the
  machine is. Ideal for measuring pure algorithmic cost.

- **`run_subscribe_msckf`** (upstream): Subscribes to ROS 2 topics via a
  `MultiThreadedExecutor` with `use_multi_threading_subs=true`. The VIO update
  runs on a detached thread while the executor continues receiving messages. If
  VIO can't keep up with the camera rate, messages queue up and eventually get
  dropped. This is what runs on real hardware.

We use serial mode first (clean measurements), then subscribe mode (realtime test).

## Background: the VIO pipeline and timing components

OpenVINS has built-in timing instrumentation in `VioManager::track_image_and_update()`
(`ov_msckf/src/core/VioManager.cpp`). When `record_timing_information: true` is set
in the estimator config YAML, it writes a CSV with one row per processed camera frame.
Each row records 6 component timings (in seconds), measured by wall-clock timestamps
rT1 through rT7:

```
Camera frame arrives
    |
    v
[1. TRACKING]  rT1 -> rT2
    Detect and track visual features across frames using KLT optical flow
    (cv::calcOpticalFlowPyrLK). In stereo mode, this includes matching features
    between left/right cameras. Also runs ArUco marker detection if enabled.
    This is pure computer vision (OpenCV) — the main consumer of CPU cycles.
    |
    v
[2. PROPAGATION]  rT2 -> rT3
    Integrate IMU measurements (gyro + accelerometer) from the last camera time
    to the current one using RK4 integration. Predicts where the camera is now.
    Also "augments" the EKF state with a new clone (snapshot of the pose at this
    timestamp). This is lightweight linear algebra — always fast (<0.3ms).
    |
    v
[3. MSCKF UPDATE]  rT3 -> rT4
    Includes feature classification (sorting lost/marginalized/max-track features
    into MSCKF vs SLAM sets, SLAM landmark marginalization) AND the core MSCKF
    EKF update. The MSCKF update takes features that have been LOST (no longer
    tracked) plus features at the marginalization boundary, and uses their
    multi-view observations to update the EKF state. These features are used once
    and discarded — they never enter the state vector. Cost scales with the number
    of lost features (capped at max_msckf_in_update=40). Involves triangulation,
    Jacobian computation, and the EKF update (dense matrix math).
    Highly variable — near zero on some frames, up to 15ms on feature-loss spikes.
    |
    v
[4. SLAM UPDATE]  rT4 -> rT5
    Updates the EKF using persistent SLAM landmarks that ARE in the state vector
    (up to max_slam=50 features). These are long-lived features tracked across
    many frames. The update is done in sequential batches (max_slam_in_update=25).
    Cost is O(n^2) in total state size (clones x SLAM features) — this is why it
    is often the single most expensive component. Stable per-frame cost since the
    number of SLAM features in the state stays near max_slam.
    |
    v
[5. SLAM DELAYED INIT]  rT5 -> rT6
    Initializes new SLAM landmarks. When a tracked feature is deemed "good enough"
    (long track, sufficient parallax), it gets triangulated and added to the state.
    Cost is spiky — zero on most frames, expensive when many features initialize
    simultaneously (e.g. after entering a new area with many new features).
    |
    v
[6. RE-TRIANGULATION & MARGINALIZATION]  rT6 -> rT7
    Two sub-steps combined:
    - Re-triangulate all active tracks in the current frame (for visualization)
    - Marginalize the oldest clone from the sliding window when it exceeds
      max_clones=11 (removes the clone's rows/columns from the covariance matrix
      and re-indexes remaining variables)
    Also includes feature database cleanup and SLAM anchor changes.
    Relatively stable cost since the sliding window size is fixed.
```

**Total** = wall-clock rT1 to rT7. For realtime operation, this must stay below the
inter-frame interval: **50ms at 20Hz** (EuRoC camera rate), **33ms at 30Hz** (typical
USB camera).

## How timing data was collected

The timing scripts automatically enable this by creating a temporary copy of
`estimator_config.yaml` with `record_timing_information: true` and passing it via the
`config_path:=` launch argument. The default config is not modified — the temp file
is cleaned up after each run.

```yaml
# These settings control timing output (in estimator_config.yaml):
record_timing_information: true    # enable per-frame CSV output
record_timing_filepath: "/tmp/traj_timing.txt"  # where the CSV is written
```

`VioManager` writes the CSV automatically during every run, regardless of which
runner (serial or subscribe) is used. After each run, the scripts copy the CSV to the
results directory.

Results are analyzed with the `ov_eval` package (built as part of the workspace):

```bash
source /opt/ros/humble/setup.bash && source ~/workspace/catkin_ws_ov/install/setup.bash

# Per-run stats: mean, std, p99, max for each component
ros2 run ov_eval timing_flamegraph <file.txt>

# Side-by-side comparison of multiple runs
ros2 run ov_eval timing_comparison <file1.txt> <file2.txt> ...

# Distribution plot of per-frame total time
ros2 run ov_eval timing_histogram <file.txt> <num_bins>
```

## Sequence selection

We chose 3 EuRoC MAV sequences to cover a range of difficulty:

| Sequence | Environment | Difficulty | Why chosen |
|----------|------------|------------|------------|
| V1_01_easy | Vicon room, slow motion | Easy | Baseline — stable tracking, most features, highest SLAM load |
| MH_03_medium | Machine hall, moderate speed | Medium | Larger space, different feature density, moderate motion |
| V1_03_difficult | Vicon room, fast motion, motion blur | Hard | Worst case for tracking — tests how KLT degrades under blur |

V1_01_easy is also the sequence used for our accuracy benchmarks (see
[evaluation](evaluation.md)), so timing and accuracy results are directly comparable.

---

## Phase 1: Serial baseline (pure algorithmic cost)

**What we ran:** `ros2_serial_msckf` on each of the 3 sequences, in both stereo and
mono modes (6 runs total). Serial mode reads the bag directly — no `ros2 bag play`
needed.

**Script:** `run_timing_benchmark.sh`
Loops over sequences × {stereo, mono}, runs `ros2 launch ov_msckf serial.launch.py`
with `max_cameras:=2 use_stereo:=true` (or `1`/`false` for mono), and copies the
timing CSV to the results directory. Skips runs whose output already exists (safe to
re-run).

**Default config:** 200 features (`num_pts`), full resolution, 4 OpenCV threads,
50 SLAM landmarks (`max_slam`), 11 clones in the sliding window.

### Stereo results (mean total time in ms)

| Component | V1_01_easy | MH_03_medium | V1_03_difficult |
|-----------|-----------|-------------|----------------|
| tracking | 2.6 (p99: 4.1, max: 10.1) | 2.7 (p99: 4.1, max: 6.4) | 3.0 (p99: 7.0, max: 14.1) |
| propagation | 0.2 (p99: 0.3) | 0.2 (p99: 0.3) | 0.2 (p99: 0.3) |
| msckf update | 1.6 (p99: 11.0, max: 15.5) | 1.2 (p99: 9.8, max: 17.2) | 1.2 (p99: 7.8, max: 14.9) |
| slam update | 4.5 (p99: 6.3, max: 9.3) | 3.6 (p99: 5.8, max: 7.6) | 2.1 (p99: 5.9, max: 7.3) |
| slam delayed | 0.9 (p99: 8.0, max: 16.5) | 1.3 (p99: 10.7, max: 23.4) | 1.5 (p99: 10.0, max: 14.2) |
| re-tri & marg | 1.5 (p99: 1.9, max: 2.7) | 1.5 (p99: 2.1, max: 2.8) | 1.5 (p99: 1.9, max: 2.5) |
| **total** | **11.3** (p99: 22.7, max: 30.2) | **10.5** (p99: 22.5, max: 30.9) | **9.5** (p99: 20.5, max: 26.0) |

*Source: `results/timing/x86/serial/stereo/{V1_01_easy,MH_03_medium,V1_03_difficult}.txt`*

Frames processed: V1_01=2776, MH_03=2302, V1_03=1990 (out of 2912 in each bag;
the difference is from the initialization period where no timing is recorded, plus
stereo sync misses where no matching pair was found within ±20ms)

### Mono results (mean total time in ms)

| Component | V1_01_easy | MH_03_medium | V1_03_difficult |
|-----------|-----------|-------------|----------------|
| tracking | 1.8 (p99: 2.7, max: 6.4) | 1.8 (p99: 2.9, max: 4.2) | 2.2 (p99: 7.6, max: 19.5) |
| propagation | 0.2 (p99: 0.3) | 0.2 (p99: 0.3) | 0.2 (p99: 0.2) |
| msckf update | 2.0 (p99: 6.2, max: 8.5) | 1.6 (p99: 5.5, max: 8.1) | 1.2 (p99: 5.2, max: 7.9) |
| slam update | 2.6 (p99: 3.5, max: 7.9) | 2.3 (p99: 3.2, max: 5.6) | 1.3 (p99: 3.0, max: 6.6) |
| slam delayed | 0.6 (p99: 3.4, max: 6.7) | 0.8 (p99: 4.2, max: 9.8) | 1.1 (p99: 5.7, max: 13.8) |
| re-tri & marg | 1.0 (p99: 1.5, max: 4.4) | 1.0 (p99: 1.5, max: 2.0) | 0.9 (p99: 1.3, max: 3.8) |
| **total** | **8.2** (p99: 13.1, max: 16.8) | **7.7** (p99: 12.8, max: 16.0) | **6.9** (p99: 13.6, max: 24.0) |

*Source: `results/timing/x86/serial/mono/{V1_01_easy,MH_03_medium,V1_03_difficult}.txt`*

Frames processed: V1_01=2799, MH_03=2310, V1_03=2004

### Phase 1 findings

1. **Comfortably within realtime on x86.** Stereo p99 is ~23ms vs 50ms budget at
   20Hz — roughly 2x headroom.

2. **SLAM update is the dominant component** on easy/medium sequences (up to 4.5ms
   mean, 40% of total). This was surprising — one might expect tracking (the OpenCV
   part) to dominate, but the EKF update with 50 persistent SLAM features is more
   expensive.

3. **Tracking scales with difficulty.** On V1_03_difficult (fast motion, blur),
   tracking mean increases from 2.6ms to 3.0ms and p99 from 4.1ms to 7.0ms. KLT
   has to work harder to track through motion blur.

4. **MSCKF update and SLAM delayed are spiky.** These components have high variance
   (std ~ mean) because their cost depends on how many features are lost or
   initialized on each frame. The p99 and max values are 3-10x the mean.

5. **Stereo is ~38% slower than mono** (11.3ms vs 8.2ms on V1_01). The gap:
   - Tracking: +0.8ms (stereo matching between left/right cameras)
   - SLAM update: +1.9ms (larger state vector — stereo adds extrinsic calibration)
   - Re-tri & marg: +0.5ms (more features to re-triangulate)

6. **Paradox: difficult sequences are faster.** V1_03_difficult (9.5ms) is faster
   than V1_01_easy (11.3ms) because fewer features survive motion blur, so SLAM
   update and MSCKF update have less work. The tracking component gets more
   expensive, but the overall pipeline gets lighter.

---

## Phase 2: Config sensitivity sweeps

**What we ran:** 5 config variants on V1_01_easy (stereo, serial mode) to find which
knobs matter most for RPi5 optimization.

**Script:** `run_timing_sweep.sh`
For each variant, copies `estimator_config.yaml` to a temp file in the **same
directory** (important — the YAML uses `relative_config_imu` and
`relative_config_imucam` which are resolved relative to the config file's path),
applies sed substitutions, and runs serial mode with `config_path:=<temp_file>`.
Cleans up the temp file after all runs.

**Why these variants:**

| Variant | Config change | Rationale |
|---------|--------------|-----------|
| A: Downsample | `downsample_cameras: true` | Halves image resolution -> directly reduces KLT cost. Cheapest accuracy tradeoff since features are still detected, just at lower resolution. |
| B: 100 features | `num_pts: 100` (from 200) | Fewer features means less work everywhere — tracking, triangulation, and EKF updates all scale with feature count. |
| C: 300 features | `num_pts: 300` (from 200) | Upper bound test — shows the cost of more features if accuracy demands it. |
| D: No SLAM | `max_slam: 0, max_slam_in_update: 0` | Since SLAM update is the dominant component (Phase 1 finding), what happens if we eliminate it entirely? Features that would become SLAM landmarks go through MSCKF instead. |
| E: 1 OpenCV thread | `num_opencv_threads: 1` (from 4) | RPi5 has 4 cores with no hyperthreading. How much does OpenCV parallelism actually help? Should we save those cores for ROS2 instead? |

### Results (all times in ms, V1_01_easy stereo)

| Variant | Tracking | Propagation | MSCKF upd | SLAM upd | SLAM delay | Re-tri/marg | **Total** | **p99** | **max** |
|---------|----------|-------------|-----------|----------|------------|-------------|-----------|---------|---------|
| **Baseline** (200pts, full-res, 4 thr) | 2.6 | 0.2 | 1.6 | 4.5 | 0.9 | 1.5 | **11.3** | **19.2** | **30.2** |
| **A: Downsample** | 1.7 | 0.2 | 1.5 | 4.6 | 0.9 | 0.6 | **9.5** | **14.9** | **23.8** |
| **B: 100 features** | 1.8 | 0.2 | 0.1 | 2.9 | 0.5 | 1.3 | **6.8** | **12.7** | **32.5** |
| **C: 300 features** | 3.3 | 0.2 | 3.5 | 4.6 | 1.0 | 1.8 | **14.3** | **22.6** | **33.4** |
| **D: No SLAM** | 2.5 | 0.1 | 3.2 | — | — | 1.5 | **7.4** | **14.6** | **20.7** |
| **E: 1 OpenCV thread** | 3.7 | 0.2 | 1.6 | 4.5 | 0.9 | 1.4 | **12.3** | **20.5** | **34.7** |

*Source: `results/timing/x86/serial/sweep/{A_downsample,B_num_pts_100,C_num_pts_300,D_no_slam,E_opencv_1thread}.txt`; baseline row from `results/timing/x86/serial/stereo/V1_01_easy.txt`*

### Phase 2 findings — impact ranking

1. **100 features** (-4.5ms, **-40%**): Biggest single win. MSCKF update nearly
   vanishes (1.6ms -> 0.1ms — fewer features are lost per frame so fewer MSCKF
   updates happen). SLAM update drops from 4.5ms to 2.9ms (fewer features in the
   state). Tracking drops slightly (less extraction work).
   *Tradeoff:* accuracy may degrade on difficult sequences with fewer visual cues.

2. **No SLAM** (-3.9ms, **-35%**): Eliminates SLAM update (4.5ms) and SLAM delayed
   (0.9ms) entirely. But MSCKF update doubles from 1.6ms to 3.2ms — features that
   would have become persistent SLAM landmarks now go through the one-shot MSCKF
   path instead, each requiring triangulation.
   *Tradeoff:* no persistent landmarks means worse long-term drift, especially in
   revisited areas.

3. **Downsample** (-1.8ms, **-16%**): Tracking drops 35% (2.6ms -> 1.7ms) — KLT
   on quarter-pixel images is much cheaper. Re-tri & marg also drops (1.5ms -> 0.6ms).
   Other components barely change since the number of features is the same.
   *Tradeoff:* minimal — features are still detected, just at lower resolution.
   This is likely the cheapest accuracy tradeoff.

4. **1 OpenCV thread** (+1.0ms, **+9%**): Only tracking is affected (2.6ms -> 3.7ms,
   +42%). SLAM/MSCKF updates don't use OpenCV threading at all.
   *Key insight:* OpenCV parallelism gives only moderate benefit for this workload.
   On RPi5, dedicating cores to the ROS2 executor may be more valuable than giving
   them to OpenCV.

5. **300 features** (+3.0ms, **+27%**): Diminishing returns. 50% more features
   costs 27% more total time. MSCKF update more than doubles (+119%) since more
   features are lost per frame. Tracking grows +27%.

### Key insight: SLAM update is the bottleneck

At 4.5ms mean (40% of total), SLAM update is the single largest component. It is
O(n^2) in total state size (sliding window clones x SLAM features). Three paths to
reduce it:

1. Disable SLAM entirely (`max_slam: 0`) — simplest, saves 35%
2. Reduce `max_slam` (e.g. 25 instead of 50) — partial savings, keeps some landmarks
3. Reduce `max_slam_in_update` (batch size) — spreads cost across frames

### Key insight: OpenCV threading is not the bottleneck

Going from 4 threads to 1 thread costs only +1ms (+9%). This means:
- KLT tracking is not heavily parallelized in OpenCV for 200 features at 752x480
- RPi5's 4 Cortex-A76 cores are better used for the ROS2 executor + VIO thread
- Don't over-optimize `num_opencv_threads` — the savings are elsewhere

---

## Phase 3: Subscribe mode realtime feasibility

**What we ran:** `run_subscribe_msckf` with bag playback at 1x, 2x, and 5x speed.
This tests whether the system can keep up with realtime sensor data under ROS 2
middleware overhead.

**Script:** `run_timing_subscribe.sh [rate]`
Launches `ros2 launch ov_msckf subscribe.launch.py` in the background, waits 3
seconds for the node to start, then runs `ros2 bag play <bag> --rate <rate>` which
blocks until the bag finishes. After playback, waits 5 seconds for OpenVINS to drain
its message queue, then kills the process and copies the timing CSV. Reports
processed frame count vs expected (2912) to quantify drops.

**Why different rates:** 1x tests normal realtime. Higher rates stress-test to find
the breaking point — the playback rate at which VIO falls behind and starts dropping
frames.

### Frame drop analysis

| Playback rate | Processed frames | "Dropped" | Drop % |
|---------------|-----------------|-----------|--------|
| 1.0x (realtime) | 2800 | 112 | 3.8% |
| 2.0x | 2800 | 112 | 3.8% |
| 5.0x | 2799 | 113 | 3.9% |
| Serial (reference) | 2776 | 136 | 4.7% |

*Source: `results/timing/x86/subscribe/V1_01_easy_rate{1.0,2.0,5.0}.txt`; serial reference from `results/timing/x86/serial/stereo/V1_01_easy.txt`*

The ~112-136 "missing" frames are **NOT performance-related drops**. They come from:
1. The initialization period (first ~2s before VIO converges, no timing is recorded)
2. Stereo sync misses (serial uses a strict +/-20ms window; subscribe uses ROS2's
   `ApproximateTime` policy which is slightly more permissive)

Subscribe actually processes MORE frames than serial (2800 vs 2776) because of this
sync policy difference (also noted in [evaluation](evaluation.md) when comparing
accuracy results).

The drop count being identical at 1x, 2x, and 5x proves these are not
performance-related. **x86 has zero real frame drops even at 5x realtime.**

### Per-component timing: subscribe vs serial

| Component | Serial (ms) | Subscribe 1x (ms) | Subscribe 2x (ms) | Subscribe 5x (ms) |
|-----------|------------|-------------------|-------------------|-------------------|
| tracking | 2.6 | **7.1** | 4.0 | 2.9 |
| propagation | 0.2 | 0.4 | 0.2 | 0.1 |
| msckf update | 1.6 | 3.1 | 1.8 | 0.2 |
| slam update | 4.5 | **7.4** | 4.9 | 0.3 |
| slam delayed | 0.9 | 1.2 | 1.3 | 0.5 |
| re-tri & marg | 1.5 | 1.8 | 1.7 | 1.6 |
| **total** | **11.3** | **20.9** | **13.9** | **5.7** |

*Source: `results/timing/x86/subscribe/V1_01_easy_rate{1.0,2.0,5.0}.txt`; serial column from `results/timing/x86/serial/stereo/V1_01_easy.txt`*

### Phase 3 findings

1. **Subscribe 1x is 1.85x slower than serial** (20.9ms vs 11.3ms). The overhead
   comes from the ROS2 `MultiThreadedExecutor`, message deserialization through the
   subscriber pipeline, and CPU contention between the VIO update thread and the
   executor's callback threads. This is real overhead that exists on any ROS2
   deployment — it's the cost of the middleware.

2. **Subscribe 2x is faster than 1x** (13.9ms vs 20.9ms). This is counterintuitive
   but makes sense: at 1x rate, the executor has idle time between callbacks where
   CPU caches cool and threads context-switch more. At 2x, messages arrive
   back-to-back, keeping the CPU pipeline hot and reducing scheduling overhead.

3. **Subscribe 5x shows artificially low times** (5.7ms). At 5x, VIO can't process
   frames as fast as they arrive, so it runs with a lighter state (fewer tracked
   features, fewer SLAM landmarks converge). The SLAM and MSCKF updates shrink
   accordingly. This is NOT a valid measure of algorithmic cost — it's an artifact
   of VIO running in a degraded mode.

4. **Tracking suffers most from contention** — 2.7x slower in subscribe 1x vs
   serial (7.1ms vs 2.6ms). KLT optical flow is compute-intensive and sensitive to
   cache pressure from concurrent threads. The EKF updates (which are memory-bound
   matrix operations) also slow down but by a smaller factor.

---

## Phase 3b: Paper reproduction and subscribe-mode reliability

Phase 3b reproduced the OpenVINS laptop results from Semenova et al. (2024)
on V2_02_medium using 3-clock timing (wall, process CPU, thread CPU) and
identified a SLAM feature collapse failure mode in subscribe mode. This led
to a SLAM recovery mechanism that prevents the irrecoverable empty-state
feedback loop.

The full analysis is split across two dedicated documents:

- **[Benchmark Analysis](benchmark-analysis.md)** — paper comparison (30 runs),
  3-clock timing breakdown, accuracy and consistency results, RPi5 projections
- **[Subscribe Reliability](subscribe-reliability.md)** — root cause analysis
  of subscribe-mode non-determinism, SLAM recovery mechanism, fork changes

---

## RPi5 projections — methodology and caveats

The RPi5 estimates below are **projections, not measurements**. They use two scaling
factors applied to the x86 measurements. Phase 4 (actual RPi5 measurements) will
replace these projections — treat them as order-of-magnitude guidance.

### Factor 1: CPU speed ratio (3.5x slower)

Our x86 host is an Intel i7-1185G7 (Tiger Lake, 4C/8T, up to 4.8 GHz single-core
boost, 12MB L3 cache). The RPi5 has a Broadcom BCM2712 with 4x Cortex-A76 at
2.4 GHz, 2MB shared L2 cache. The 3.5x factor combines:

| Dimension | Ratio | Notes |
|-----------|-------|-------|
| Clock frequency | ~1.5x | i7 sustains ~3.5 GHz avg under load vs RPi5's 2.4 GHz |
| Instructions per clock (IPC) | ~1.3x | Tiger Lake microarchitecture vs Cortex-A76 for scalar workloads |
| SIMD width | ~2x | AVX2 (256-bit) vs NEON (128-bit) — affects OpenCV KLT heavily |
| Cache hierarchy | ~1.5x | 12MB L3 vs 2MB L2 — affects large matrix operations in EKF |
| **Combined estimate** | **~3-4x** | We use 3.5x as the middle of the range |

The SIMD factor may be smaller if OpenCV on ARM has well-optimized NEON intrinsics
for KLT. The cache factor may be larger for SLAM update (large dense matrices). We
won't know until we measure.

### Factor 2: Subscribe-mode overhead (1.85x)

From Phase 3, subscribe mode is 1.85x slower than serial on x86 (20.9ms vs 11.3ms).
On RPi5 this factor could differ:

- RPi5 has 4 physical cores (no hyperthreading) vs x86's 4C/8T — thread contention
  may be worse since there are fewer hardware threads available
- But RPi5 at lower throughput means less queue pressure — contention could also be
  less severe
- We assume the same 1.85x, knowing it could range from 1.5x to 2.5x

### Combined projections

RPi5 serial ~ x86 serial x 3.5
RPi5 subscribe ~ x86 serial x 3.5 x 1.85

| Scenario | x86 serial (measured) | RPi5 serial (x3.5) | RPi5 subscribe (x1.85) | Budget (20Hz) | Verdict |
|----------|----------------------|-------------------|----------------------|---------------|---------|
| Stereo baseline | 11.3ms | ~40ms | ~74ms | 50ms | **too slow** |
| Mono baseline | 8.2ms | ~29ms | ~54ms | 50ms | **borderline** |
| Stereo + 100 pts | 6.8ms | ~24ms | ~44ms | 50ms | OK |
| Downsample stereo | 9.5ms | ~33ms | ~61ms | 50ms | subscribe too slow |
| No SLAM stereo | 7.4ms | ~26ms | ~48ms | 50ms | OK |
| Mono + downsample + 100pts | ~4ms (est) | ~14ms | ~26ms | 50ms | **comfortable** |

At 30Hz (33ms budget — relevant for live USB cameras on RPi5), only the most
aggressive configs (mono + downsample + reduced features) have headroom.

### Thermal throttling risk

RPi5's Cortex-A76 cores throttle from 2.4 GHz to ~1.8 GHz under sustained load
(passive cooling) or ~1.5 GHz (fanless case). This would add another 1.3-1.6x
factor on top of the projections. Monitor via:
```bash
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq
```

---

## Summary

### Where the time goes (stereo, V1_01_easy, serial mode)

```
Total: 11.3ms per frame
  ┌─────────────────────────────────────────┐
  │  SLAM update         4.5ms  (40%)  ████████████████████
  │  Tracking            2.6ms  (23%)  ████████████
  │  MSCKF update        1.6ms  (14%)  ████████
  │  Re-tri & marg       1.5ms  (13%)  ███████
  │  SLAM delayed        0.9ms  ( 8%)  ████
  │  Propagation         0.2ms  ( 2%)  █
  └─────────────────────────────────────────┘
```

### What helps most (stereo, V1_01_easy)

| Optimization | Savings | Notes |
|-------------|---------|-------|
| Reduce features to 100 | -40% | Biggest win. Impacts tracking + all updates. |
| Disable SLAM (max_slam=0) | -35% | Eliminates largest component. MSCKF takes over. |
| Switch to mono | -27% | Removes stereo matching + smaller state. |
| Downsample images | -16% | Cheap win. Tracking + re-tri savings. |
| Reduce OpenCV threads | +9% | Modest. Not worth worrying about. |
| Increase features to 300 | +27% | Diminishing returns. Avoid. |

*Source: derived from `results/timing/x86/serial/{stereo/V1_01_easy.txt,mono/V1_01_easy.txt,sweep/*.txt}`*

### Recommended RPi5 starting configs (to be validated in Phase 4)

1. **Conservative:** Mono, downsample, 100 features, no SLAM -> est. ~26ms subscribe
2. **Balanced:** Mono, downsample, 150 features, max_slam=25 -> est. ~35ms subscribe
3. **Aggressive:** Stereo, 100 features -> est. ~44ms subscribe (tight but possible)

---

## Phase 4: RPi5 measurements (actual hardware)

| | |
|---|---|
| **Date** | 2026-04-12 |
| **Host** | Raspberry Pi 5 (BCM2712, 4x Cortex-A76 @ 2.4 GHz, 8 GB RAM) |
| **OS** | Debian 13 (Trixie), Docker container running Ubuntu 22.04 / ROS 2 Humble |
| **Build** | `colcon build --symlink-install` inside Docker (same source as x86) |
| **Dataset** | EuRoC MAV — same 3 sequences as x86 baseline |
| **Config** | Default: 200 features, full resolution, 4 OpenCV threads, 50 SLAM landmarks |

### Stereo timing results (mean total time in ms)

| Component | V1_01_easy | MH_03_medium | V1_03_difficult |
|-----------|-----------|-------------|----------------|
| tracking | 6.9 (p99: 10.4, max: 25.4) | 6.6 (p99: 9.6, max: 24.2) | 8.0 (p99: 13.6, max: 29.2) |
| propagation | 0.4 (p99: 0.8) | 0.4 (p99: 0.6) | 0.4 (p99: 0.6) |
| msckf update | 2.8 (p99: 13.0, max: 41.0) | 2.0 (p99: 9.9, max: 29.2) | 2.0 (p99: 8.0, max: 27.5) |
| slam update | 7.3 (p99: 11.4, max: 17.7) | 5.9 (p99: 10.9, max: 20.3) | 3.4 (p99: 9.0, max: 15.2) |
| slam delayed | 1.6 (p99: 8.5, max: 30.6) | 2.2 (p99: 11.0, max: 37.7) | 2.6 (p99: 10.9, max: 23.6) |
| re-tri & marg | 5.1 (p99: 6.5, max: 20.3) | 4.9 (p99: 6.1, max: 12.4) | 5.2 (p99: 6.5, max: 11.2) |
| **total** | **24.2** (p99: 38.8, max: 64.5) | **22.0** (p99: 36.4, max: 59.3) | **21.6** (p99: 35.4, max: 61.4) |

*Source: `results/timing/rpi5/serial/stereo/{V1_01_easy,MH_03_medium_bagstart5,V1_03_difficult}.txt`*

Frames processed: V1_01=2776, MH_03=2302, V1_03=1990 (same as x86)

**Note on MH_03:** The default run (`bag_start=0`) causes the stereo filter to
diverge on RPi5 (see [accuracy section](#accuracy-rpi5-vs-x86) below). The timing
numbers above are from `bag_start:=5.0` which produces a healthy filter. The
diverged run (`MH_03_medium.txt` in the results directory) shows artificially low
times (14.5ms total) because the filter runs idle with near-zero SLAM/MSCKF updates.

### Mono timing results (mean total time in ms)

| Component | V1_01_easy | MH_03_medium | V1_03_difficult |
|-----------|-----------|-------------|----------------|
| tracking | 4.7 (p99: 7.1, max: 16.7) | 4.6 (p99: 7.1, max: 20.1) | 5.5 (p99: 10.9, max: 30.8) |
| propagation | 0.5 (p99: 0.6) | 0.4 (p99: 0.6) | 0.4 (p99: 0.6) |
| msckf update | 3.3 (p99: 9.3, max: 16.4) | 2.6 (p99: 8.0, max: 14.0) | 2.0 (p99: 6.7, max: 13.0) |
| slam update | 4.2 (p99: 6.2, max: 14.8) | 3.5 (p99: 6.2, max: 17.4) | 2.2 (p99: 5.5, max: 8.3) |
| slam delayed | 1.1 (p99: 4.7, max: 11.2) | 1.6 (p99: 6.0, max: 14.1) | 2.0 (p99: 7.2, max: 27.1) |
| re-tri & marg | 3.2 (p99: 4.1, max: 10.1) | 3.1 (p99: 4.0, max: 8.7) | 2.9 (p99: 4.1, max: 20.8) |
| **total** | **17.1** (p99: 25.3, max: 51.8) | **15.8** (p99: 24.9, max: 51.0) | **15.0** (p99: 25.4, max: 44.5) |

*Source: `results/timing/rpi5/serial/mono/{V1_01_easy,MH_03_medium,V1_03_difficult}.txt`*

Frames processed: V1_01=2799, MH_03=2310, V1_03=2004

### Phase 4 timing findings: x86 vs RPi5

#### Overall slowdown

| Mode | Sequence | x86 (ms) | RPi5 (ms) | Ratio |
|------|----------|---------|----------|-------|
| Stereo | V1_01_easy | 11.3 | 24.2 | 2.1x |
| Stereo | MH_03_medium | 10.5 | 22.0 | 2.1x |
| Stereo | V1_03_difficult | 9.5 | 21.6 | 2.3x |
| Mono | V1_01_easy | 8.2 | 17.1 | 2.1x |
| Mono | MH_03_medium | 7.7 | 15.8 | 2.1x |
| Mono | V1_03_difficult | 6.9 | 15.0 | 2.2x |

*Source: derived from `results/timing/{x86,rpi5}/serial/{stereo,mono}/*.txt`*

**Actual slowdown: 2.1-2.3x** — significantly better than the 3.5x projection from
Phase 3. The Cortex-A76 cores are more capable than estimated, especially for the
matrix-heavy EKF updates.

#### Per-component slowdown (RPi5 / x86, stereo, averaged across sequences)

| Component | Slowdown | Likely cause |
|-----------|----------|-------------|
| tracking | 2.6x | NEON (128-bit) vs AVX2 (256-bit) for KLT optical flow |
| propagation | 2.0x | Lightweight scalar math, stable across platforms |
| msckf update | 1.7x | Dense matrix ops, better than expected |
| slam update | 1.6x | Dense matrix ops, Eigen NEON is well-optimized |
| slam delayed | 1.8x | Triangulation + state augmentation |
| re-tri & marg | **3.4x** | Cache-bound: 2MB L2 (RPi5) vs 12MB L3 (x86) |

*Source: derived from `results/timing/{x86,rpi5}/serial/stereo/*.txt` (averaged across sequences)*

The re-triangulation & marginalization component scales worst (3.4x). This step
iterates over all active features and the full covariance matrix, making it sensitive
to cache size. The SLAM and MSCKF EKF updates (dense matrix math) scale best at
1.6-1.7x — Eigen's NEON backend is effective for these operations.

#### Realtime feasibility (serial mode)

| Scenario | RPi5 mean (ms) | RPi5 p99 (ms) | Budget 20Hz (50ms) | Budget 30Hz (33ms) |
|----------|---------------|---------------|--------------------|--------------------|
| Stereo baseline | 24.2 | 38.8 | OK | p99 exceeds |
| Mono baseline | 17.1 | 25.3 | OK | OK |

*Source: `results/timing/rpi5/serial/{stereo,mono}/V1_01_easy.txt`*

Serial mode is comfortably within the 20Hz budget. Mono at 30Hz has ~8ms headroom
at p99. Subscribe mode (not yet measured on RPi5) will add ROS2 middleware overhead —
on x86 this was 1.85x, which would put stereo at ~45ms mean and ~72ms p99 (above
budget). Subscribe mode measurements are needed to confirm.

### Accuracy: RPi5 vs x86 {#accuracy-rpi5-vs-x86}

All runs are stereo, serial mode, default config (200 features, max_slam=50).

#### Absolute Trajectory Error

| Sequence | x86 rmse_ori (deg) | RPi5 rmse_ori (deg) | x86 rmse_pos (m) | RPi5 rmse_pos (m) |
|----------|--------------------|---------------------|-------------------|-------------------|
| V1_01_easy | 0.569 | 0.686 | 0.038 | 0.044 |
| MH_03_medium (bag_start=0) | 1.170 | **diverged** | 0.115 | **diverged** |
| MH_03_medium (bag_start=5) | 1.164 | 1.031 | 0.089 | 0.116 |
| V1_03_difficult | 2.861 | 2.818 | 0.058 | 0.063 |

*Source: x86 from `results/stereo/estimate_{V1_01_easy,MH_03_medium,MH_03_medium_bagstart5,V1_03_difficult}.txt`; RPi5 from `results/rpi5/stereo/estimate_{V1_01_easy,MH_03_medium_diverged,MH_03_medium_bagstart5,V1_03_difficult}.txt`*

#### Relative Pose Error (median orientation / median position)

| Segment | V1_01 x86 | V1_01 RPi5 | MH_03 x86 (skip5) | MH_03 RPi5 (skip5) | V1_03 x86 | V1_03 RPi5 |
|---------|-----------|------------|--------------------|--------------------|-----------|------------|
| seg 8 | 0.528 / 0.057 | 0.546 / 0.055 | 0.373 / 0.133 | 0.295 / 0.147 | 0.894 / 0.081 | 0.892 / 0.083 |
| seg 16 | 0.368 / 0.051 | 0.589 / 0.054 | 0.514 / 0.117 | 0.437 / 0.123 | 1.068 / 0.102 | 1.136 / 0.104 |
| seg 24 | 0.467 / 0.047 | 0.564 / 0.062 | 0.494 / 0.121 | 0.527 / 0.144 | 1.306 / 0.114 | 1.109 / 0.117 |
| seg 32 | 0.565 / 0.051 | 0.702 / 0.067 | 0.709 / 0.140 | 0.604 / 0.158 | — | 1.116 / 0.135 |
| seg 40 | 0.600 / 0.038 | 0.495 / 0.068 | 0.761 / 0.138 | 0.678 / 0.184 | — | 1.100 / 0.132 |

*Source: x86 from `results/stereo/estimate_{V1_01_easy,MH_03_medium_bagstart5,V1_03_difficult}.txt`; RPi5 from `results/rpi5/stereo/estimate_{V1_01_easy,MH_03_medium_bagstart5,V1_03_difficult}.txt`*

#### Accuracy findings

1. **V1_03_difficult is nearly identical** across platforms — rmse_ori differs by
   only 0.04 deg. This is the best evidence that the algorithm runs correctly on ARM.

2. **V1_01_easy and MH_03 show chaotic divergence** — RPi5 wins some RPE segments,
   loses others. Neither platform is consistently worse. This is expected for a
   nonlinear EKF with different floating-point backends (NEON vs SSE/AVX).

3. **MH_03 stereo diverges on RPi5 without `bag_start:=5.0`.** The MAV is picked up
   and carried at the start of this sequence, creating a marginal initialization
   scenario. The x86 filter survives this section; the RPi5 filter does not.
   This is a known class of issue — see [rpng/open_vins#435](https://github.com/rpng/open_vins/issues/435)
   (pose divergence on RPi4) and [rpng/open_vins#141](https://github.com/rpng/open_vins/issues/141)
   (MH_03 initialization issues). Skipping 5 seconds fully resolves it.

4. **Mono mode is unaffected** — MH_03 mono works correctly on RPi5 even without
   `bag_start` (rmse_ori=1.306, rmse_pos=0.145 at bag_start=0 vs rmse_ori=1.334,
   rmse_pos=0.117 at bag_start=5 — both healthy). This confirms the divergence is
   specific to the stereo codepath during marginal initialization.

5. **Determinism:** Results are deterministic within the same platform in serial mode
   (same binary + same data = same output). `num_opencv_threads` does not affect
   results (verified with threads=1 vs threads=4). Cross-platform exact reproducibility
   is not achievable due to NEON vs SSE/AVX floating-point differences.

---

## Phase 4b: RPi5 config sensitivity sweeps

Same 5 config variants as Phase 2, run on V1_01_easy (stereo, serial mode) on RPi5.

### Results (all times in ms, V1_01_easy stereo)

| Variant | Tracking | Propagation | MSCKF upd | SLAM upd | SLAM delay | Re-tri/marg | **Total** | **p99** | **max** |
|---------|----------|-------------|-----------|----------|------------|-------------|-----------|---------|---------|
| **Baseline** (200pts, full-res, 4 thr) | 6.9 | 0.4 | 2.8 | 7.3 | 1.6 | 5.1 | **24.2** | **38.8** | **64.5** |
| **A: Downsample** | 4.6 | 0.5 | 2.6 | 7.5 | 1.8 | 1.9 | **18.8** | **29.1** | **48.0** |
| **B: 100 features** | 5.9 | 0.4 | 0.3 | 4.4 | 1.0 | 5.6 | **17.6** | **28.4** | **61.5** |
| **C: 300 features** | 10.1 | 0.5 | 6.2 | 7.9 | 2.0 | 6.4 | **33.1** | **48.5** | **68.4** |
| **D: No SLAM** | 8.2 | 0.2 | 5.3 | — | — | 5.3 | **19.1** | **32.3** | **53.8** |
| **E: 1 OpenCV thread** | 10.6 | 0.5 | 2.9 | 7.4 | 1.7 | 5.6 | **28.6** | **43.1** | **64.8** |

*Source: `results/timing/rpi5/serial/sweep/{A_downsample,B_num_pts_100,C_num_pts_300,D_no_slam,E_opencv_1thread}.txt`; baseline row from `results/timing/rpi5/serial/stereo/V1_01_easy.txt`*

### RPi5 sweep findings vs x86

| Variant | x86 total (ms) | RPi5 total (ms) | x86 savings | RPi5 savings | Ratio |
|---------|----------------|-----------------|-------------|--------------|-------|
| Baseline | 11.3 | 24.2 | — | — | 2.1x |
| A: Downsample | 9.5 (-16%) | 18.8 (**-22%**) | -16% | **-22%** | 2.0x |
| B: 100 features | 6.8 (-40%) | 17.6 (**-27%**) | -40% | **-27%** | 2.6x |
| C: 300 features | 14.3 (+27%) | 33.1 (**+37%**) | +27% | **+37%** | 2.3x |
| D: No SLAM | 7.4 (-35%) | 19.1 (**-21%**) | -35% | **-21%** | 2.6x |
| E: 1 OpenCV thread | 12.3 (+9%) | 28.6 (**+18%**) | +9% | **+18%** | 2.3x |

*Source: derived from `results/timing/{x86,rpi5}/serial/sweep/*.txt` + `results/timing/{x86,rpi5}/serial/stereo/V1_01_easy.txt`*

Key differences vs x86:

1. **Downsample is more effective on RPi5** (-22% vs -16%). The re-tri & marg
   component drops from 5.1ms to 1.9ms (-63%) — much larger than on x86 (-60%).
   Since this is the most cache-bound component (3.4x slowdown), halving the image
   resolution significantly reduces the feature data that must be re-triangulated.
   **Downsample is the best bang-for-buck optimization on RPi5.**

2. **100 features is less effective on RPi5** (-27% vs -40%). On x86, MSCKF update
   nearly vanished (1.6ms -> 0.1ms); on RPi5 it also drops (2.8ms -> 0.3ms) but the
   re-tri & marg component barely changes (5.1ms -> 5.6ms), eating into the savings.

3. **No SLAM is less effective on RPi5** (-21% vs -35%). SLAM update is eliminated
   but MSCKF update nearly doubles (2.8ms -> 5.3ms) as features are rerouted, and
   re-tri & marg stays constant at 5.3ms.

4. **1 OpenCV thread hurts more on RPi5** (+18% vs +9%). Tracking jumps from 6.9ms
   to 10.6ms (+54%). On ARM, the OpenCV NEON KLT benefits more from parallelism than
   on x86 with AVX2. **Keep 4 OpenCV threads on RPi5.**

5. **300 features is more dangerous on RPi5** (+37% vs +27%). At 33.1ms mean and
   48.5ms p99, this nearly exceeds the 50ms budget at 20Hz. Avoid.

### RPi5 optimization ranking (by absolute savings)

| Optimization | RPi5 savings | RPi5 total | p99 | 20Hz budget |
|-------------|-------------|-----------|-----|-------------|
| Downsample | -5.4ms (-22%) | 18.8ms | 29.1ms | comfortable |
| 100 features | -6.6ms (-27%) | 17.6ms | 28.4ms | comfortable |
| No SLAM | -5.1ms (-21%) | 19.1ms | 32.3ms | OK |
| Combined (downsample + 100pts) | est. -10ms | ~14ms | ~22ms | comfortable at 30Hz |

*Source: `results/timing/rpi5/serial/sweep/{A_downsample,B_num_pts_100,D_no_slam}.txt` (combined row is estimated, not measured)*

---

## Phase 4c: RPi5 subscribe mode

Subscribe mode (`run_subscribe_msckf`) with bag playback at 1x, 2x, and 5x rate
on V1_01_easy (stereo).

### Frame drop analysis

| Playback rate | Processed frames | "Dropped" | Drop % | Notes |
|---------------|-----------------|-----------|--------|-------|
| 1.0x (realtime) | 2800 | 112 | 3.8% | Same as x86 — init/sync, not perf |
| 2.0x | 2800 | 112 | 3.8% | No real drops |
| 5.0x | **1369** | **1543** | **53.0%** | **Real performance drops** |

*Source: `results/timing/rpi5/subscribe/V1_01_easy_rate{1.0,2.0,5.0}.txt`*

On x86, all three rates processed ~2800 frames (zero real drops even at 5x). On
RPi5, 1x and 2x are fine but **5x causes 53% frame loss** — the VIO pipeline
cannot keep up at 100Hz effective camera rate.

### Per-component timing: subscribe vs serial

| Component | Serial (ms) | Subscribe 1x (ms) | Subscribe 2x (ms) | Subscribe 5x (ms) |
|-----------|------------|-------------------|-------------------|-------------------|
| tracking | 6.9 | 7.3 | 7.7 | 11.8 |
| propagation | 0.4 | 0.4 | 0.2 | 0.2 |
| msckf update | 2.8 | 2.8 | 0.3 | 0.3 |
| slam update | 7.3 | 7.0 | 0.3 | 0.0 |
| slam delayed | 1.6 | 1.7 | 0.9 | 0.6 |
| re-tri & marg | 5.1 | 4.9 | 5.0 | 6.3 |
| **total** | **24.2** | **24.1** | **14.5** | **19.3** |

*Source: `results/timing/rpi5/subscribe/V1_01_easy_rate{1.0,2.0,5.0}.txt`; serial column from `results/timing/rpi5/serial/stereo/V1_01_easy.txt`*

### Phase 4c findings

1. **Subscribe 1x overhead is negligible on RPi5** (24.1ms vs 24.2ms serial). This
   is a major surprise — on x86, subscribe 1x was 1.85x slower than serial (20.9ms
   vs 11.3ms). On RPi5 there is effectively **no ROS2 middleware penalty**. Possible
   explanation: at 24ms per frame on RPi5 vs 11ms on x86, the VIO thread occupies the
   CPU more continuously, keeping caches warm and reducing the idle-time context
   switching that inflated x86 subscribe times.

2. **Subscribe 2x shows degraded filter** (14.5ms total). SLAM update drops to 0.3ms
   and MSCKF to 0.3ms — the same pattern as the diverged MH_03 run. At 2x rate, the
   RPi5 can't fully keep up, leading to a lighter (degraded) filter state. x86 at 2x
   was 13.9ms but still processed the same frame count — RPi5 also processes 2800
   frames but the filter is clearly running in a compromised mode.

3. **Subscribe 5x drops 53% of frames.** x86 dropped ~4% at 5x. This confirms the
   RPi5 breaking point is between 2x and 5x realtime for stereo with default config.

4. **Realtime verdict for RPi5 subscribe mode:**

   | Scenario | Mean (ms) | p99 (ms) | Budget 20Hz | Budget 30Hz |
   |----------|-----------|----------|-------------|-------------|
   | Stereo baseline | 24.1 | 39.4 | **OK** | p99 exceeds |
   | Stereo + downsample (est.) | ~19 | ~29 | OK | **OK** |
   | Mono baseline (est.) | ~17 | ~25 | OK | OK |

   *Source: `results/timing/rpi5/subscribe/V1_01_easy_rate1.0.txt`; downsample/mono rows projected from `results/timing/rpi5/serial/{sweep/A_downsample.txt,mono/V1_01_easy.txt}`*

   **Stereo at 20Hz is realtime-feasible on RPi5** — subscribe mode adds negligible
   overhead unlike on x86. For 30Hz (live USB camera), downsample or mono is needed.

### x86 vs RPi5 subscribe comparison

| Metric | x86 1x | RPi5 1x | Ratio |
|--------|--------|---------|-------|
| Total (ms) | 20.9 | 24.1 | 1.15x |
| Subscribe overhead vs serial | **1.85x** | **1.00x** | — |
| Frames at 5x | 2799 | 1369 | 2.0x drop |

*Source: `results/timing/{x86,rpi5}/subscribe/V1_01_easy_rate{1.0,5.0}.txt` + `results/timing/{x86,rpi5}/serial/stereo/V1_01_easy.txt`*

The subscribe overhead disappearing on RPi5 means the **serial timing numbers are
directly representative of realtime performance** — a major simplification for
planning.

---

## Raw timing data

All CSV timing files are committed in `results/timing/`:

```
results/timing/x86/
├── serial/
│   ├── stereo/
│   │   ├── V1_01_easy.txt     (2776 frames)
│   │   ├── MH_03_medium.txt   (2302 frames)
│   │   └── V1_03_difficult.txt (1990 frames)
│   ├── mono/
│   │   ├── V1_01_easy.txt     (2799 frames)
│   │   ├── MH_03_medium.txt   (2310 frames)
│   │   └── V1_03_difficult.txt (2004 frames)
│   └── sweep/
│       ├── A_downsample.txt   (2774 frames)
│       ├── B_num_pts_100.txt  (2776 frames)
│       ├── C_num_pts_300.txt  (2776 frames)
│       ├── D_no_slam.txt      (2776 frames)
│       └── E_opencv_1thread.txt (2776 frames)
├── subscribe/
│   ├── V1_01_easy_rate1.0.txt (2800 frames)
│   ├── V1_01_easy_rate2.0.txt (2800 frames)
│   └── V1_01_easy_rate5.0.txt (2799 frames)
│
results/timing/rpi5/
├── serial/
│   ├── stereo/
│   │   ├── V1_01_easy.txt              (2776 frames)
│   │   ├── MH_03_medium.txt            (2302 frames, diverged — see note)
│   │   ├── MH_03_medium_bagstart5.txt  (2302 frames, healthy filter)
│   │   └── V1_03_difficult.txt         (1990 frames)
│   ├── mono/
│   │   ├── V1_01_easy.txt     (2799 frames)
│   │   ├── MH_03_medium.txt   (2310 frames)
│   │   └── V1_03_difficult.txt (2004 frames)
│   └── sweep/
│       ├── A_downsample.txt   (2774 frames)
│       ├── B_num_pts_100.txt  (2776 frames)
│       ├── C_num_pts_300.txt  (2776 frames)
│       ├── D_no_slam.txt      (2776 frames)
│       └── E_opencv_1thread.txt (2776 frames)
└── subscribe/
    ├── V1_01_easy_rate1.0.txt (2800 frames)
    ├── V1_01_easy_rate2.0.txt (2800 frames)
    └── V1_01_easy_rate5.0.txt (1369 frames, 53% dropped)

results/rpi5/stereo/
├── estimate_V1_01_easy.txt
├── estimate_MH_03_medium_diverged.txt  (bag_start=0, diverged)
├── estimate_MH_03_medium_bagstart5.txt (bag_start=5, healthy)
└── estimate_V1_03_difficult.txt
```

## Scripts

All scripts are in the workspace root. They skip runs whose output already exists
(safe to re-run) and source ROS 2 internally.

| Script | Phase | What it does |
|--------|-------|-------------|
| `run_timing_benchmark.sh` | 1 | Runs serial mode on 3 sequences x {stereo, mono}. |
| `run_timing_sweep.sh` | 2 | Runs 5 config variants on V1_01_easy (serial mode). |
| `run_timing_subscribe.sh [rate]` | 3 | Subscribe mode + bag playback at given rate. |

## Key source files

| File | Relevance |
|------|-----------|
| `src/open_vins/ov_msckf/src/core/VioManager.cpp` | rT1-rT7 timing instrumentation and CSV output |
| `src/open_vins/ov_msckf/src/core/VioManagerOptions.h` | `record_timing_information` / `record_timing_filepath` |
| `src/open_vins/config/euroc_mav/estimator_config.yaml` | All tuning knobs referenced in this document |
| `src/open_vins/ov_msckf/src/ros2_serial_msckf.cpp` | Serial runner (deterministic, bag-direct) |
| `src/open_vins/ov_msckf/src/run_subscribe_msckf.cpp` | Subscribe runner (ROS2 realtime) |
| `src/open_vins/ov_eval/cmake/ROS2.cmake` | Which analysis tools are built for ROS2 |

## Next steps

- **Thermal throttling:** Run sustained benchmarks (all 3 sequences back-to-back)
  while monitoring `scaling_cur_freq` to quantify throttling impact.
- **CPU utilization profiling:** Monitor `htop` during subscribe mode to see core
  distribution between VIO thread, ROS2 executor, and OpenCV workers.
- **Accuracy impact:** Run `error_singlerun` on the optimized configs (100pts,
  downsample, no SLAM) to measure the accuracy tradeoff on EuRoC V1_01.
- **Deep profiling:** If a specific component needs sub-function analysis, use
  `perf record -g -F 999` to identify hot functions within tracking or SLAM update.
- **Live camera:** Test with RPi5 + Raspicam2 at 30Hz to validate the 33ms budget
  under real sensor conditions.
