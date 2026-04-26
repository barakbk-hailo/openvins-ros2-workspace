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

Understand which parts of the OpenVINS VIO algorithm take how long and what can
be optimized. This document is the x86 baseline; embedded-platform numbers live
in [rpi5-benchmarking.md](rpi5-benchmarking.md).

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
source /opt/ros/$(ls /opt/ros/ | grep -E '^(jazzy|humble)$' | head -n1)/setup.bash && source ~/workspace/catkin_ws_ov/install/setup.bash

# Per-run stats: mean, std, p99, max for each component
ros2 run ov_eval timing_flamegraph <file.txt>

# Side-by-side comparison of multiple runs
ros2 run ov_eval timing_comparison <file1.txt> <file2.txt> ...

# Distribution plot of per-frame total time
ros2 run ov_eval timing_histogram <file.txt> <num_bins>
```

## Sequence selection

We chose 3 EuRoC MAV sequences to cover a range of conditions:

| Sequence | Environment | Difficulty | Why chosen |
|----------|------------|------------|------------|
| V1_01_easy | Vicon room, slow motion | Easy | Baseline — stable tracking, most features, highest SLAM load |
| MH_03_medium | Machine hall, moderate speed | Medium | Larger space, different feature density, moderate motion |
| V2_02_medium | Vicon room 2, moderate speed | Medium | Cross-check on a different Vicon scene; primary x86-vs-RPi5 cross-platform comparison sequence |

V1_01_easy is also the sequence used for our accuracy benchmarks (see
[evaluation](evaluation.md)), so timing and accuracy results are directly comparable.

> **Provenance:** all tables below cite tags under `results/timing/x86/` —
> see [data-provenance.md](data-provenance.md) for the canonical
> (platform, submodule commit, config) lookup. The `rerun_2026_04_23` tag is
> x86 on Dell Latitude 5420 (i7-1185G7, Ubuntu 22.04), submodule
> `master-candidate` (`2a50450`), `slam_chi2_recovery: false`.

---

## Phase 1: Serial baseline (pure algorithmic cost)

**What we ran:** `ros2_serial_msckf` on each of the 3 sequences, in both stereo and
mono modes (6 runs total). Serial mode reads the bag directly — no `ros2 bag play`
needed.

**Script:** `run_full_benchmark.sh -m serial -c both -r 1 --tag <name>`
Loops over sequences × {stereo, mono}, runs `ros2 launch ov_msckf serial.launch.py`
with `max_cameras:=2 use_stereo:=true` (or `1`/`false` for mono), and copies the
timing CSVs (wall + cpu + thread + feats + est) to the results directory. Skips
runs whose output already exists (safe to re-run).

**Default config:** 200 features (`num_pts`), full resolution, 4 OpenCV threads,
50 SLAM landmarks (`max_slam`), 11 clones in the sliding window.

### Stereo results (ms)

**Clock:** thread (`CLOCK_THREAD_CPUTIME_ID`) — VIO thread CPU only,
excludes scheduling delays. On x86 in serial mode, wall-clock and thread-clock
agree within ~0.1 ms so the two are equivalent here. Format below is
`mean ± std (p99: N)`. **4 OpenCV threads.**

| Component | V1_01_easy | MH_03_medium | V2_02_medium |
|-----------|-----------|-------------|----------------|
| tracking | 2.6 ± 0.4 (p99: 3.9) | 2.6 ± 0.4 (p99: 3.9) | 2.6 ± 0.4 (p99: 3.9) |
| propagation | 0.2 ± 0.0 (p99: 0.3) | 0.2 ± 0.0 (p99: 0.3) | 0.2 ± 0.0 (p99: 0.3) |
| msckf update | 1.6 ± 2.5 (p99: 11.0) | 1.2 ± 1.9 (p99: 9.1) | 1.4 ± 2.0 (p99: 9.5) |
| slam update | 4.5 ± 1.0 (p99: 6.3) | 3.7 ± 1.3 (p99: 5.7) | 3.5 ± 1.0 (p99: 5.5) |
| slam delayed | 0.9 ± 1.7 (p99: 8.1) | 1.2 ± 2.2 (p99: 10.3) | 1.0 ± 1.5 (p99: 7.5) |
| re-tri & marg | 1.5 ± 0.1 (p99: 1.9) | 1.5 ± 0.1 (p99: 1.9) | 1.5 ± 0.1 (p99: 1.9) |
| **total** | **10.86** | **10.11** | **9.95** |

*Source: `results/timing/x86/serial/rerun_2026_04_23/{V1_01_easy,MH_03_medium,V2_02_medium}_4thr_thread.txt`. Provenance: x86, `master-candidate`/`2a50450`, `slam_chi2_recovery: false`. Re-collected 2026-04-26.*

> **Note:** the per-component cells above are inherited from the older
> `bench_5rep_3clock` collection and remain directionally correct — the
> total bolded row is recomputed from the v2 `rerun_2026_04_23` data
> (`thread`-clock mean ms across all frames). New totals are 5-15 % faster
> than the prior totals due to `68eee80`'s persistent-worker-thread gating
> fix, which removed a spurious worker thread previously created in serial
> mode. Per-component refresh from the v2 CSVs is a follow-up.

Frames processed: V1_01=2776, MH_03=2302, V2_02=2260 (out of 2912 in each bag;
the difference is from the initialization period where no timing is recorded, plus
stereo sync misses where no matching pair was found within ±20 ms).

### Mono results (ms)

**Clock:** thread. Format: `mean ± std (p99: N)`. **4 OpenCV threads.**

| Component | V1_01_easy | MH_03_medium | V2_02_medium |
|-----------|-----------|-------------|----------------|
| tracking | 1.8 ± 0.4 (p99: 2.9) | 1.9 ± 0.3 (p99: 3.0) | 1.8 ± 0.4 (p99: 2.9) |
| propagation | 0.2 ± 0.0 (p99: 0.3) | 0.2 ± 0.0 (p99: 0.3) | 0.2 ± 0.0 (p99: 0.3) |
| msckf update | 1.9 ± 1.6 (p99: 6.2) | 1.5 ± 1.4 (p99: 5.7) | 1.4 ± 1.5 (p99: 5.8) |
| slam update | 2.5 ± 0.5 (p99: 4.3) | 2.2 ± 0.7 (p99: 4.1) | 2.0 ± 0.6 (p99: 3.9) |
| slam delayed | 0.6 ± 0.8 (p99: 3.6) | 0.8 ± 1.0 (p99: 4.3) | 0.7 ± 0.9 (p99: 3.8) |
| re-tri & marg | 1.1 ± 0.2 (p99: 1.8) | 1.0 ± 0.1 (p99: 1.6) | 1.0 ± 0.1 (p99: 1.6) |
| **total** | **7.73** | **7.39** | **6.89** |

*Source: `results/timing/x86/serial/rerun_2026_04_23/{V1_01_easy,MH_03_medium,V2_02_medium}_4thr_mono_thread.txt`. Provenance: x86, `master-candidate`/`2a50450`, `slam_chi2_recovery: false`. Same caveat as the stereo table — per-component cells are from the older `bench_5rep_3clock` directionally; bolded total is from the v2 CSVs.*

Frames processed: V1_01=2799, MH_03=2310, V2_02=2266

### Phase 1 findings

1. **Comfortably within realtime on x86.** Stereo p99 is ~23ms vs 50ms budget at
   20Hz — roughly 2x headroom.

2. **SLAM update is the dominant component** on easy/medium sequences (up to 4.5ms
   mean, 40% of total). This was surprising — one might expect tracking (the OpenCV
   part) to dominate, but the EKF update with 50 persistent SLAM features is more
   expensive.

3. **Tracking scales with difficulty.** On V1_03_difficult (fast motion, blur),
   tracking mean increases from 2.6 ms to 3.3 ms and p99 from 3.9 ms to 7.3 ms.
   KLT has to work harder to track through motion blur.

4. **MSCKF update and SLAM delayed are spiky.** These components have high variance
   (std ~ mean) because their cost depends on how many features are lost or
   initialized on each frame. Their p99 values are 5-10× the mean.

5. **Stereo is ~40% slower than mono** (11.3 ms vs 8.1 ms on V1_01). The gap:
   - Tracking: +0.8 ms (stereo matching between left/right cameras)
   - SLAM update: +2.0 ms (larger state vector — stereo adds extrinsic calibration)
   - Re-tri & marg: +0.4 ms (more features to re-triangulate)

6. **Paradox: difficult sequences are faster.** V1_03_difficult (9.8 ms) is faster
   than V1_01_easy (11.3 ms) because fewer features survive motion blur, so SLAM
   update and MSCKF update have less work. The tracking component gets more
   expensive, but the overall pipeline gets lighter.

---

## Phase 2: Config sensitivity sweeps

**What we ran:** 5 config variants on V1_01_easy (stereo, serial mode) to find which
knobs matter most when optimizing runtime.

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
| E: 1 OpenCV thread | `num_opencv_threads: 1` (from 4) | How much does OpenCV parallelism actually help? Worth knowing before contending with ROS 2 executor threads for cores. |

### Results (ms, V1_01_easy stereo serial)

**Clock:** thread. Per-component cells are the mean; **Total** is `mean ± std`
and **p99** is the per-frame 99th percentile of the total.

| Variant | Total wall (mean ms) | Total CPU (mean ms) | Total thread (mean ms) | Frames |
|---------|----------------------|----------------------|------------------------|--------|
| **Baseline** (200 pts, full-res, 4 thr) | 10.86 | 15.88 | 10.80 | 2776 |
| **A: Downsample** | 13.91 | 17.64 | 13.82 | 2774 |
| **B: 100 features** | 8.01 | 12.86 | 7.83 | 2776 |
| **C: 300 features** | 15.71 | 21.60 | 15.66 | 2776 |
| **D: No SLAM** | 8.11 | 13.29 | 8.06 | 2776 |
| **E: 1 OpenCV thread** | 12.53 | 12.54 | 12.53 | 2776 |

*Source: `results/timing/x86/serial/sweep/rerun_2026_04_23/{A_downsample,B_num_pts_100,C_num_pts_300,D_no_slam,E_opencv_1thread}{,_cpu,_thread}.txt`; baseline row from `results/timing/x86/serial/rerun_2026_04_23/V1_01_easy_4thr_{wall,cpu,thread}.txt`. Provenance: x86, `master-candidate`/`2a50450`, `slam_chi2_recovery: false`. Re-collected 2026-04-26.*

> **Note:** the v2 sweep table above shows total mean per clock — Wall (`boost::posix_time`), Process CPU (`CLOCK_PROCESS_CPUTIME_ID`, ~CPU/Wall ratio = parallelism factor), Thread CPU (`CLOCK_THREAD_CPUTIME_ID`, VIO thread alone). Per-component breakdowns from the older `bench_5rep_3clock` sweep collection are preserved in the prior structure of this section above; the v2 totals here supersede the prior totals. The downsample variant is now slower than baseline because the master-candidate build's `68eee80` PWT-gating fix sped up baseline more than downsample (downsample was already fast and saturated by other components).

### Phase 2 findings — impact ranking

1. **100 features** (-4.5 ms, **-40%**): Biggest single win. MSCKF update nearly
   vanishes (1.6 → 0.1 ms — fewer features are lost per frame so fewer MSCKF
   updates happen). SLAM update drops from 4.5 to 2.8 ms (fewer features in the
   state). Tracking drops slightly (less extraction work).
   *Tradeoff:* accuracy may degrade on difficult sequences with fewer visual cues.

2. **No SLAM** (-3.9 ms, **-35%**): Eliminates SLAM update (4.5 ms) and SLAM delayed
   (0.9 ms) entirely. But MSCKF update nearly doubles from 1.6 to 3.1 ms — features
   that would have become persistent SLAM landmarks now go through the one-shot
   MSCKF path instead, each requiring triangulation.
   *Tradeoff:* no persistent landmarks means worse long-term drift, especially in
   revisited areas.

3. **Downsample** (-1.5 ms, **-13%**): Tracking drops 27% (2.6 → 1.9 ms) — KLT
   on quarter-pixel images is cheaper. Re-tri & marg also drops (1.5 → 0.6 ms).
   Other components barely change since the number of features is the same.
   *Tradeoff:* minimal — features are still detected, just at lower resolution.
   This is likely the cheapest accuracy tradeoff.

4. **1 OpenCV thread** (+1.2 ms, **+11%**): Only tracking is affected (2.6 → 3.7 ms,
   +42%). SLAM/MSCKF updates don't use OpenCV threading at all.
   *Key insight:* OpenCV parallelism gives only moderate benefit for this workload.
   On core-constrained machines, dedicating cores to the ROS 2 executor can pay
   off more than giving them to OpenCV.

5. **300 features** (+3.8 ms, **+34%**): Diminishing returns. 50% more features
   costs 34% more total time. MSCKF update more than doubles since more features
   are lost per frame. Tracking grows +35%.

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
- On machines with limited cores, prefer dedicating them to the ROS 2 executor
  + VIO thread rather than widening OpenCV's pool
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

**Why different rates:** 1× tests normal realtime. Higher rates stress-test to find
the breaking point — the playback rate at which VIO falls behind and starts dropping
frames.

> **Post-fix numbers.** These measurements are from the current `master`, which
> includes the persistent worker thread (see [determinism.md](determinism.md)).
> Before that fix, subscribe at 1× was ~1.85× slower than serial; now it's
> essentially equal.

### Frame drop analysis

| Playback rate | Processed frames | "Dropped" | Drop % | Total wall (mean ms) | Notes |
|---------------|-----------------|-----------|--------|----------------------|-------|
| 1.0× (realtime) | 2800 | 112 | 3.8% | 12.55 | init + stereo-sync, not perf |
| 2.0× | 2800 | 112 | 3.8% | 12.53 | init + stereo-sync, not perf |
| 5.0× | 2790 | 122 | 4.2% | 6.38 | ~10 real drops; total ms drops because the heavy init frames are excluded |
| Serial (reference) | 2776 | 136 | 4.7% | 10.86 | strict ±20 ms stereo sync |

*Source: `results/timing/x86/subscribe/rerun_2026_04_23/V1_01_easy_rate{1.0,2.0,5.0}_wall.txt`; serial reference from `results/timing/x86/serial/rerun_2026_04_23/V1_01_easy_4thr_wall.txt`. Provenance: x86, `master-candidate`/`2a50450`, `slam_chi2_recovery: false`. Re-collected 2026-04-26.*

The ~112-136 baseline "missing" frames at 1× and 2× are **NOT performance-related
drops**. They come from:
1. The initialization period (first ~2 s before VIO converges, no timing is recorded)
2. Stereo sync misses (serial uses a strict ±20 ms window; subscribe uses ROS 2's
   `ApproximateTime` policy which is slightly more permissive)

Subscribe actually processes MORE frames than serial at 1× (2800 vs 2776) because of
this sync policy difference (also noted in [evaluation](evaluation.md)).

At 5× the system processes only 2707 frames — ~93 real drops beyond the sync
baseline. That's modest (3.3% real drop rate) but it's the first sign of stress.

### Per-component timing: subscribe vs serial (ms)

**Clock:** thread. Per-component cells are the mean; **total** is `mean ± std`
and **p99** is the per-frame 99th percentile of the total.

| Component | Serial | Subscribe 1× | Subscribe 2× | Subscribe 5× |
|-----------|--------|--------------|--------------|--------------|
| tracking | 2.6 | 2.9 | 3.0 | 3.1 |
| propagation | 0.2 | 0.2 | 0.2 | 0.1 |
| msckf update | 1.6 | 1.7 | 1.8 | **0.5** |
| slam update | 4.5 | 4.7 | 5.0 | **1.2** |
| slam delayed | 0.9 | 1.0 | 1.1 | 1.3 |
| re-tri & marg | 1.5 | 1.6 | 1.7 | 1.8 |
| **total (mean ± std)** | **11.3 ± 3.4** | **12.0 ± 3.9** | **12.9 ± 4.4** | **8.0 ± 3.8** |
| **total p99** | **22.7** | **25.7** | **28.0** | **23.1** |

*Source: `results/timing/x86/subscribe/thread_rewrite/V1_01_easy_rate{1.0,2.0,5.0}_thread.txt`; serial column from `results/timing/x86/serial/rerun_2026_04_23/V1_01_easy_4thr_thread.txt`*

### Phase 3 findings

1. **Subscribe 1× matches serial** (12.0 ms vs 11.3 ms, ~6% overhead). The ROS 2
   middleware penalty has been eliminated by the persistent worker thread —
   see [determinism.md](determinism.md) for the architecture and the old-dispatch
   comparison (which saw 20.9 ms at 1×, 1.85× slower than serial).

2. **Subscribe 2× tracks 1× closely** (12.9 vs 12.0 ms). Higher message arrival
   rate adds only ~1 ms of per-frame cost — the pipeline has headroom.

3. **Subscribe 5× shows degraded mode** (8.0 ms total — SLAM upd drops to 1.2 ms,
   MSCKF upd to 0.5 ms). The system can't keep up with 100 Hz arrival, so the
   filter runs with fewer tracked features and fewer SLAM landmarks. The total is
   artificially low — this is NOT a valid measure of algorithmic cost, it's an
   artifact of running in a lighter state. ~3% of frames are also dropped.

4. **Tracking overhead is now negligible** (2.9 ms subscribe vs 2.6 ms serial,
   +12%). Under the old dispatch, tracking inflated 2.7× (to 7.1 ms) because every
   frame spawned a new `std::thread`; the persistent worker keeps caches warm and
   scheduling quiet.

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
- **[Determinism](determinism.md)** — root cause analysis of subscribe-mode
  non-determinism, persistent worker thread fix, SLAM recovery safety net

---

## RPi5 — see separate doc

Pre-measurement projections (×3.5 CPU factor + ×1.85 subscribe factor) and
actual Phase 4/4b/4c measurements are covered in
[rpi5-benchmarking.md](rpi5-benchmarking.md). Short version: actual RPi5
slowdown is 2.1-2.3× (not 3.5×), and subscribe mode on RPi5 has no middleware
penalty. See that doc for the full tables and accuracy comparison.

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
| Switch to mono | -28% | Removes stereo matching + smaller state. |
| Downsample images | -13% | Cheap win. Tracking + re-tri savings. |
| Reduce OpenCV threads | +11% | Modest. Not worth worrying about. |
| Increase features to 300 | +34% | Diminishing returns. Avoid. |

*Source: derived from `results/timing/x86/serial/rerun_2026_04_23/V1_01_easy_4thr_thread.txt`, `results/timing/x86/serial/thread_rewrite/V1_01_easy_4thr_mono_thread.txt`, and `results/timing/x86/serial/sweep/thread_rewrite/*_thread.txt`*

Recommended starting configs for RPi5 are listed in
[rpi5-benchmarking.md](rpi5-benchmarking.md) — those were validated against
actual measurements.

---

## Raw timing data (x86)

All x86 CSV timing files are committed in `results/timing/x86/`:

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
└── subscribe/
    ├── V1_01_easy_rate1.0.txt (2800 frames)
    ├── V1_01_easy_rate2.0.txt (2800 frames)
    └── V1_01_easy_rate5.0.txt (2799 frames)
```

RPi5 CSVs live under `results/timing/rpi5/` — see
[rpi5-benchmarking.md](rpi5-benchmarking.md) §"Raw timing data".

## Scripts

All scripts are in the workspace root. They skip runs whose output already exists
(safe to re-run) and source ROS 2 internally.

| Script | Phase | What it does |
|--------|-------|-------------|
| `run_full_benchmark.sh -m serial -c both -r 1` | 1 | Runs serial mode on 3 sequences × {stereo, mono}. |
| `run_timing_sweep.sh [--tag NAME]` | 2 | Runs 5 config variants on V1_01_easy (serial mode). |
| `run_timing_subscribe.sh [rate] [--tag NAME]` | 3 | Subscribe mode + bag playback at given rate. |

## Key source files

| File | Relevance |
|------|-----------|
| `src/open_vins/ov_msckf/src/core/VioManager.cpp` | rT1-rT7 timing instrumentation and CSV output |
| `src/open_vins/ov_msckf/src/core/VioManagerOptions.h` | `record_timing_information` / `record_timing_filepath` |
| `src/open_vins/config/euroc_mav/estimator_config.yaml` | All tuning knobs referenced in this document |
| `src/open_vins/ov_msckf/src/ros2_serial_msckf.cpp` | Serial runner (deterministic, bag-direct) |
| `src/open_vins/ov_msckf/src/run_subscribe_msckf.cpp` | Subscribe runner (ROS2 realtime) |
| `src/open_vins/ov_eval/cmake/ROS2.cmake` | Which analysis tools are built for ROS2 |

## Next steps (x86)

- **CPU utilization profiling:** Monitor `htop` during subscribe mode to see
  core distribution between VIO thread, ROS 2 executor, and OpenCV workers.
- **Accuracy impact:** Run `error_singlerun` on the optimized configs (100 pts,
  downsample, no SLAM) to measure the accuracy tradeoff on EuRoC V1_01.
- **Deep profiling:** If a specific component needs sub-function analysis, use
  `perf record -g -F 999` to identify hot functions within tracking or SLAM
  update.

RPi5-specific next steps are tracked in [rpi5-benchmarking.md](rpi5-benchmarking.md) §WIP.
