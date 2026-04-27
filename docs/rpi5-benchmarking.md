# RPi5 Benchmarking

This doc collects all RPi5-specific timing and accuracy measurements. See
[rpi5-setup.md](rpi5-setup.md) for how to build and run on RPi5;
[timing.md](timing.md) and [benchmark-analysis.md](benchmark-analysis.md)
cover the x86 side.

| | |
|---|---|
| **Date** | 2026-04-12 |
| **Host** | Raspberry Pi 5 (BCM2712, 4× Cortex-A76 @ 2.4 GHz, 8 GB RAM) |
| **OS** | Debian 13 (Trixie), Docker container running Ubuntu 22.04 / ROS 2 Humble |
| **Build** | `colcon build --symlink-install` inside Docker (same source as x86) |
| **Dataset** | EuRoC MAV — same 3 sequences as x86 baseline (V1_01_easy, MH_03_medium, V1_03_difficult) |
| **Config** | Default: 200 features, full resolution, 4 OpenCV threads, 50 SLAM landmarks |

---

## 1. Pre-measurement projections

Before we had RPi5 hardware, we projected timing by scaling x86 serial
measurements using two factors:

- **CPU factor: 3.5×** — combines clock ratio (~1.5×), IPC (~1.3×), SIMD width
  (~2× AVX2 vs NEON), cache hierarchy (~1.5× 12 MB L3 vs 2 MB L2). Central
  estimate in a 3-4× range.
- **Subscribe factor: 1.85×** — from x86 Phase 3 (20.9 vs 11.3 ms on V1_01).

### Projected (stereo, default config, 200 features)

| Sequence | x86 serial (wall) | RPi5 serial (×3.5) | RPi5 subscribe (×3.5×1.85) | Budget |
|----------|-------------------|--------------------|-----------------------------|--------|
| V1_01_easy | 11.3 ms | ~40 ms | ~73 ms | 50 ms @20 Hz |
| MH_03_medium | 10.4 ms | ~36 ms | ~67 ms | 50 ms @20 Hz |
| V2_02_medium | 10.1 ms | ~35 ms | ~65 ms | 50 ms @20 Hz |

*Source: x86 serial wall from `results/timing/x86/serial/rerun_2026_04_23/{V1_01_easy,MH_03_medium,V2_02_medium}_4thr_wall.txt`; RPi5 columns are projections (×3.5, ×1.85), not measurements.*

**The projection said:** serial is within the 20 Hz budget with 10-15 ms
headroom; subscribe exceeds budget by 30-45% with default config.

### Projection vs actual (summary)

| | Projection | Actual (§3 below) |
|---|---|---|
| RPi5 serial / x86 serial ratio | 3.5× | **2.1-2.3×** |
| RPi5 subscribe overhead vs serial | 1.85× (same as x86) | **1.0×** (essentially none) |
| Stereo baseline RPi5 total | ~40 ms | **24 ms** |
| 20 Hz budget headroom | 10 ms | **26 ms** |

The actual numbers are much better than the projection. Two effects account
for the gap:

1. **Cortex-A76 is more capable than scaled.** The SIMD width factor (assumed
   2×) turned out closer to 1.6× because OpenCV's NEON KLT path is
   well-optimized; the cache-hierarchy factor (assumed 1.5×) hit only the
   re-triangulation step.
2. **Subscribe overhead disappears on RPi5.** With the persistent worker
   thread fix ([determinism.md](determinism.md)), subscribe now matches serial
   on both x86 and RPi5. On RPi5 specifically, the longer per-frame compute
   also keeps caches warm, so there's no idle-time context-switching penalty.

The rest of this doc is the actual measurements.

---

## 2. Phase 4: Serial baseline (actual hardware)

### Stereo timing results (ms)

**Clock:** wall (the RPi5 runs predate our 3-clock instrumentation — thread
CSVs not available for these sequences yet; re-measurement with thread clock is
tracked in §7 WIP). Format: `mean ± std (p99: N)`. **4 OpenCV threads.**

| Component | V1_01_easy | MH_03_medium | V1_03_difficult |
|-----------|-----------|-------------|----------------|
| tracking | 6.9 ± 1.5 (p99: 12.7) | 6.6 ± 1.3 (p99: 11.1) | 8.0 ± 2.4 (p99: 20.0) |
| propagation | 0.4 ± 0.2 (p99: 0.7) | 0.4 ± 0.1 (p99: 0.6) | 0.4 ± 0.1 (p99: 0.6) |
| msckf update | 2.8 ± 4.4 (p99: 19.1) | 2.0 ± 3.4 (p99: 15.4) | 2.0 ± 2.6 (p99: 14.2) |
| slam update | 7.3 ± 1.8 (p99: 13.3) | 5.9 ± 2.2 (p99: 9.8) | 3.4 ± 2.4 (p99: 8.6) |
| slam delayed | 1.6 ± 3.0 (p99: 13.8) | 2.2 ± 3.8 (p99: 16.7) | 2.6 ± 3.5 (p99: 17.1) |
| re-tri & marg | 5.1 ± 0.6 (p99: 7.9) | 4.9 ± 0.5 (p99: 7.1) | 5.2 ± 0.5 (p99: 7.5) |
| **total** | **24.2 ± 6.3** (p99: 44.5) | **22.0 ± 6.2** (p99: 43.0) | **21.6 ± 5.9** (p99: 40.6) |

*Source: `results/timing/rpi5/serial/stereo/{V1_01_easy,MH_03_medium_bagstart5,V1_03_difficult}.txt`*

Frames processed: V1_01=2776, MH_03=2302, V1_03=1990 (same as x86).

**Note on MH_03:** The default run (`bag_start=0`) causes the stereo filter to
diverge on RPi5 (see [accuracy section](#3-accuracy-rpi5-vs-x86) below). The
timing numbers above are from `bag_start:=5.0` which produces a healthy
filter. The diverged run (`MH_03_medium.txt` in the results directory) shows
artificially low times (14.5 ms total) because the filter runs idle with
near-zero SLAM/MSCKF updates.

### Mono timing results (ms)

**Clock:** wall. Format: `mean ± std (p99: N)`. **4 OpenCV threads.**

| Component | V1_01_easy | MH_03_medium | V1_03_difficult |
|-----------|-----------|-------------|----------------|
| tracking | 4.7 ± 1.0 (p99: 8.7) | 4.6 ± 1.1 (p99: 8.8) | 5.5 ± 2.3 (p99: 14.9) |
| propagation | 0.5 ± 0.1 (p99: 0.6) | 0.4 ± 0.1 (p99: 0.6) | 0.4 ± 0.1 (p99: 0.5) |
| msckf update | 3.3 ± 2.6 (p99: 10.6) | 2.6 ± 2.3 (p99: 9.6) | 2.0 ± 2.0 (p99: 8.8) |
| slam update | 4.2 ± 0.9 (p99: 6.7) | 3.5 ± 1.2 (p99: 5.8) | 2.2 ± 1.4 (p99: 5.0) |
| slam delayed | 1.1 ± 1.5 (p99: 6.6) | 1.6 ± 1.9 (p99: 8.4) | 2.0 ± 2.3 (p99: 9.3) |
| re-tri & marg | 3.2 ± 0.4 (p99: 4.4) | 3.1 ± 0.4 (p99: 4.3) | 2.9 ± 0.5 (p99: 3.9) |
| **total** | **17.1 ± 3.5** (p99: 26.9) | **15.8 ± 3.9** (p99: 27.3) | **15.0 ± 4.5** (p99: 28.3) |

*Source: `results/timing/rpi5/serial/mono/{V1_01_easy,MH_03_medium,V1_03_difficult}.txt`*

Frames processed: V1_01=2799, MH_03=2310, V1_03=2004.

### x86 vs RPi5 slowdown

#### Overall slowdown

| Mode | Sequence | x86 (ms) | RPi5 (ms) | Ratio |
|------|----------|---------|----------|-------|
| Stereo | V1_01_easy | 11.3 | 24.2 | 2.1× |
| Stereo | MH_03_medium | 10.5 | 22.0 | 2.1× |
| Stereo | V1_03_difficult | 9.5 | 21.6 | 2.3× |
| Mono | V1_01_easy | 8.2 | 17.1 | 2.1× |
| Mono | MH_03_medium | 7.7 | 15.8 | 2.1× |
| Mono | V1_03_difficult | 6.9 | 15.0 | 2.2× |

*Source: derived from `results/timing/{x86,rpi5}/serial/{stereo,mono}/*.txt`*

**Actual slowdown: 2.1-2.3×** — significantly better than the 3.5× projection.
The Cortex-A76 cores are more capable than estimated, especially for the
matrix-heavy EKF updates.

#### Per-component slowdown (RPi5 / x86, stereo, averaged across sequences)

| Component | Slowdown | Likely cause |
|-----------|----------|-------------|
| tracking | 2.6× | NEON (128-bit) vs AVX2 (256-bit) for KLT optical flow |
| propagation | 2.0× | Lightweight scalar math, stable across platforms |
| msckf update | 1.7× | Dense matrix ops, better than expected |
| slam update | 1.6× | Dense matrix ops, Eigen NEON is well-optimized |
| slam delayed | 1.8× | Triangulation + state augmentation |
| re-tri & marg | **3.4×** | Cache-bound: 2 MB L2 (RPi5) vs 12 MB L3 (x86) |

*Source: derived from `results/timing/{x86,rpi5}/serial/stereo/*.txt` (averaged across sequences)*

The re-triangulation & marginalization component scales worst (3.4×). This
step iterates over all active features and the full covariance matrix, making
it sensitive to cache size. The SLAM and MSCKF EKF updates (dense matrix
math) scale best at 1.6-1.7× — Eigen's NEON backend is effective for these
operations.

#### Realtime feasibility (serial mode)

| Scenario | RPi5 mean (ms) | RPi5 p99 (ms) | Budget 20 Hz (50 ms) | Budget 30 Hz (33 ms) |
|----------|---------------|---------------|--------------------|--------------------|
| Stereo baseline | 24.2 | 38.8 | OK | p99 exceeds |
| Mono baseline | 17.1 | 25.3 | OK | OK |

*Source: `results/timing/rpi5/serial/{stereo,mono}/V1_01_easy.txt`*

Serial mode is comfortably within the 20 Hz budget. Mono at 30 Hz has ~8 ms
headroom at p99.

---

## 3. Accuracy: RPi5 vs x86

All runs are stereo, serial mode, default config (200 features, max_slam=50).

> **Cross-platform determinism caveat.** Results on the same platform are
> bit-reproducible (RPi5 run A = RPi5 run B; x86 run A = x86 run B). Across
> platforms they are **not** bit-identical — RPi5 and x86 produce slightly
> different trajectories from the same input. The root cause is NEON (RPi5)
> vs SSE/AVX (x86) floating-point ordering in Eigen operations: the same
> dot-product computed in different SIMD lane counts yields different
> round-off. This is expected behavior for a nonlinear EKF, not an RPi5 bug.
> Trajectory-level metrics (ATE, RPE) stay comparable; see the tables below.

### Absolute Trajectory Error — 3-sequence subset (with MH_03 bag_start handling)

| Sequence | x86 rmse_ori (deg) | RPi5 rmse_ori (deg) | x86 rmse_pos (m) | RPi5 rmse_pos (m) |
|----------|--------------------|---------------------|-------------------|-------------------|
| V1_01_easy | 0.569 | 0.686 | 0.038 | 0.044 |
| MH_03_medium (bag_start=0) | 1.170 | **diverged** | 0.115 | **diverged** |
| MH_03_medium (bag_start=5) | 1.164 | 1.031 | 0.089 | 0.116 |
| V1_03_difficult | 2.861 | 2.818 | 0.058 | 0.063 |

*Source: x86 from `results/stereo/estimate_{V1_01_easy,MH_03_medium,MH_03_medium_bagstart5,V1_03_difficult}.txt`; RPi5 from `results/rpi5/stereo/estimate_{V1_01_easy,MH_03_medium_diverged,MH_03_medium_bagstart5,V1_03_difficult}.txt`. Provenance: x86 = Latitude 5420 / `master-candidate`/`2a50450` / `slam_chi2_recovery: false`; RPi5 = openhd@192.168.200.81 / Docker `openvins-humble:latest` rebuilt 2026-04-26 / same submodule + config.*

### Absolute Trajectory Error — full 5 Vicon × stereo+mono paper-repro

Cross-platform comparison of all 5 Vicon-room sequences from the paper Table II reproduction. Both columns at default `bag_start=0` and shipping config (`slam_chi2_recovery: false`).

| Sequence | x86 stereo (ori°/pos m) | RPi5 stereo (ori°/pos m) | x86 mono (ori°/pos m) | RPi5 mono (ori°/pos m) |
|----------|--------------------------|---------------------------|------------------------|-------------------------|
| V1_01_easy | 0.569 / 0.038 | 0.686 / 0.044 | 0.645 / 0.062 | 0.574 / 0.056 |
| V1_02_medium | 1.622 / 0.053 | 1.714 / 0.062 | 1.655 / 0.060 | 1.694 / 0.062 |
| V1_03_difficult | 2.861 / 0.058 | 2.818 / 0.063 | 2.673 / 0.073 | 2.926 / 0.070 |
| V2_01_easy | 1.250 / 0.063 | 1.097 / 0.067 | 1.314 / 0.163 | 0.942 / 0.126 |
| V2_02_medium | 1.212 / 0.051 | 1.166 / 0.048 | 1.477 / 0.078 | 1.797 / 0.072 |

*Source: x86 from `results/{stereo,mono}/estimate_{V1_01_easy,V1_02_medium,V1_03_difficult,V2_01_easy,V2_02_medium}{,_mono}.txt` (paper-repro committed estimates, regenerated under master-candidate); RPi5 from `results/rpi5/rerun_2026_04_26_paper/{V1_01_easy,V1_02_medium,V1_03_difficult,V2_01_easy,V2_02_medium}_4thr{,_mono}_pose.txt`. Provenance: same as table above. RPi5 paper-repro collected 2026-04-26 via `serial.launch.py` inside Docker `openvins-humble:latest`.*

**Cross-platform stereo position drift (RPi5 vs x86):** V1_01 +6 mm, V1_02 +9 mm, V1_03 +5 mm, V2_01 +4 mm, V2_02 −3 mm. RPi5 is consistently within ±10 mm of x86 on stereo position, with mixed signs — confirms the NEON-vs-SSE/AVX numerical drift caveat above is real but bounded.

### Relative Pose Error (median orientation / median position)

| Segment | V1_01 x86 | V1_01 RPi5 | MH_03 x86 (skip5) | MH_03 RPi5 (skip5) | V1_03 x86 | V1_03 RPi5 |
|---------|-----------|------------|--------------------|--------------------|-----------|------------|
| seg 8 | 0.528 / 0.057 | 0.546 / 0.055 | 0.373 / 0.133 | 0.295 / 0.147 | 0.894 / 0.081 | 0.892 / 0.083 |
| seg 16 | 0.368 / 0.051 | 0.589 / 0.054 | 0.514 / 0.117 | 0.437 / 0.123 | 1.068 / 0.102 | 1.136 / 0.104 |
| seg 24 | 0.467 / 0.047 | 0.564 / 0.062 | 0.494 / 0.121 | 0.527 / 0.144 | 1.306 / 0.114 | 1.109 / 0.117 |
| seg 32 | 0.565 / 0.051 | 0.702 / 0.067 | 0.709 / 0.140 | 0.604 / 0.158 | — | 1.116 / 0.135 |
| seg 40 | 0.600 / 0.038 | 0.495 / 0.068 | 0.761 / 0.138 | 0.678 / 0.184 | — | 1.100 / 0.132 |

*Source: x86 from `results/stereo/estimate_{V1_01_easy,MH_03_medium_bagstart5,V1_03_difficult}.txt`; RPi5 from `results/rpi5/stereo/estimate_{V1_01_easy,MH_03_medium_bagstart5,V1_03_difficult}.txt`*

### Accuracy findings

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

5. **Within-platform determinism:** Results are deterministic within the same
   platform in serial mode (same binary + same data = same output).
   `num_opencv_threads` does not affect results (verified with threads=1 vs
   threads=4).

---

## 4. Phase 4b: Config sensitivity sweeps

Same 5 config variants as timing.md Phase 2, run on V1_01_easy (stereo,
serial mode) on RPi5.

### Results (ms, V1_01_easy stereo serial)

**Clock:** wall. Per-component cells are mean; **Total** is `mean ± std` and
**p99** is the per-frame 99th percentile of the total.

| Variant | Tracking | Propagation | MSCKF upd | SLAM upd | SLAM delay | Re-tri/marg | **Total (mean ± std)** | **p99** |
|---------|----------|-------------|-----------|----------|------------|-------------|------------------------|---------|
| **Baseline** (200 pts, full-res, 4 thr) | 6.9 | 0.4 | 2.8 | 7.3 | 1.6 | 5.1 | **24.2 ± 6.3** | **44.5** |
| **A: Downsample** | 4.6 | 0.5 | 2.6 | 7.5 | 1.8 | 1.9 | **18.8 ± 4.4** | **32.5** |
| **B: 100 features** | 5.9 | 0.4 | 0.3 | 4.4 | 1.0 | 5.6 | **17.6 ± 4.6** | **39.7** |
| **C: 300 features** | 10.1 | 0.5 | 6.2 | 7.9 | 2.0 | 6.4 | **33.1 ± 6.6** | **54.1** |
| **D: No SLAM** | 8.2 | 0.2 | 5.3 | — | — | 5.3 | **19.1 ± 5.7** | **38.7** |
| **E: 1 OpenCV thread** | 10.6 | 0.5 | 2.9 | 7.4 | 1.7 | 5.6 | **28.6 ± 6.2** | **48.4** |

*Source: `results/timing/rpi5/serial/sweep/{A_downsample,B_num_pts_100,C_num_pts_300,D_no_slam,E_opencv_1thread}.txt`; baseline row from `results/timing/rpi5/serial/stereo/V1_01_easy.txt`*

### RPi5 sweep findings vs x86

| Variant | x86 total (ms) | RPi5 total (ms) | x86 savings | RPi5 savings | Ratio |
|---------|----------------|-----------------|-------------|--------------|-------|
| Baseline | 11.3 | 24.2 | — | — | 2.1× |
| A: Downsample | 9.5 (-16%) | 18.8 (**-22%**) | -16% | **-22%** | 2.0× |
| B: 100 features | 6.8 (-40%) | 17.6 (**-27%**) | -40% | **-27%** | 2.6× |
| C: 300 features | 14.3 (+27%) | 33.1 (**+37%**) | +27% | **+37%** | 2.3× |
| D: No SLAM | 7.4 (-35%) | 19.1 (**-21%**) | -35% | **-21%** | 2.6× |
| E: 1 OpenCV thread | 12.3 (+9%) | 28.6 (**+18%**) | +9% | **+18%** | 2.3× |

*Source: derived from `results/timing/{x86,rpi5}/serial/sweep/*.txt` + `results/timing/{x86,rpi5}/serial/stereo/V1_01_easy.txt`*

Key differences vs x86:

1. **Downsample is more effective on RPi5** (-22% vs -16%). The re-tri & marg
   component drops from 5.1 ms to 1.9 ms (-63%) — much larger than on x86 (-60%).
   Since this is the most cache-bound component (3.4× slowdown), halving the image
   resolution significantly reduces the feature data that must be re-triangulated.
   **Downsample is the best bang-for-buck optimization on RPi5.**

2. **100 features is less effective on RPi5** (-27% vs -40%). On x86, MSCKF update
   nearly vanished (1.6 ms → 0.1 ms); on RPi5 it also drops (2.8 ms → 0.3 ms) but the
   re-tri & marg component barely changes (5.1 ms → 5.6 ms), eating into the savings.

3. **No SLAM is less effective on RPi5** (-21% vs -35%). SLAM update is eliminated
   but MSCKF update nearly doubles (2.8 ms → 5.3 ms) as features are rerouted, and
   re-tri & marg stays constant at 5.3 ms.

4. **1 OpenCV thread hurts more on RPi5** (+18% vs +9%). Tracking jumps from 6.9 ms
   to 10.6 ms (+54%). On ARM, the OpenCV NEON KLT benefits more from parallelism than
   on x86 with AVX2. **Keep 4 OpenCV threads on RPi5.**

5. **300 features is more dangerous on RPi5** (+37% vs +27%). At 33.1 ms mean and
   48.5 ms p99, this nearly exceeds the 50 ms budget at 20 Hz. Avoid.

### RPi5 optimization ranking (by absolute savings)

| Optimization | RPi5 savings | RPi5 total | p99 | 20 Hz budget |
|-------------|-------------|-----------|-----|-------------|
| Downsample | -5.4 ms (-22%) | 18.8 ms | 29.1 ms | comfortable |
| 100 features | -6.6 ms (-27%) | 17.6 ms | 28.4 ms | comfortable |
| No SLAM | -5.1 ms (-21%) | 19.1 ms | 32.3 ms | OK |
| Combined (downsample + 100 pts) | est. -10 ms | ~14 ms | ~22 ms | comfortable at 30 Hz |

*Source: `results/timing/rpi5/serial/sweep/{A_downsample,B_num_pts_100,D_no_slam}.txt` (combined row is estimated, not measured)*

---

## 5. Phase 4c: Subscribe mode on RPi5

Subscribe mode (`run_subscribe_msckf`) with bag playback at 1×, 2×, and 5× rate
on V1_01_easy (stereo).

### Frame drop analysis

| Playback rate | Processed frames | "Dropped" | Drop % | Notes |
|---------------|-----------------|-----------|--------|-------|
| 1.0× (realtime) | 2800 | 112 | 3.8% | Same as x86 — init/sync, not perf |
| 2.0× | 2800 | 112 | 3.8% | No real drops |
| 5.0× | **1369** | **1543** | **53.0%** | **Real performance drops** |

*Source: `results/timing/rpi5/subscribe/V1_01_easy_rate{1.0,2.0,5.0}.txt`*

On x86, all three rates processed ~2800 frames (zero real drops even at 5×).
On RPi5, 1× and 2× are fine but **5× causes 53% frame loss** — the VIO
pipeline cannot keep up at 100 Hz effective camera rate.

### Per-component timing: subscribe vs serial (ms)

**Clock:** wall. Per-component cells are mean; totals shown as
`mean ± std` with separate p99 row.

| Component | Serial | Subscribe 1× | Subscribe 2× | Subscribe 5× |
|-----------|--------|--------------|--------------|--------------|
| tracking | 6.9 | 7.3 | 7.7 | 11.8 |
| propagation | 0.4 | 0.4 | 0.2 | 0.2 |
| msckf update | 2.8 | 2.8 | 0.3 | 0.3 |
| slam update | 7.3 | 7.0 | 0.3 | 0.0 |
| slam delayed | 1.6 | 1.7 | 0.9 | 0.6 |
| re-tri & marg | 5.1 | 4.9 | 5.0 | 6.3 |
| **total (mean ± std)** | **24.2 ± 6.3** | **24.1 ± 6.6** | **14.5 ± 4.2** | **19.3 ± 5.6** |
| **total p99** | **44.5** | **47.2** | **32.2** | **40.8** |

*Source: `results/timing/rpi5/subscribe/V1_01_easy_rate{1.0,2.0,5.0}.txt`; serial column from `results/timing/rpi5/serial/stereo/V1_01_easy.txt`*

### Findings

1. **Subscribe 1× overhead is negligible on RPi5** (24.1 ms vs 24.2 ms serial).
   On x86 (pre-persistent-worker), subscribe 1× was 1.85× slower than serial.
   On RPi5 there is effectively **no ROS 2 middleware penalty**. Possible
   explanation: at 24 ms per frame on RPi5 vs 11 ms on x86, the VIO thread
   occupies the CPU more continuously, keeping caches warm and reducing the
   idle-time context switching. The persistent worker thread fix
   ([determinism.md](determinism.md)) has since eliminated this gap on x86 too.

2. **Subscribe 2× shows degraded filter** (14.5 ms total). SLAM update drops to
   0.3 ms and MSCKF to 0.3 ms — the same pattern as the diverged MH_03 run. At
   2× rate, the RPi5 can't fully keep up, leading to a lighter (degraded) filter
   state. RPi5 processes the same 2800 frames but the filter is clearly running
   in a compromised mode.

3. **Subscribe 5× drops 53% of frames.** x86 dropped ~4% at 5×. This confirms the
   RPi5 breaking point is between 2× and 5× realtime for stereo with default
   config.

### Realtime verdict for RPi5 subscribe mode

| Scenario | Mean (ms) | p99 (ms) | Budget 20 Hz | Budget 30 Hz |
|----------|-----------|----------|-------------|-------------|
| Stereo baseline | 24.1 | 39.4 | **OK** | p99 exceeds |
| Stereo + downsample (est.) | ~19 | ~29 | OK | **OK** |
| Mono baseline (est.) | ~17 | ~25 | OK | OK |

*Source: `results/timing/rpi5/subscribe/V1_01_easy_rate1.0.txt`; downsample/mono rows projected from `results/timing/rpi5/serial/{sweep/A_downsample.txt,mono/V1_01_easy.txt}`*

**Stereo at 20 Hz is realtime-feasible on RPi5** — subscribe mode adds
negligible overhead unlike on pre-fix x86. For 30 Hz (live USB camera),
downsample or mono is needed.

### x86 vs RPi5 subscribe comparison

| Metric | x86 1× | RPi5 1× | Ratio |
|--------|--------|---------|-------|
| Total (ms) | 20.9 | 24.1 | 1.15× |
| Subscribe overhead vs serial | **1.85×** (pre-fix) | **1.00×** | — |
| Frames at 5× | 2799 | 1369 | 2.0× drop |

*Source: `results/timing/{x86,rpi5}/subscribe/V1_01_easy_rate{1.0,5.0}.txt` + `results/timing/{x86,rpi5}/serial/stereo/V1_01_easy.txt`*

The subscribe overhead disappearing on RPi5 means the **serial timing numbers
are directly representative of realtime performance** — a major simplification
for planning.

---

## Real-time scheduling flags tested

The PWT investigation (see [docs/determinism.md §6](determinism.md#6-rpi5-deployment-investigation))
compared timing variability with vs without Docker real-time scheduling flags.
The flags tested were:

| Flag | What it does |
|---|---|
| `--cap-add=SYS_NICE` | Lets processes call `sched_setscheduler()` to raise their own scheduling priority. Required for the `rtprio` ulimit to take effect. |
| `--ulimit rtprio=99` | Raises the RT-priority ceiling for processes inside the container from the default 0 to 99 (the highest non-kernel value). |
| `--ulimit memlock=-1` | Removes the per-process memory-lock cap — lets the runtime `mlockall()` to keep VIO state pages in RAM and avoid page-fault stalls. |
| `--cpuset-cpus=0-3` | Pins the container to CPU cores 0–3 (all four RPi5 A76 cores). Mostly for explicitness; the default scheduler already uses all cores. |

The exact comparison from the investigation finding **no measurable improvement
from RT flags** (cross-run wall-time CV stayed within session-noise) is
reproducible with two `run_full_benchmark.sh` invocations differing only in
`--docker-flags`:

```bash
# Without RT flags (baseline)
bash scripts/run_full_benchmark.sh -m subscribe -s V1_01_easy -t 4 -r 10 \
    --docker openvins-humble:latest --tag rerun_pwt_baseline

# With RT flags
bash scripts/run_full_benchmark.sh -m subscribe -s V1_01_easy -t 4 -r 10 \
    --docker openvins-humble:latest \
    --docker-flags '--cap-add=SYS_NICE --ulimit rtprio=99 --ulimit memlock=-1 --cpuset-cpus=0-3' \
    --tag rerun_pwt_rtflags

# Compare with the post-hoc analyzer:
python3 scripts/aggregate_pwt.py ~/results/timing/rpi5/subscribe/rerun_pwt_baseline --ate
python3 scripts/aggregate_pwt.py ~/results/timing/rpi5/subscribe/rerun_pwt_rtflags --ate
```

The two halves are sequential (one tag, then the other) to minimise
between-session drift; this is what the now-deleted `run_pwt_final_ab.sh`
wrapper used to do as a single command. With the consolidated orchestrator
the two invocations are explicit at the cost of one extra command line.

---

## 6. Raw timing data

```
results/timing/rpi5/
├── serial/
│   ├── stereo/
│   │   ├── V1_01_easy.txt              (2776 frames)
│   │   ├── MH_03_medium.txt            (2302 frames, diverged — see §3)
│   │   ├── MH_03_medium_bagstart5.txt  (2302 frames, healthy filter)
│   │   └── V1_03_difficult.txt         (1990 frames)
│   ├── mono/
│   │   ├── V1_01_easy.txt              (2799 frames)
│   │   ├── MH_03_medium.txt            (2310 frames)
│   │   └── V1_03_difficult.txt         (2004 frames)
│   └── sweep/
│       ├── A_downsample.txt            (2774 frames)
│       ├── B_num_pts_100.txt           (2776 frames)
│       ├── C_num_pts_300.txt           (2776 frames)
│       ├── D_no_slam.txt               (2776 frames)
│       └── E_opencv_1thread.txt        (2776 frames)
└── subscribe/
    ├── V1_01_easy_rate1.0.txt          (2800 frames)
    ├── V1_01_easy_rate2.0.txt          (2800 frames)
    └── V1_01_easy_rate5.0.txt          (1369 frames, 53% dropped)

results/rpi5/stereo/
├── estimate_V1_01_easy.txt
├── estimate_MH_03_medium_diverged.txt   (bag_start=0, diverged)
├── estimate_MH_03_medium_bagstart5.txt  (bag_start=5, healthy)
└── estimate_V1_03_difficult.txt
```

---

## 7. WIP / next steps

Things still pending on RPi5:

- **Thermal throttling validation.** Cortex-A76 throttles from 2.4 GHz → ~1.8 GHz
  under sustained load (passive cooling) or ~1.5 GHz (fanless case). No
  sustained-load measurements yet. Monitor via:
  ```bash
  cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq
  ```
  Proposed: run all 3 sequences back-to-back ×5 reps while logging freq.
- **Config sweep on MH_03 and V1_03.** Phase 4b swept only V1_01. MH_03 and
  V1_03 may respond differently (MH_03 has worse cache behavior; V1_03 has
  more tracking work).
- **Subscribe mode on MH_03 and V1_03.** Phase 4c tested only V1_01 at 1×/2×/5×.
  MH_03 divergence issue may also affect subscribe mode — untested.
- **Persistent-worker-thread benchmarks on RPi5** — *measured*, see
  [determinism.md §6](determinism.md#6-rpi5-follow-up-why-accuracy-variance-doesnt-transfer).
  Summary: subscribe overhead is ~1.05× serial on RPi5 post-fix (timing and
  SLAM health transfer cleanly), but accuracy variance does not transfer —
  see that section for the three-intervention comparison
  (baseline / RT flags / max-interval).
- **Live camera at 30 Hz.** RPi5 + Raspicam2 at 30 Hz with the downsample
  config has not been exercised end-to-end — see [rpi5-setup.md](rpi5-setup.md)
  §"Live-sensor deployment (future work)" for the wiring.
- **Process-CPU profiling under sustained load.** Expected ~2.6 cores avg
  for subscribe 4-thr from the x86 projection; unconfirmed on RPi5.
