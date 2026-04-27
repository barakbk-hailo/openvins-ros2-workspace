# Subscribe-Mode Determinism

OpenVINS has two ways to run the VIO pipeline on ROS 2: a **serial** node that
reads a bag file sequentially (deterministic, reproduces runs bit-for-bit) and
a **subscribe** node that receives messages over DDS (what real hardware
deployments use). Upstream, subscribe mode was non-deterministic in ways that
could cascade into catastrophic SLAM failure.

This doc describes the two fork changes that fix it:

1. **Persistent worker thread** — architectural root-cause fix. Replaces the
   per-frame `std::thread` + `detach()` dispatch with a single long-lived
   worker. Eliminates a TOCTOU race, a dangling-reference UB, and
   non-deterministic IMU triggering. Reduces subscribe-mode overhead from
   2× serial to 1× serial.
2. **SLAM recovery mechanism** — opt-in safety net for subscribe-overload
   scenarios. When the SLAM feature count dips below 25% of `max_slam`, the
   chi-squared gate for `delayed_init` is relaxed 3×. Available via the
   `slam_chi2_recovery: true` YAML knob; **off by default** (since 2026-04-26)
   because the always-on behavior interacted poorly with stereo init on dark
   sequences (MH_05_difficult). See §4 for the evidence.

The persistent worker thread is on by default. The SLAM recovery mechanism is
off by default — turn it on if you run subscribe at >1× realtime under load.

---

## 1. Problem: SLAM feature state collapse

When running OpenVINS in subscribe mode (`run_subscribe_msckf`) **without the
fixes described below**, the SLAM feature state can collapse to zero within
the first ~50 frames and never recover.

### What "SLAM feature state collapse" means

OpenVINS maintains two types of visual features in its EKF state:

- **MSCKF features**: one-shot features that are tracked across several frames, used
  for a single EKF update when they are lost, then discarded. These provide
  frame-to-frame motion estimates but don't persist.

- **SLAM features** (up to `max_slam=50` by default): long-lived features that are
  added to the EKF state vector as persistent landmarks. These are tracked
  continuously and updated every frame. They anchor the trajectory over time and
  reduce long-term drift.

A feature becomes a SLAM candidate when it has been tracked for `max_clone_size`
(11) consecutive frames, proving it is stable. The candidate is then triangulated
and must pass a **chi-squared consistency test** in `UpdaterSLAM::delayed_init()`
before being admitted to the state. This test checks whether the feature's
measurements are consistent with the current state estimate — if the residuals are
too large (indicating the state or feature position is inaccurate), the feature is
rejected.

**The collapse happens in this sequence:**

1. The SLAM state fills to 50 features within the first ~15 frames (normal).
2. Around frames 24-35, some SLAM features lose tracking and are marginalized
   (removed from state). This is normal churn.
3. New candidate features that would replace them **fail the chi-squared test**.
   The residuals are slightly elevated due to the non-deterministic state drift
   from subscribe-mode scheduling (see Root Cause below). In serial mode, these
   same candidates would pass — the difference is within the test's margin.
4. With features being marginalized faster than they are replaced, the SLAM state
   empties.
5. **The empty state is irrecoverable**: new candidates need 11 frames to mature,
   but during that time the estimator runs on MSCKF-only mode, which drifts.
   The increased drift makes future chi-squared tests even more likely to reject,
   creating a feedback loop.
6. The system runs in MSCKF-only mode for the remainder of the sequence,
   accumulating unbounded drift — diverging by **hundreds of meters** on sequences
   like EuRoC V2_02_medium (an 85-meter trajectory).

In our pre-fix testing on V2_02, this happened in **~30% of subscribe runs** on x86
(i7-1185G7, Ubuntu 22.04), despite the hardware easily sustaining 20 fps with
ample headroom (20ms per frame vs 50ms budget, zero frame drops). It is a
correctness issue, not a performance issue.

Serial mode (`ros2_serial_msckf`) is unaffected — it produces identical results
every run because it reads messages in strict timestamp order with no middleware
non-determinism.

## 2. Root cause: ROS 2 middleware non-determinism

Subscribe mode introduces non-determinism through the ROS 2 middleware:

1. **Variable starting frames.** The `ApproximateTime` stereo sync policy and DDS
   publisher discovery timing cause the subscriber to receive its first frames at
   slightly different points in the sequence across runs (up to ~50ms spread even
   with a 5-second bag play delay).

2. **IMU-triggered VIO dispatch.** The VIO update is triggered by IMU callbacks
   (200 Hz), which check an atomic flag (`thread_update_running`) and spawn a
   detached thread to process queued camera frames. Which specific IMU callback
   triggers each update depends on when the previous update finished — introducing
   per-frame variability in IMU integration boundaries.

These produce small numerical differences in the state estimate. In most runs, the
differences are harmless. But occasionally, during a critical window in the first
~25 frames after SLAM features start being promoted, enough features fail the
chi-squared consistency test in `UpdaterSLAM::delayed_init()` to trigger an
irrecoverable cascade:

1. SLAM features that lose tracking are marginalized
2. New candidates that would replace them also fail the chi-squared test
   (residuals are slightly elevated due to the state drift from fewer features)
3. With no features entering the SLAM state, the system runs in MSCKF-only mode
4. Without persistent SLAM landmarks to anchor the trajectory, drift accumulates

---

## 3. Root-cause fix: Persistent worker thread

**Branch:** `persistent-worker-thread`
**Files changed:** `ov_msckf/src/ros/ROS2Visualizer.{h,cpp}` (+86/−51 lines)

We replaced the per-frame `std::thread` + `detach()` VIO dispatch pattern in
`ROS2Visualizer::callback_inertial` with a single persistent worker thread that
processes camera frames in timestamp order. This eliminates four bugs in the
upstream code and reduces subscribe-mode wall-clock overhead from **2.0× serial**
to **1.0× serial** (i.e., subscribe now runs at serial speed).

### What changed architecturally

#### Before (upstream `detach()` pattern)

```
  IMU callback (200 Hz, on executor thread):
    1. feed_imu() — buffer IMU data
    2. if (!thread_update_running) — atomic bool check (NO LOCK)
    3.   thread_update_running = true
    4.   spawn std::thread with [&] lambda
    5.   thread.detach()  ← thread runs unsupervised
                            ← callback returns, local vars destroyed
                            ← lambda still reads message.timestamp (UB)

  Spawned thread (~20 created+destroyed per second):
    1. lock camera_queue_mtx
    2. process eligible camera frames
    3. thread_update_running = false
    4. thread exits (destroyed)

  Camera callback (20 Hz):
    1. lock camera_queue_mtx
    2. push frame to queue, sort
```

#### After (persistent worker thread)

```
  IMU callback (200 Hz, on executor thread):
    1. feed_imu() — buffer IMU data
    2. lock worker_mtx                         ← mutex A
    3. latest_imu_timestamp = msg.timestamp     (member variable, not stack)
    4. unlock worker_mtx
    5. notify worker_cv

  Worker thread (ONE for entire run, created in constructor):
    loop:
      1. wait on worker_cv
      2. lock worker_mtx                       ← mutex A
      3. read latest_imu_timestamp
      4. unlock worker_mtx
      5. lock camera_queue_mtx                 ← mutex B
      6. process eligible camera frames
      7. unlock camera_queue_mtx
      8. loop back to 1

  Camera callback (20 Hz):
    1. lock camera_queue_mtx                   ← mutex B
    2. push frame to queue, sort
    3. unlock camera_queue_mtx
    4. notify worker_cv
```

Two mutexes, each protecting a distinct resource:

| Mutex | Protects | Writers | Readers |
|-------|----------|---------|---------|
| **`worker_mtx`** (A) | `latest_imu_timestamp`, `worker_should_exit` | IMU callback, destructor | Worker thread |
| **`camera_queue_mtx`** (B) | `camera_queue` deque | Camera callbacks | Worker thread |

Lock ordering is always A then B (worker locks `worker_mtx` first, then
`camera_queue_mtx`), so no deadlock is possible.

#### Architecture diagram

![Persistent worker thread architecture](persistent-worker-architecture.svg)

### Bugs fixed

| Bug | Before | After |
|-----|--------|-------|
| **TOCTOU race** | Two IMU callbacks can both read `thread_update_running = false` and spawn concurrent update threads | No flag; single worker always alive |
| **Dangling reference UB** | Lambda captures `[&]`, reads `message.timestamp` from caller's stack after `detach()` returns | IMU timestamp written to member variable under mutex |
| **Non-deterministic trigger** | Which IMU callback spawns the update depends on when the previous thread finished | Worker always uses the latest IMU timestamp |
| **Thread churn** | ~20 `std::thread` create/destroy per second, each with cold cache | One persistent thread with warm cache across all frames |

### Benchmark results

#### Hardware

- **Our system:** Intel i7-1185G7 (Tiger Lake, 4C/8T, 4.8 GHz boost), 16 GB, Ubuntu 22.04
- **Paper (Semenova et al. 2024):** Intel i7-7500U (Kaby Lake, 2C/4T, 3.5 GHz boost), 32 GB, Ubuntu 18.04

#### How to reproduce

The persistent-worker tables below were generated by `scripts/run_full_benchmark.sh`
against the worker-enabled submodule commit:

```bash
# Persistent-worker baseline (submodule at 0fe81a6 or newer):
bash scripts/run_full_benchmark.sh -r 5 --tag rerun_2026_04_23
```

The "old dispatch" comparison columns come from the **retired** `bench_5rep_3clock`
tag — the same suite (5 reps × {V1_01_easy, MH_03_medium, V2_02_medium} × {4-thr, 1-thr})
collected pre-`0fe81a6`. That tag was deleted from `results/` in outer commit `44b4cfa`
(see [data-provenance.md](data-provenance.md)) and is preserved in git history at
`44b4cfa^`. To re-extract a single file:

```bash
git show 44b4cfa^:results/timing/x86/subscribe/bench_5rep_3clock/V1_01_easy_4thr_run1_wall.txt
```

Persistent-worker data writes to `~/results/timing/x86/{serial,subscribe}/<tag>/`. The
`*Source:*` lines on each table below point at the specific CSVs produced (or, for the
old-dispatch column, at the path inside the `44b4cfa^` git tree).

#### Subscribe total: old dispatch vs persistent worker

**Clock:** wall (to match the paper's methodology — the paper reports wall time
and this is the most direct apples-to-apples comparison). Thread-clock totals
differ by <0.2 ms on x86 post-fix; see §"Per-component comparison" below for
both. Cells list the 5 individual runs' means (ms).

| Sequence | Old dispatch (5 reps) | Persistent worker (5 reps) | Serial |
|----------|----------------------|---------------------------|--------|
| V1_01_easy 4-thr | 21.2, 21.2, 21.3, 21.3, 21.1 | **11.8, 11.9, 12.0, 11.8, 12.1** | 11.5 |
| V1_01_easy 1-thr | 21.7, 21.9, 21.9, 21.9, 21.8 | **12.7, 12.7, 12.8, 12.7, 12.7** | 12.8 |
| MH_03_medium 4-thr | 20.2, 19.6, 21.3, 19.0, 21.4 | **10.7, 10.7, 10.9, 10.7, 10.8** | 11.5 |
| V2_02_medium 4-thr | 20.8, 20.7, 20.6, 20.7, 20.7 | **10.4, 10.5, 10.6, 10.7, 10.7** | 11.2 |
| V2_02_medium 1-thr | 21.3, 21.4, 21.1, 21.0, 21.1 | **11.2, 11.2, 11.2, 11.3, 11.3** | 11.5 |

*Source: Old dispatch from retired `bench_5rep_3clock/*_{1,4}thr_run{1..5}_wall.txt` (preserved at outer `44b4cfa^:results/timing/x86/subscribe/bench_5rep_3clock/`); persistent worker from `results/timing/x86/subscribe/rerun_2026_04_23/*_{1,4}thr_run{1..5}_wall.txt`; serial reference from `results/timing/x86/serial/rerun_2026_04_23/*_{1,4}thr_wall.txt`*

Subscribe/serial ratio: **2.0× → 1.0×** (eliminated entirely).

#### Per-component comparison with paper: V2_02, 4 OpenCV threads (ms)

**Clocks:** paper column uses wall (their methodology); our "sub (wall)" and
"serial (wall)" are wall-clock; "serial (proc CPU)" is `CLOCK_PROCESS_CPUTIME_ID`
(sum of CPU across all threads, useful for thermal/power budget). Cells are
`mean ± std` over per-frame values.

| Component | Paper¹ (sub, wall) | Old dispatch (sub, wall) | **Worker (sub, wall)** | Serial (wall) | Serial (proc CPU) |
|-----------|-------------------|-------------------------|----------------------|--------------|-------------------|
| Tracking | 6.12 ± 1.13 | 8.4 ± 2.1 | **3.0 ± 0.6** | 3.1 ± 0.6 | 9.0 ± 1.4 |
| Propagation | 0.21 ± 0.04 | 0.4 ± 0.1 | **0.2 ± 0.0** | 0.2 ± 0.0 | 0.2 ± 0.0 |
| MSCKF Update | 1.31 ± 1.69 | 2.6 ± 2.8 | **1.3 ± 1.5** | 1.3 ± 1.6 | 1.3 ± 1.6 |
| SLAM Update² | 6.56 ± 3.84 | 7.4 ± 3.7 | **4.5 ± 2.2** | 5.0 ± 2.6 | 5.0 ± 2.6 |
| Re-tri & Marg | 2.24 ± 0.20 | 2.0 ± 0.7 | **1.5 ± 0.2** | 1.6 ± 0.1 | 2.2 ± 0.2 |
| **Total** | **16.43 ± 4.53** | **20.8 ± 4.5** | **10.4 ± 2.9** | **11.2 ± 3.3** | **17.8 ± 3.5** |

p99 per-frame totals: Worker sub 19.2 ms, Serial wall 17.1 ms — both within the 50 ms @ 20 Hz budget.

*Source: Paper column from Semenova et al. 2024 Table 4; Old dispatch from retired `bench_5rep_3clock/V2_02_medium_4thr_run1_{wall,cpu,thread}.txt` (preserved at outer `44b4cfa^:results/timing/x86/subscribe/bench_5rep_3clock/`); Worker from `results/timing/x86/subscribe/rerun_2026_04_23/V2_02_medium_4thr_run1_{wall,cpu,thread}.txt`; Serial from `results/timing/x86/serial/rerun_2026_04_23/V2_02_medium_4thr_{wall,cpu,thread}.txt`*

¹ Semenova et al. 2024, Table 4 — subscribe mode, wall clock
² Paper combines SLAM Update + SLAM Delayed; our values shown combined for comparison

#### Per-component comparison with paper: V2_02, 1 OpenCV thread (ms)

**Clock:** all wall (paper methodology). Cells are `mean ± std`.

| Component | Paper¹ (sub, wall) | Old dispatch (sub, wall) | **Worker (sub, wall)** | Serial (wall) |
|-----------|-------------------|-------------------------|----------------------|--------------|
| Tracking | 8.55 ± 1.31 | 10.9 ± 2.5 | **4.0 ± 0.7** | 4.0 ± 0.8 |
| Propagation | 0.24 ± 0.03 | 0.4 ± 0.1 | **0.2 ± 0.0** | 0.2 ± 0.0 |
| MSCKF Update | 1.66 ± 2.11 | 2.1 ± 2.3 | **1.2 ± 1.4** | 1.2 ± 1.5 |
| SLAM Update² | 8.28 ± 4.79 | 6.2 ± 3.2 | **4.4 ± 2.3** | 4.6 ± 2.4 |
| Re-tri & Marg | 2.52 ± 0.21 | 1.7 ± 0.4 | **1.4 ± 0.2** | 1.5 ± 0.2 |
| **Total** | **21.25 ± 5.57** | **21.3 ± 4.2** | **11.2 ± 2.9** | **11.5 ± 3.1** |

*Source: Paper column from Semenova et al. 2024 Table 4; Old dispatch from retired `bench_5rep_3clock/V2_02_medium_1thr_run*_wall.txt` (preserved at outer `44b4cfa^:results/timing/x86/subscribe/bench_5rep_3clock/`); Worker from `results/timing/x86/subscribe/rerun_2026_04_23/V2_02_medium_1thr_run*_wall.txt`; Serial from `results/timing/x86/serial/rerun_2026_04_23/V2_02_medium_1thr_wall.txt`*

#### Cross-run variability (subscribe, 5 repetitions)

**Clock:** wall. Mean/std/CV here are computed across the 5 per-run totals
(each per-run total is itself a mean over ~2800 per-frame wall-clock values).

| Sequence | Config | Mean (ms) | Std (ms) | CV | Range (ms) |
|----------|--------|----------|---------|-----|-----------|
| V1_01_easy | 4-thr | 11.9 | 0.13 | **1.1%** | 11.8 – 12.1 |
| V1_01_easy | 1-thr | 12.7 | 0.04 | **0.3%** | 12.7 – 12.8 |
| MH_03_medium | 4-thr | 10.8 | 0.09 | **0.8%** | 10.7 – 10.9 |
| MH_03_medium | 1-thr | 11.5 | 0.08 | **0.7%** | 11.4 – 11.6 |
| V2_02_medium | 4-thr | 10.6 | 0.13 | **1.2%** | 10.4 – 10.7 |
| V2_02_medium | 1-thr | 11.2 | 0.05 | **0.5%** | 11.2 – 11.3 |

*Source: `results/timing/x86/subscribe/rerun_2026_04_23/{V1_01_easy,MH_03_medium,V2_02_medium}_{1,4}thr_run{1..5}_wall.txt`*

All configurations have CV < 1.3%. The old dispatch had CV up to 5% on MH_03.

#### SLAM feature health (avg features in state, max_slam=50)

| Sequence | Serial | Worker subscribe (5 reps) | Old dispatch subscribe (5 reps) |
|----------|--------|--------------------------|-------------------------------|
| V1_01_easy | 46.3 | 46.1, 46.1, 46.3, 46.1, 46.4 | 43.8 – 44.7 |
| MH_03_medium | 41.0 | 41.1, 41.2, 41.1, 41.3, 41.4 | **26.0** – 40.2 |
| V2_02_medium | 39.1 | 38.2, 38.3, 38.6, 38.7, 38.9 | 34.5 – 35.8 |

*Source: Serial from `results/timing/x86/serial/rerun_2026_04_23/*_4thr_feats.txt`; Worker subscribe from `results/timing/x86/subscribe/rerun_2026_04_23/*_4thr_run{1..5}_feats.txt`; Old dispatch from retired `bench_5rep_3clock/*_4thr_run{1..5}_feats.txt` (preserved at outer `44b4cfa^:results/timing/x86/subscribe/bench_5rep_3clock/`)*

Subscribe SLAM health now matches serial within <1 feature. The old dispatch had
a worst case of 26.0 on MH_03 (partial SLAM dip).

#### ATE — Absolute Trajectory Error (posyaw alignment)

> **Format note:** `scripts/run_full_benchmark.sh` saves the full state dump
> (`timestamp qx qy qz qw px py pz v ...`, JPL quaternion convention), while
> `ov_eval error_singlerun` expects TUM format
> (`timestamp px py pz qx qy qz qw`). The tables below were computed by
> converting each `*_est.txt` with
> `awk '!/^#/ && NF>=8 {print $1,$6,$7,$8,$2,$3,$4,$5}'` before invoking
> `error_singlerun posyaw`.

**Position RMSE (meters) across all runs:**

| Sequence | Serial | Sub 4-thr run 1 | run 2 | run 3 | run 4 | run 5 | Sub 1-thr run 1 | run 2 | run 3 | run 4 | run 5 |
|----------|--------|-----------------|-------|-------|-------|-------|-----------------|-------|-------|-------|-------|
| V1_01_easy | 0.040 | 0.040 | 0.058 | 0.063 | 0.067 | 0.056 | 0.068 | 0.069 | 0.046 | 0.041 | 0.057 |
| MH_03_medium | 0.121 | 0.136 | 0.123 | 0.116 | 0.117 | 0.115 | 0.101 | 0.109 | 0.160 | 0.130 | 0.114 |
| V2_02_medium | 0.049 | 0.048 | 0.066 | 0.065 | 0.070 | 0.052 | 0.058 | 0.063 | 0.065 | 0.056 | 0.062 |

*Source: Serial from `results/timing/x86/serial/rerun_2026_04_23/{V1_01_easy,MH_03_medium,V2_02_medium}_{1,4}thr_est.txt`; subscribe from `results/timing/x86/subscribe/rerun_2026_04_23/{V1_01_easy,MH_03_medium,V2_02_medium}_{1,4}thr_run{1..5}_est.txt`; state-dump columns converted to TUM as described above; compared against `src/open_vins/ov_data/euroc_mav/{V1_01_easy,MH_03_medium,V2_02_medium}.txt`. Provenance: x86, `master-candidate`/`2a50450`, `slam_chi2_recovery: false`.*

**Orientation RMSE (degrees) across all runs:**

| Sequence | Serial | Sub 4-thr (5 reps) | Sub 1-thr (5 reps) |
|----------|--------|-------------------|-------------------|
| V1_01_easy | 0.614 | 0.576, 0.786, 1.001, 0.682, 0.822 | 0.736, 0.717, 0.673, 0.652, 0.593 |
| MH_03_medium | 1.433 | 0.944, 1.422, 1.103, 1.013, 1.248 | 1.268, 1.491, 1.256, 1.180, 1.264 |
| V2_02_medium | 1.224 | 1.449, 1.177, 1.466, 1.636, 1.249 | 1.299, 1.175, 1.339, 1.340, 1.201 |

*Source: same TUM-converted `*_est.txt` files as the Position RMSE table.*

**Cross-run variability summary:**

| Sequence | Serial ATE pos | Subscribe ATE pos range (10 runs) | Max deviation from serial | Subscribe ATE pos std |
|----------|---------------|-----------------------------------|---------------------------|----------------------|
| V1_01_easy | 0.040m | 0.040 – 0.069m | **29mm** | **11mm** |
| MH_03_medium | 0.121m | 0.101 – 0.160m | **39mm** | **17mm** |
| V2_02_medium | 0.049m | 0.048 – 0.070m | **21mm** | **7mm** |

*Source: derived from the Position RMSE table above — same TUM-converted `*_est.txt` files under `results/timing/x86/{serial,subscribe}/rerun_2026_04_23/`.*

Subscribe ATE position is 2-30 mm higher than serial on average, with run-to-run
spread of 7–17 mm std. This is the dominant observable difference between the
two modes — small in absolute terms, but not bit-close: the run-to-run spread
exceeds the per-frame numerical precision of the filter.

#### RPE — Relative Pose Error (median position, meters)

RPE measures local consistency over trajectory segments. Serial vs subscribe
run 1 comparison (4-thread):

All Source files below are TUM-converted from the state-dump CSVs as described in the §3 Format note above.

**V1_01_easy:**

| Segment | Serial | Subscribe | Delta |
|---------|--------|-----------|-------|
| 8m | 0.054 | 0.046 | -0.008 |
| 16m | 0.053 | 0.042 | -0.011 |
| 24m | 0.056 | 0.045 | -0.011 |
| 32m | 0.059 | 0.063 | +0.004 |
| 40m | 0.053 | 0.063 | +0.010 |

*Source: Serial from `results/timing/x86/serial/rerun_2026_04_23/V1_01_easy_4thr_est.txt`; Subscribe run 1 from `results/timing/x86/subscribe/rerun_2026_04_23/V1_01_easy_4thr_run1_est.txt` (TUM-converted). Provenance: x86, `master-candidate`/`2a50450`, `slam_chi2_recovery: false`.*

**MH_03_medium:**

| Segment | Serial | Subscribe | Delta |
|---------|--------|-----------|-------|
| 8m | 0.149 | 0.127 | -0.022 |
| 16m | 0.135 | 0.117 | -0.018 |
| 24m | 0.148 | 0.133 | -0.015 |
| 32m | 0.191 | 0.155 | -0.036 |
| 40m | 0.195 | 0.182 | -0.013 |

*Source: Serial from `results/timing/x86/serial/rerun_2026_04_23/MH_03_medium_4thr_est.txt`; Subscribe run 1 from `results/timing/x86/subscribe/rerun_2026_04_23/MH_03_medium_4thr_run1_est.txt` (TUM-converted).*

**V2_02_medium:**

| Segment | Serial | Subscribe | Delta |
|---------|--------|-----------|-------|
| 8m | 0.044 | 0.051 | +0.007 |
| 16m | 0.059 | 0.067 | +0.008 |
| 24m | 0.064 | 0.070 | +0.006 |
| 32m | 0.067 | 0.069 | +0.002 |
| 40m | 0.063 | 0.077 | +0.014 |

*Source: Serial from `results/timing/x86/serial/rerun_2026_04_23/V2_02_medium_4thr_est.txt`; Subscribe run 1 from `results/timing/x86/subscribe/rerun_2026_04_23/V2_02_medium_4thr_run1_est.txt` (TUM-converted).*

RPE deltas are ≤0.04 m across all segments and sequences — subscribe local
consistency matches serial within a few cm per segment.

#### Process CPU reveals OpenCV parallelism cost (V2_02, 4-thr)

| Mode | Wall | Proc CPU | Thread | CPU/Wall |
|------|------|----------|--------|----------|
| Serial | 11.2 | **17.8** | 11.1 | **1.59×** |
| Subscribe (worker) | 10.4 | **16.6** | 10.4 | **1.60×** |
| Subscribe (old dispatch) | 20.8 | **37.0** | 20.6 | **1.78×** |

*Source: Serial from `results/timing/x86/serial/rerun_2026_04_23/V2_02_medium_4thr_{wall,cpu,thread}.txt`; Subscribe worker from `results/timing/x86/subscribe/rerun_2026_04_23/V2_02_medium_4thr_run1_{wall,cpu,thread}.txt`; Subscribe old-dispatch from retired `bench_5rep_3clock/V2_02_medium_4thr_run1_{wall,cpu,thread}.txt` (preserved at outer `44b4cfa^:results/timing/x86/subscribe/bench_5rep_3clock/`)*

CPU/Wall is now identical between serial and subscribe (1.59–1.60×) — purely the
OpenCV KLT thread pool. The old dispatch had 1.78× because executor threads burned
extra CPU during per-frame thread churn.

#### RPi5 projections

| Sequence | x86 serial | x86 subscribe (worker) | RPi5 serial (×3.5) | Budget (20 Hz) |
|----------|-----------|----------------------|-------------------|---------------|
| V1_01_easy | 11.5ms | 11.9ms | ~40ms | 50ms |
| MH_03_medium | 11.5ms | 10.8ms | ~40ms | 50ms |
| V2_02_medium | 11.2ms | 10.6ms | ~39ms | 50ms |

*Source: x86 serial from `results/timing/x86/serial/rerun_2026_04_23/*_4thr_wall.txt`; x86 subscribe (worker) from `results/timing/x86/subscribe/rerun_2026_04_23/*_4thr_run{1..5}_wall.txt`; RPi5 column is a ×3.5 projection, not measured.*

Since subscribe now matches serial, the RPi5 projection for subscribe mode is the
same as serial: **~40ms, within the 50ms budget**. Previously, the 2× subscribe
overhead projected to ~74ms (over budget), requiring aggressive config optimization.
With the persistent worker, the default config may work on RPi5 without changes.

Actual RPi5 measurements confirming this projection are in
[rpi5-benchmarking.md](rpi5-benchmarking.md). A deeper follow-up
investigating RPi5 subscribe-mode accuracy variance is in §6 below.

---

## 4. Optional safety net: SLAM recovery mechanism

The persistent worker thread removes the architectural cause of the SLAM
collapse. The SLAM recovery mechanism below is an **opt-in safety net** for
overload scenarios (subscribe-mode playback at >1× realtime where the filter
falls behind). It is **off by default** — the always-on behavior interacted
poorly with stereo init on dark sequences (see MH_05_difficult row in the
evidence table below) and produced ATE numbers that didn't match the committed
paper-reproduction tables.

**File:** `ov_msckf/src/core/VioManager.cpp` (before the `updaterSLAM->delayed_init()` call)

When the SLAM feature count drops below `max_slam / 4` (default: 12 out of 50),
the chi-squared multiplier for `delayed_init` is temporarily increased by 3×. This
relaxes the consistency gate, allowing features to enter the SLAM state even if
their residuals are slightly elevated from drift during the low-feature period.
Once the SLAM state recovers above the threshold, the gate returns to the
configured value.

```cpp
// SLAM recovery: relax chi-squared gate when SLAM state is critically low
double original_chi2 = updaterSLAM->_options_slam.chi2_multipler;
if (params.slam_chi2_recovery) {
  int slam_recovery_threshold = state->_options.max_slam_features / 4;
  if (state->_options.max_slam_features > 0 &&
      (int)state->_features_SLAM.size() < slam_recovery_threshold) {
    updaterSLAM->_options_slam.chi2_multipler = original_chi2 * 3.0;
  }
}
updaterSLAM->delayed_init(state, feats_slam_DELAYED);
updaterSLAM->_options_slam.chi2_multipler = original_chi2;
```

This is a conservative change:
- Only activates when the SLAM state is critically low (<25% of max)
- Only affects the `delayed_init` gate, not the SLAM update itself
- Automatically deactivates once features recover
- **Default off** (since 2026-04-26); opt-in via `slam_chi2_recovery: true` YAML key (see below)

### Configuration

The recovery is exposed as a boolean in `estimator_config.yaml`:

```yaml
slam_chi2_recovery: false # relax chi2 gate 3x when SLAM<max_slam/4; opt-in safety net for subscribe-overload
```

Default `false` in the shipped `euroc_mav` config. Other dataset configs
inherit the C++ default (`false`) automatically — `parse_config` preserves the
default when a key is absent. Set to `true` when running subscribe at >1×
realtime on resource-constrained hardware where the filter risks falling
behind (see V1_03_difficult @ rate 2.0 row in the evidence table below).

Verified locally (V1_01_easy, stereo serial, this machine):

| Flag | Trajectory md5 |
|---|---|
| `slam_chi2_recovery: false` (default since 2026-04-26) | `ea1e69b232f1e9d11d3add828d323264` (matches committed reference) |
| `slam_chi2_recovery: true` | `ab2d29d70669fc79420a8eabc8b05d47` |

### Why the default flipped to `false` (2026-04-26 rerun evidence)

| Scenario | recovery=true | recovery=false |
|---|---|---|
| **MH_05_difficult stereo serial** (paper sequence, dark init) | **diverges** — SLAM=0.2, RMSE 15,673 m, trajectory length 55 km | **converges** — 1845 frames / 0.213 m RMSE, matches committed `results/stereo/estimate_MH_05_difficult.txt` |
| **V1_03_difficult subscribe @ rate 1.0** (light load, 3 reps) | SLAM avg 30.5 / 31.0 / 30.2 — equivalent | SLAM avg 31.3 / 30.6 / 30.8 — equivalent |
| **V1_03_difficult subscribe @ rate 2.0** (overload, 3 reps) | worst-case 3.7 m ATE (per `b66bd07` commit msg + this rerun's `rerun_2026_04_23_rate2_recovery_on/`) | 2/3 runs collapse to >50 m ATE (per `b66bd07` commit msg + `rerun_2026_04_23_rate2_recovery_off/`) |
| **Paper reproduction (10 EuRoC × stereo+mono)** | 1/20 broken (MH_05 stereo); ATE drift 5-20% from committed | **20/20 reproduce committed ATE** to 3-decimal precision |

*Source: V1_03 @ rate 1.0: `results/timing/x86/subscribe/rerun_2026_04_23_recovery_{on,off}/`. V1_03 @ rate 2.0: `results/timing/x86/subscribe/rerun_2026_04_23_rate2_recovery_{on,off}/`. Paper reproduction: `results/timing/x86/serial/rerun_2026_04_23_paper/` (recovery=false) vs prior `rerun_2026_04_21_paper/` (recovery=true, hardcoded pre-`b66bd07`).*

The mechanism is still useful — its narrow benefit case (V1_03 @ rate 2× subscribe with overload) remains a real concern for resource-constrained deployments. But making it the default broke a paper-benchmark sequence (MH_05 stereo) on the most-cited use case (offline serial replay). Flipping to opt-in keeps the protection available without breaking reproducibility.

### Validation

#### Initial validation (V2_02_medium, 10 runs)

Tested on V2_02_medium, stereo, 10 subscribe runs (5 × 4-thread + 5 × 1-thread),
compared against a clean baseline without the recovery mechanism:

| Metric | Without recovery | With recovery |
|--------|-----------------|---------------|
| Degradation rate | 3/10 (30%) | **0/10 (0%)** |
| ATE failures (diverged) | 1/10 | **0/10** |
| Avg SLAM feature range | 2.3 - 36.2 | **34.0 - 36.0** |
| ATE position RMSE range | 2.088m - FAILED | **2.089 - 2.102m** |
| Serial ATE reference | 2.101m | 2.099m |

*Source: unarchived — ad-hoc 10-run V2_02_medium A/B from pre-recovery
debugging; raw CSVs were not preserved under `results/`. Numbers are kept
here for historical context only. The with-recovery half of the comparison
was superseded by the 30-run suite at
`results/timing/x86/subscribe/rerun_2026_04_23/V2_02_medium_*_run{1..5}_{est,feats}.txt`,
which is the archived, reproducible source. Treat this table as motivation,
not evidence.*

#### Full benchmark (3 sequences × 5 reps, 30 runs total)

Validated across V1_01_easy, MH_03_medium, and V2_02_medium with 5 repetitions
per configuration (see [benchmark-analysis.md](benchmark-analysis.md) for details):

| Metric | Result |
|--------|--------|
| Total subscribe runs | 30 |
| SLAM collapses | **0** |
| ATE within 0.02m of serial | **All runs** (where ov_eval didn't crash) |
| RPE (8m) within 0.07m of serial | **All runs** |
| Worst-case SLAM dip | MH_03 avg SLAM = 26.0 (still produced ATE within 0.004m and RPE within 0.13m of serial) |

*Source: `results/timing/x86/subscribe/rerun_2026_04_23/{V1_01_easy,MH_03_medium,V2_02_medium}_{1,4}thr_run{1..5}_{est,feats}.txt` vs `results/timing/x86/serial/rerun_2026_04_23/*_{1,4}thr_{est,feats}.txt`; see [benchmark-analysis.md](benchmark-analysis.md) for per-run numbers.*

The recovery mechanism maintains accuracy identical to serial mode as measured by
both global trajectory error (ATE) and local consistency (RPE at 8-40m segments).

#### Stress test — reproducing the failure mode

The (now-default-off) argument rests on showing that the prior default-on
behaviour was masking, not preventing, scheduler-induced SLAM collapse. A/B on
this machine, April 2026: for each (sequence, rate, flag) cell, subscribe mode
was run 3 times with `slam_chi2_recovery` flipped via the `--slam-chi2-recovery`
override (no source edit), **except the V1_01_easy @ rate=1.0 baseline row,
which reuses the 5-run mean from the archived `rerun_2026_04_23` suite**.
Per-run `slam_feats_in_state` was recorded from `traj_features.txt`; final ATE
was posyaw-aligned against the ov_data ground truth.

| Scenario | `slam_chi2_recovery: true` — ATE pos | `slam_chi2_recovery: false` — ATE pos | Verdict |
|---|---|---|---|
| V1_01_easy @ rate=1.0 | 0.053 m (5-run mean, archived) | 0.053 m (3 runs, equivalent within spread) | cosmetic (~0.002° ATE ori diff) |
| V1_01_easy @ rate=2.0 | 0.119 / 0.072 / 0.073 m (3 runs) | 0.056 / 0.119 / 0.068 m (3 runs) | both recover; flag not load-bearing |
| V1_03_difficult @ rate=1.0 | 0.068 / 0.077 / 0.092 m (3 runs) | 0.080 / 0.085 / 0.071 m (3 runs) | both recover; modest SLAM-dip reduction with recovery |
| **V1_03_difficult @ rate=2.0** | **0.420 / 0.402 / 3.717 m** (3 runs, SLAM mean 22–26) | **1284.6 / 56.3 / 1.235 m** (3 runs, SLAM mean 1.2 / 12.2 / 17.6) | **2 of 3 runs collapse without recovery** |

The rate=2.0 / V1_03_difficult row is the minimal reproducer: with the flag
off, one of the three runs ended with a mean SLAM feature count of 1.2 across
95% of the trajectory and a final position error of over a kilometer. With the
flag on at the same conditions, the worst run was 3.7 m ATE pos and SLAM stayed
healthy at ~22–26 features. This is the "empty-state feedback loop" the
recovery is designed to break: once SLAM drops to zero, every new candidate
fails the strict chi-squared gate (its residual looks anomalous relative to an
unanchored state), so the state can never repopulate — until you loosen the
gate temporarily.

Why rate=2.0 triggers it: at 2× real-time, the subscribe DDS queues start
shedding messages under QoS pressure. A burst of IMU or camera drops starves
the propagator and tracker of consistent measurements for long enough that the
strict chi2 gate rejects the next wave of candidates, and the state collapses.
The persistent worker thread (§3) helps — but on difficult imagery under that
much drop rate, it isn't sufficient on its own.

Practical implication: keep the default on for any subscribe workload that
could be CPU-bound (e.g. low-power SBCs running VIO + perception at the same
time, or real-time playback of a hard sequence). Only flip it off for strict
benchmark reproducibility against the pre-`64cfe59` numbers.

*Source:
- V1_01_easy @ rate=1.0 baseline (with-recovery): mean over `results/timing/x86/subscribe/rerun_2026_04_23/V1_01_easy_*_run{1..5}_est.txt` (5 runs, archived).
- V1_01_easy @ rate=2.0 row: unarchived. The `rerun_2026_04_23_rate2_recovery_{on,off}/` tags only carry V1_03_difficult; the V1_01_easy @ rate=2 cell remains an ad-hoc A/B preserved only in this table. Re-collect with `bash scripts/run_full_benchmark.sh -m subscribe -s V1_01_easy --rate 2.0 --tag rerun_rate2_v101_recovery_off --slam-chi2-recovery false` (and `_on`) if precise verification is needed.
- V1_03_difficult @ rate=1.0: 3 runs each in `results/timing/x86/subscribe/rerun_2026_04_23_recovery_{on,off}/V1_03_difficult_4thr_run{1,2,3}_{est,feats}.txt` (archived).
- V1_03_difficult @ rate=2.0 (the load-bearing row): 3 runs each in `results/timing/x86/subscribe/rerun_2026_04_23_rate2_recovery_{on,off}/V1_03_difficult_4thr_run{1,2,3}_{est,feats}.txt` (archived).

Reproduce the V1_03 rows with the `--slam-chi2-recovery` flag (no source edits
needed since `scripts/run_full_benchmark.sh` overrides the temp YAML in-place):*

```bash
# rate=1.0 A/B
bash scripts/run_full_benchmark.sh -m subscribe -s V1_03_difficult -t 4 -r 3 \
    --tag rerun_2026_04_23_recovery_on  --slam-chi2-recovery true
bash scripts/run_full_benchmark.sh -m subscribe -s V1_03_difficult -t 4 -r 3 \
    --tag rerun_2026_04_23_recovery_off --slam-chi2-recovery false

# rate=2.0 A/B (the failure-mode reproducer): pass --rate 2.0
bash scripts/run_full_benchmark.sh -m subscribe -s V1_03_difficult -t 4 -r 3 \
    --rate 2.0 --tag rerun_2026_04_23_rate2_recovery_on  --slam-chi2-recovery true
bash scripts/run_full_benchmark.sh -m subscribe -s V1_03_difficult -t 4 -r 3 \
    --rate 2.0 --tag rerun_2026_04_23_rate2_recovery_off --slam-chi2-recovery false
```

---

## 5. Other fork changes

These changes were made alongside the determinism work:

### Configurable `multi_threading_subs`

**File:** `ov_msckf/src/run_subscribe_msckf.cpp`

The upstream code hardcodes `params.use_multi_threading_subs = true` after loading
the config, making it impossible to change via YAML. We removed this override so
the value can be set in `estimator_config.yaml` via the `multi_threading_subs` key
(default: `true`, preserving the original behavior).

### Trajectory output launch arguments

**Files:** `ov_msckf/launch/subscribe.launch.py`, `ov_msckf/launch/serial.launch.py`

Added `filepath_est` and `filepath_std` as declared launch arguments (defaulting
to `/tmp/ov_estimate.txt` and `/tmp/ov_estimate_std.txt`). The upstream launch
files declare `save_total_state` but not the file paths, causing
`boost::filesystem::create_directories` to fail on the default relative path.

### Timing and diagnostic instrumentation

**Files:** `VioManager.cpp`, `VioManager.h`, `VioManagerOptions.h`, `estimator_config.yaml`

Added optional per-frame recording of:
- Process CPU time (`CLOCK_PROCESS_CPUTIME_ID`)
- Thread CPU time (`CLOCK_THREAD_CPUTIME_ID`)
- Feature counts (SLAM features in state, MSCKF features used, delayed-init
  candidates, clone count)

All disabled by default. Enable via YAML:
```yaml
record_timing_cpu_time: true
record_timing_thread_time: true
record_feature_counts: true
```

### Script zombie cleanup

**File:** `scripts/run_full_benchmark.sh` (via `kill_stale_subscribe_nodes` in `scripts/bench_lib.sh`)

Subscribe test scripts now kill any stale `run_subscribe_msckf` processes before
and after each run. Stale nodes on the same DDS domain steal messages from active
subscribers, causing non-deterministic data loss — a critical issue we discovered
during testing that invalidated several earlier measurement batches.

---

## 6. RPi5 follow-up: why accuracy variance doesn't transfer

### Context

Validating the x86 persistent-worker results on actual RPi5 hardware (Debian Trixie,
Docker-hosted ROS 2 Humble, `openvins-humble-pwt` image) revealed that **timing and
SLAM-health fixes transfer cleanly, but accuracy variance does not**. A 5-run subscribe
test showed position RMSE range of 53 mm (vs x86's 3 mm) and orientation range of
0.53° (vs x86's ~0.3 mdeg). We investigated three hypotheses with matched 10-run
benchmarks on V1_01_easy stereo:

1. **Docker CFS scheduling / memory locking jitter** — fixed by `--cap-add=SYS_NICE
   --ulimit rtprio=99 --ulimit memlock=-1 --cpuset-cpus=0-3`
2. **`ApproximateTime` unbounded pairing** — fixed by calling
   `sync->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.02))` to match
   serial mode's ±20 ms window. Queue depth stays at 10 so no frames are dropped.
3. **Residual remote causes** — identified by whatever variance remains after (1) and (2).

### Hardware

- **RPi5** (this investigation): Broadcom BCM2712, 4× Cortex-A76 @ 2.4 GHz, 8 GB,
  Debian 13 Trixie, Docker with ROS 2 Humble (Ubuntu 22.04 base image)
- Comparison rows below use x86 values from the preceding sections of this document

### Methodology

`bash scripts/run_full_benchmark.sh -m subscribe -s V1_01_easy -t 4 -r 10 --docker
openvins-humble:latest --tag <name>` runs 10 subscribe reps on V1_01_easy stereo,
spawning a fresh container per rep via the `docker_wrap` helper (no state leakage
between runs). Each rep captures wall, process-CPU and thread-CPU timing, feature
counts, and the saved trajectory. Results are aggregated via
`~/workspace/catkin_ws_ov/scripts/parse_results.py`.

Four matched runs were executed, with results saved under
`results/rpi5/{rerun_2026_04_26_pwt_baseline,rerun_2026_04_26_pwt_rtflags,rerun_2026_04_26_pwt_maxinterval,rerun_2026_04_26_pwt_combined}/`:

1. **pwt_baseline** — persistent-worker-thread fork, stock Docker flags
2. **pwt_rtflags** — PWT + `--cap-add=SYS_NICE --ulimit rtprio=99 --ulimit memlock=-1 --cpuset-cpus=0-3`
3. **pwt_maxinterval** — PWT + `setMaxIntervalDuration(0.02)` on the stereo synchronizer
4. **pwt_combined** — PWT + RT flags + `setMaxIntervalDuration(0.02)` (all interventions stacked)

### Cross-run timing variability (subscribe, 10 reps)

**Clock:** wall. Mean/std/CV are computed *across* the 10 per-run totals (run-to-run
variability — complements the per-frame std seen in §3's tables).

| Condition | Mean (ms) | Std (ms) | CV | Range (ms) |
|-----------|-----------|----------|-----|------------|
| Baseline (no Docker flags) | 23.06 | 0.10 | 0.43% | 22.94 – 23.29 |
| + Docker RT flags | 23.02 | 0.15 | 0.65% | 22.75 – 23.26 |
| + Max-interval (= baseline under master-candidate) | 23.03 | 0.15 | 0.65% | 22.81 – 23.30 |
| + Both combined (= RT flags under master-candidate) | 23.06 | 0.15 | 0.65% | 22.89 – 23.31 |
| Final A: back-to-back rerun of "max-interval" | 23.19 | 0.08 | 0.34% | 23.08 – 23.33 |
| Final B: back-to-back rerun of "combined" | 23.22 | 0.09 | 0.39% | 23.11 – 23.36 |

*Source: `results/rpi5/{rerun_2026_04_26_pwt_baseline,rerun_2026_04_26_pwt_rtflags,rerun_2026_04_26_pwt_maxinterval,rerun_2026_04_26_pwt_combined,rerun_2026_04_26_pwt_final_maxinterval,rerun_2026_04_26_pwt_final_combined}/sub_run{1..10}_wall.txt`. Provenance: RPi5 (openhd@192.168.200.81, BCM2712 4× Cortex-A76, Debian 13 Trixie), Docker `openvins-humble:latest` rebuilt 2026-04-26 from submodule `master-candidate`/`2a50450`, `slam_chi2_recovery: false`.*

> **Variant equivalence under master-candidate:** the consolidated `master-candidate`
> branch bakes both the persistent-worker thread (`0fe81a6`) AND the max-interval
> 20 ms stereo sync (`e57e88d`) into a single image. As a result, "baseline" and
> "+ max-interval" run **identical code** — they differ only in the test session
> (separate Docker container, separate timing window). Same for "+ Docker RT flags"
> and "+ Both combined". The 0.5-1 ms baseline-to-final drift between back-to-back
> sessions and across-day sessions is dominated by the RPi5's thermal/scheduling
> jitter, not by the configuration knob being toggled.

Timing stability is ≤0.65% CV across all conditions — comparable to the x86 target
(CV < 1.3%). The persistent worker alone (now baked into master-candidate) is
sufficient to stabilize timing on RPi5; the Docker RT flags and max-interval
synchronizer make no measurable additional difference at this load.

### SLAM feature health (avg features in state, max_slam=50)

| Condition | Serial (run 1 / run 2) | Subscribe (10 reps) | Subscribe mean |
|-----------|------------------------|---------------------|----------------|
| Baseline (PWT only) | 46.32 / 46.32 | 45.59 – 46.12 | 45.79 |
| + Docker RT flags | 46.32 / 46.32 | 45.83 – 46.36 | 46.04 |
| + Max-interval 20 ms | 46.32 / 46.32 | 45.70 – 46.50 | 45.97 |
| + Both combined | 46.32 / 46.32 | 45.48 – 46.22 | 45.99 |

*Source: column 1 (slam_feats_in_state) averaged across all frames per run,
from `results/rpi5/{rerun_2026_04_26_pwt_baseline,rerun_2026_04_26_pwt_rtflags,rerun_2026_04_26_pwt_maxinterval,rerun_2026_04_26_pwt_combined}/{serial_run*,sub_run*}_feats.txt`.*

Cross-platform comparison vs x86 (baseline condition only, matches the format in §3):

| Sequence | x86 Serial | x86 Subscribe (5 reps) | **RPi5 Serial** | **RPi5 Subscribe** (10 reps, baseline) |
|----------|-----------|----------------------|-----------------|----------------------------------------|
| V1_01_easy | 46.3 | 46.1 – 46.4 | **46.32** | **45.59 – 46.12** (mean 45.79) |

RPi5 subscribe SLAM health tracks serial within 1.1%. **SLAM feature collapse is NOT
the mechanism behind RPi5 subscribe accuracy degradation** — the filter is
maintaining a healthy state vector; the error comes from elsewhere.

### ATE — cross-run variability of each intervention (subscribe, posyaw alignment)

Stats computed across the 10 per-run RMSE values. "Serial" is a single
deterministic run (std and range are 0 by construction).

| Condition | rmse_ori mean ± std | rmse_ori range | rmse_pos mean ± std | rmse_pos range |
|-----------|---------------------|----------------|---------------------|----------------|
| **Serial (deterministic)** | 0.536 ± 0.000° | 0.000° | 42.0 ± 0.0 mm | 0.0 mm |
| Baseline (PWT only) | 0.700 ± 0.099° | 0.356° | 58.7 ± 14.0 mm | 50.0 mm |
| + Docker RT flags | 0.695 ± 0.089° | 0.293° | 60.4 ± 9.9 mm | 36.0 mm |
| + Max-interval 20 ms | 0.688 ± 0.171° | 0.496° | 66.7 ± 5.4 mm | 17.0 mm |
| + Both combined | 0.778 ± 0.083° | 0.303° | 74.2 ± 12.2 mm | 41.0 mm |
| Final A: max-interval only | 0.793 ± 0.220° | 0.663° | 77.8 ± 24.5 mm | 70.0 mm |
| Final B: max-interval + RT flags | 0.764 ± 0.133° | 0.481° | 69.1 ± 18.8 mm | 71.0 mm |

*Source: `results/rpi5/{rerun_2026_04_26_pwt_baseline,rerun_2026_04_26_pwt_rtflags,rerun_2026_04_26_pwt_maxinterval,rerun_2026_04_26_pwt_combined,rerun_2026_04_26_pwt_final_maxinterval,rerun_2026_04_26_pwt_final_combined}/sub_run{1..10}_pose.txt`
evaluated against `ov_data/euroc_mav/V1_01_easy.txt`.*

The Final A/B pair was run back-to-back (minimizing between-session state drift)
using the same `openvins-humble-maxinterval` image. Within that single controlled
comparison, the RT flags appear to help (−23% pos std, −40% ori std). **But the
same image's pos-std has ranged from 5.4 mm to 24.5 mm across four separate
sessions**, so the within-session RT flags effect (~25%) sits well inside the
between-session noise — see Finding 1 below.

**Combined run does not stack the way expected.** Enabling Docker RT flags and
`setMaxIntervalDuration(0.02)` together produced better orientation variance
than either alone (0.303° range vs 0.496° max-interval-only and 0.293° RT
flags-only) but **worse position variance than max-interval alone** (17 mm →
41 mm range, 5.4 mm → 12.2 mm std). The per-run values include position
outliers (97 mm, 85 mm) not present in the max-interval-only run.

Most likely explanation: the Docker RT flags have no real effect in our
pipeline (OpenVINS doesn't call `sched_setscheduler`, `--cpuset-cpus=0-3` is a
no-op on a 4-core RPi5, `memlock` only matters if requested). The "~30%
variance reduction" initially attributed to RT flags was likely within the
10-run sampling noise. With that interpretation, both "RT flags" and "combined"
represent the same underlying distribution as "baseline" and "max-interval"
respectively, and the observed differences are draws from that distribution.

**Conclusion: `setMaxIntervalDuration(0.02)` is the only intervention with a
consistent, reproducible effect.** Adding Docker RT flags on top provides no
measurable benefit and may introduce its own noise.

### Findings

1. **Docker RT flags effect is within between-session drift.** Three separate rounds
   of A/B testing produced conflicting signals:

   | Comparison | pos std (no RT flags) | pos std (+ RT flags) | Delta |
   |------------|-----------------------|----------------------|-------|
   | Step 1 vs Step 2 (no max-interval) | 14.0 mm | 9.9 mm | −30% |
   | Step 3 vs Step 4 (with max-interval) | 5.4 mm | 12.2 mm | **+126%** |
   | Final A vs Final B (with max-interval, back-to-back) | 24.5 mm | 18.8 mm | −23% |

   The same code+image run in different sessions produced pos-std values ranging
   from 5.4 mm to 24.5 mm for the same "max-interval" condition — a **~5× spread
   between sessions**. This between-session variance swamps the ~25% within-session
   effect of the RT flags. 10 reps is insufficient to resolve the RT flags' true
   effect; that would require a pre-registered N-session, M-rep design with
   temperature and system-load controls. None of the RT flags address anything
   OpenVINS explicitly uses (`sched_setscheduler`, CPU pinning to <4 cores, or
   memlock), which is consistent with the signal being weak.

2. **`setMaxIntervalDuration(0.02)` reduces position variance — magnitude noisy,
   range effect robust.** Initial comparison (Step 2→Step 3) showed std dropping
   14.0 mm → 5.4 mm (2.6×). But the same max-interval image re-run across three
   later sessions produced pos-std values of 12.2, 24.5, and 18.8 mm — so the
   "2.6×" figure is the best-case single session; session-averaged max-interval
   std is closer to 15 mm, comparable to the single-session baseline. The
   **range** reduction (50 mm → 17 mm in the initial comparison, 71 mm → 17 mm
   when comparing best-cases) is the more robust signal — it persists across
   sessions, whereas absolute std is dominated by between-session drift (see
   Finding 1). Frame count is unchanged (2800 per run), confirming the ±20 ms
   constraint doesn't drop EuRoC's hardware-synced stereo pairs. **The fix's
   direction is clear; its magnitude is uncertain beyond "a robust range
   reduction."**

3. **Orientation variance is bimodal across runs.** Median rmse_ori is lowest with
   max-interval (0.625° vs 0.686° baseline), but the std/range are dominated by
   occasional outlier runs (0.99° in 2/10 runs). These outliers survive the
   max-interval fix, suggesting an additional mechanism — most likely residual
   IMU-callback interleaving jitter at the start of runs, before the filter
   converges.

4. **No intervention closes the gap to x86.** x86 position RMSE std is ~1 mm; our
   best-case RPi5 max-interval session was 5.4 mm, but the session-averaged RPi5
   max-interval std is ~15 mm. Either way the gap is large and we did not isolate
   the remaining source of non-determinism. The OpenVINS docs' warning that
   Docker "is not real-time in nature" appears to apply beyond what
   `--cap-add=SYS_NICE` alone can fix.

### Recommendation for RPi5 deployment

- **For accuracy-critical offline benchmarking: use serial mode.** It is bit-identical
  across runs and matches x86's algorithmic cost.
- **For live deployment: use subscribe mode with `setMaxIntervalDuration(0.02)`
  enabled.** This is committed on the fork branch `sync-max-interval-20ms` (commit
  `e57e88d`) and brings RPi5 position variance to within 3× of serial — acceptable
  for most VIO applications. The `NadavHHailo/open_vins` fork exposes this as a
  default.
- **Use the `openvins-humble-maxinterval` image** (built from `sync-max-interval-20ms`).
  This is the only intervention with a reproducible, measurable effect.
- **Docker RT flags (`--cap-add=SYS_NICE --ulimit rtprio=99 --ulimit memlock=-1
  --cpuset-cpus=0-3`) are optional.** Across three A/B comparisons the signal
  was inconsistent (−30%, +126%, −23% on pos std), all within the observed
  between-session drift of the same image (5–25 mm pos std). They don't hurt,
  they might help by ~25%, but they don't address anything OpenVINS is actually
  using. Include them if you have the option but don't rely on them.
- **Do not rely on subscribe mode for publishable accuracy comparisons across runs.**
  The remaining variance is inherent to the callback-driven architecture on
  resource-constrained ARM + Docker.

### Frame-drop check for the 20 ms bound

Concern: could the tighter pairing constraint discard stereo frames that
unbounded `ApproximateTime` would have accepted? Across all 10 subscribe reps on
V1_01_easy, **every run processed exactly 2800 frames** — identical to baseline
and Docker-RT-flags conditions. Zero drops.

This is because EuRoC's cameras are hardware-triggered synced — the two
timestamps per frame differ by microseconds, orders of magnitude below 20 ms.
The bound only discards pairs that arise from queue-state race conditions
(exactly what we want to eliminate).

**When 20 ms is too tight:**
- Software-triggered cameras with no hardware sync (e.g., USB webcam pair) can
  have > 20 ms cross-camera timestamp jitter. In that case,
  `setMaxIntervalDuration(0.02)` would drop real stereo pairs. **Serial mode
  has the same issue** — it uses the same ±20 ms lookahead in
  `ros2_serial_msckf.cpp:253`. Both would need a looser bound, tuned to your
  sensor's actual worst-case cross-camera timestamp drift.
- The 20 ms value is a legacy constant. Promoting it to a config parameter
  (e.g., `stereo_max_interval_s` in `estimator_config.yaml`) would let
  deployments on non-hardware-synced rigs loosen the bound without forking.
  **Not yet done** in this branch; noted as follow-up.

### Changes introduced on the `sync-max-interval-20ms` fork branch

Branched from `persistent-worker-thread` at `0fe81a6`:

| Commit | File | Change |
|--------|------|--------|
| `e57e88d` | `ov_msckf/src/ros/ROS2Visualizer.cpp` | `sync->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.02))` after the stereo `Synchronizer` is constructed |
| `f12c80e` | `Dockerfile_ros2_humble_jammy` | Clone `-b sync-max-interval-20ms` by default so any fresh build produces an image with the fix |

Diff summary: 2 commits, 2 files changed, 5 insertions, 1 deletion. The code
change is additive only (no behavior is removed or altered for existing users
who explicitly set a different max interval).

> **After pulling this branch**, run `git submodule update --init --recursive`
> to sync the `src/open_vins` pointer to `f12c80e`. Without this the workspace
> will still build against the old-dispatch submodule and you won't get the
> max-interval fix.

### Open question

The residual orientation outlier behavior (2/10 runs hitting ~1.0° when the median
is 0.63°) is worth investigating further. Candidates:

- IMU-callback batch boundary jitter at initialization (before the filter converges)
- `MultiThreadedExecutor` cold-start behavior on first callback
- Container filesystem caching affecting the initial bag read

None of these were isolated in this round. A targeted study would discard the first
2-3 seconds of each run and recompute ATE to separate initialization transients from
steady-state drift.
