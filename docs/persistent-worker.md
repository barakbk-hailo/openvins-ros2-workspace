# Persistent Worker Thread: Subscribe-Mode Root-Cause Fix

## Summary

We replaced the per-frame `std::thread` + `detach()` VIO dispatch pattern in
`ROS2Visualizer::callback_inertial` with a single persistent worker thread that
processes camera frames in timestamp order. This eliminates four bugs in the
upstream code and reduces subscribe-mode wall-clock overhead from **2.0x serial**
to **1.0x serial** (i.e., subscribe now runs at serial speed).

**Branch:** `persistent-worker-thread`
**Files changed:** `ov_msckf/src/ros/ROS2Visualizer.{h,cpp}` (+86/−51 lines)

## What changed architecturally

### Before (upstream `detach()` pattern)

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

### After (persistent worker thread)

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

### Architecture diagram

![Persistent worker thread architecture](persistent-worker-architecture.svg)

## Bugs fixed

| Bug | Before | After |
|-----|--------|-------|
| **TOCTOU race** | Two IMU callbacks can both read `thread_update_running = false` and spawn concurrent update threads | No flag; single worker always alive |
| **Dangling reference UB** | Lambda captures `[&]`, reads `message.timestamp` from caller's stack after `detach()` returns | IMU timestamp written to member variable under mutex |
| **Non-deterministic trigger** | Which IMU callback spawns the update depends on when the previous thread finished | Worker always uses the latest IMU timestamp |
| **Thread churn** | ~20 `std::thread` create/destroy per second, each with cold cache | One persistent thread with warm cache across all frames |

## Benchmark results

### Hardware

- **Our system:** Intel i7-1185G7 (Tiger Lake, 4C/8T, 4.8 GHz boost), 16 GB, Ubuntu 22.04
- **Paper (Semenova et al. 2024):** Intel i7-7500U (Kaby Lake, 2C/4T, 3.5 GHz boost), 32 GB, Ubuntu 18.04

### Subscribe-mode wall-clock total (ms): old dispatch vs persistent worker

| Sequence | Old dispatch (5 reps) | Persistent worker (5 reps) | Serial |
|----------|----------------------|---------------------------|--------|
| V1_01_easy 4-thr | 21.2, 21.2, 21.3, 21.3, 21.1 | **11.8, 11.9, 12.0, 11.8, 12.1** | 11.5 |
| V1_01_easy 1-thr | 21.7, 21.9, 21.9, 21.9, 21.8 | **12.7, 12.7, 12.8, 12.7, 12.7** | 12.8 |
| MH_03_medium 4-thr | 20.2, 19.6, 21.3, 19.0, 21.4 | **10.7, 10.7, 10.9, 10.7, 10.8** | 11.5 |
| V2_02_medium 4-thr | 20.8, 20.7, 20.6, 20.7, 20.7 | **10.4, 10.5, 10.6, 10.7, 10.7** | 11.2 |
| V2_02_medium 1-thr | 21.3, 21.4, 21.1, 21.0, 21.1 | **11.2, 11.2, 11.2, 11.3, 11.3** | 11.5 |

*Source: Old dispatch from `results/timing/x86/subscribe/bench_5rep_3clock/*_{1,4}thr_run{1..5}_wall.txt`; persistent worker from `results/timing/x86/subscribe/bench_persistent_worker/*_{1,4}thr_run{1..5}_wall.txt`; serial reference from `results/timing/x86/serial/bench_persistent_worker/*_{1,4}thr_wall.txt`*

Subscribe/serial ratio: **2.0x → 1.0x** (eliminated entirely).

### Per-component comparison with paper: V2_02, 4 OpenCV threads (ms)

| Component | Paper¹ (sub, wall) | Old dispatch (sub, wall) | **Worker (sub, wall)** | Serial (wall) | Serial (proc CPU) |
|-----------|-------------------|-------------------------|----------------------|--------------|-------------------|
| Tracking | 6.12 ± 1.13 | 8.4 ± 2.1 | **3.0 ± 0.6** | 3.1 ± 0.6 | 9.0 ± 1.4 |
| Propagation | 0.21 ± 0.04 | 0.4 ± 0.1 | **0.2 ± 0.0** | 0.2 ± 0.0 | 0.2 ± 0.0 |
| MSCKF Update | 1.31 ± 1.69 | 2.6 ± 2.8 | **1.3 ± 1.5** | 1.3 ± 1.6 | 1.3 ± 1.6 |
| SLAM Update² | 6.56 ± 3.84 | 7.4 ± 3.7 | **4.5 ± 2.2** | 5.0 ± 2.6 | 5.0 ± 2.6 |
| Re-tri & Marg | 2.24 ± 0.20 | 2.0 ± 0.7 | **1.5 ± 0.2** | 1.6 ± 0.1 | 2.2 ± 0.2 |
| **Total** | **16.43 ± 4.53** | **20.8 ± 4.5** | **10.4 ± 2.9** | **11.2 ± 3.3** | **17.8 ± 3.5** |

*Source: Paper column from Semenova et al. 2024 Table 4; Old dispatch from `results/timing/x86/subscribe/bench_5rep_3clock/V2_02_medium_4thr_run1_{wall,cpu,thread}.txt`; Worker from `results/timing/x86/subscribe/bench_persistent_worker/V2_02_medium_4thr_run1_{wall,cpu,thread}.txt`; Serial from `results/timing/x86/serial/bench_persistent_worker/V2_02_medium_4thr_{wall,cpu,thread}.txt`*

¹ Semenova et al. 2024, Table 4 — subscribe mode, wall clock
² Paper combines SLAM Update + SLAM Delayed; our values shown combined for comparison

### Per-component comparison with paper: V2_02, 1 OpenCV thread (ms)

| Component | Paper¹ (sub, wall) | Old dispatch (sub, wall) | **Worker (sub, wall)** | Serial (wall) |
|-----------|-------------------|-------------------------|----------------------|--------------|
| Tracking | 8.55 ± 1.31 | 10.9 ± 2.5 | **4.0 ± 0.7** | 4.0 ± 0.8 |
| Propagation | 0.24 ± 0.03 | 0.4 ± 0.1 | **0.2 ± 0.0** | 0.2 ± 0.0 |
| MSCKF Update | 1.66 ± 2.11 | 2.1 ± 2.3 | **1.2 ± 1.4** | 1.2 ± 1.5 |
| SLAM Update² | 8.28 ± 4.79 | 6.2 ± 3.2 | **4.4 ± 2.3** | 4.6 ± 2.4 |
| Re-tri & Marg | 2.52 ± 0.21 | 1.7 ± 0.4 | **1.4 ± 0.2** | 1.5 ± 0.2 |
| **Total** | **21.25 ± 5.57** | **21.3 ± 4.2** | **11.2 ± 2.9** | **11.5 ± 3.1** |

*Source: Paper column from Semenova et al. 2024 Table 4; Old dispatch from `results/timing/x86/subscribe/bench_5rep_3clock/V2_02_medium_1thr_run*_wall.txt`; Worker from `results/timing/x86/subscribe/bench_persistent_worker/V2_02_medium_1thr_run*_wall.txt`; Serial from `results/timing/x86/serial/bench_persistent_worker/V2_02_medium_1thr_wall.txt`*

### Cross-run variability (subscribe, wall clock, 5 repetitions)

| Sequence | Config | Mean (ms) | Std (ms) | CV | Range (ms) |
|----------|--------|----------|---------|-----|-----------|
| V1_01_easy | 4-thr | 11.9 | 0.13 | **1.1%** | 11.8 – 12.1 |
| V1_01_easy | 1-thr | 12.7 | 0.04 | **0.3%** | 12.7 – 12.8 |
| MH_03_medium | 4-thr | 10.8 | 0.09 | **0.8%** | 10.7 – 10.9 |
| MH_03_medium | 1-thr | 11.5 | 0.08 | **0.7%** | 11.4 – 11.6 |
| V2_02_medium | 4-thr | 10.6 | 0.13 | **1.2%** | 10.4 – 10.7 |
| V2_02_medium | 1-thr | 11.2 | 0.05 | **0.5%** | 11.2 – 11.3 |

*Source: `results/timing/x86/subscribe/bench_persistent_worker/{V1_01_easy,MH_03_medium,V2_02_medium}_{1,4}thr_run{1..5}_wall.txt`*

All configurations have CV < 1.3%. The old dispatch had CV up to 5% on MH_03.

### SLAM feature health (avg features in state, max_slam=50)

| Sequence | Serial | Worker subscribe (5 reps) | Old dispatch subscribe (5 reps) |
|----------|--------|--------------------------|-------------------------------|
| V1_01_easy | 46.3 | 46.1, 46.1, 46.3, 46.1, 46.4 | 43.8 – 44.7 |
| MH_03_medium | 41.0 | 41.1, 41.2, 41.1, 41.3, 41.4 | **26.0** – 40.2 |
| V2_02_medium | 39.1 | 38.2, 38.3, 38.6, 38.7, 38.9 | 34.5 – 35.8 |

*Source: Serial from `results/timing/x86/serial/bench_persistent_worker/*_4thr_feats.txt`; Worker subscribe from `results/timing/x86/subscribe/bench_persistent_worker/*_4thr_run{1..5}_feats.txt`; Old dispatch from `results/timing/x86/subscribe/bench_5rep_3clock/*_4thr_run{1..5}_feats.txt`*

Subscribe SLAM health now matches serial within <1 feature. The old dispatch had
a worst case of 26.0 on MH_03 (partial SLAM dip).

### ATE — Absolute Trajectory Error (posyaw alignment)

**Position RMSE (meters) across all runs:**

| Sequence | Serial | Sub 4-thr run 1 | run 2 | run 3 | run 4 | run 5 | Sub 1-thr run 1 | run 2 | run 3 | run 4 | run 5 |
|----------|--------|-----------------|-------|-------|-------|-------|-----------------|-------|-------|-------|-------|
| V1_01_easy | 1.946 | 1.943 | 1.944 | 1.945 | 1.943 | 1.945 | 1.945 | 1.946 | 1.945 | 1.945 | 1.946 |
| MH_03_medium | 3.450 | 3.441 | 3.442 | 3.441 | 3.441 | 3.445 | 3.441 | 3.441 | 3.440 | 3.441 | 3.441 |
| V2_02_medium | 2.099 | 2.101 | 2.098 | 2.094 | ov_eval¹ | 2.102 | 2.091 | 2.091 | 2.092 | 2.102 | 2.097 |

¹ ov_eval NaN assertion crash on a healthy trajectory (tool bug, not a divergence —
final position is correct).

**Orientation RMSE (millidegrees) across all runs:**

| Sequence | Serial | Sub 4-thr (5 reps) | Sub 1-thr (5 reps) |
|----------|--------|-------------------|-------------------|
| V1_01_easy | 134.6 | 134.9, 134.9, 134.6, 134.8, 134.9 | 134.6, 134.9, 134.7, 135.5, 134.9 |
| MH_03_medium | 147.2 | 146.0, 148.9, 146.8, 145.3, 145.7 | 147.1, 146.5, 146.8, 145.9, 146.6 |
| V2_02_medium | 122.4 | 122.6, 122.9, 122.3, —¹, 122.5 | 123.1, 122.9, 122.9, 122.7, 122.7 |

**Cross-run variability summary:**

| Sequence | Serial ATE pos | Subscribe ATE pos range | Max deviation | Subscribe ATE pos std |
|----------|---------------|------------------------|---------------|----------------------|
| V1_01_easy | 1.946m | 1.943 – 1.946m | **0.003m** | **0.001m** |
| MH_03_medium | 3.450m | 3.440 – 3.445m | **0.010m** | **0.002m** |
| V2_02_medium | 2.099m | 2.091 – 2.102m | **0.008m** | **0.004m** |

### RPE — Relative Pose Error (median position, meters)

RPE measures local consistency over trajectory segments. Serial vs subscribe
run 1 comparison (4-thread):

**V1_01_easy:**

| Segment | Serial | Subscribe | Delta |
|---------|--------|-----------|-------|
| 8m | 3.222 | 3.202 | 0.020 |
| 16m | 2.832 | 2.826 | 0.006 |
| 24m | 2.874 | 2.873 | 0.001 |
| 32m | 2.458 | 2.434 | 0.024 |
| 40m | 2.378 | 2.363 | 0.015 |

**MH_03_medium:**

| Segment | Serial | Subscribe | Delta |
|---------|--------|-----------|-------|
| 8m | 5.675 | 5.663 | 0.012 |
| 16m | 3.531 | 3.534 | 0.003 |
| 24m | 4.884 | 4.893 | 0.009 |
| 32m | 5.362 | 5.391 | 0.029 |
| 40m | 3.442 | 3.438 | 0.004 |

**V2_02_medium:**

| Segment | Serial | Subscribe | Delta |
|---------|--------|-----------|-------|
| 8m | 2.861 | 2.870 | 0.009 |
| 16m | 3.234 | 3.230 | 0.004 |
| 24m | 2.710 | 2.682 | 0.028 |
| 32m | 3.215 | 3.171 | 0.044 |
| 40m | 3.032 | 3.029 | 0.003 |

RPE deltas are <0.05m across all segments and sequences — subscribe local
consistency matches serial.

*Source: Serial from `results/timing/x86/serial/bench_persistent_worker/*_4thr_est.txt`; Worker subscribe from `results/timing/x86/subscribe/bench_persistent_worker/*_4thr_run{1..5}_est.txt`; compared against `src/open_vins/ov_data/euroc_mav/{V1_01_easy,MH_03_medium,V2_02_medium}.txt`*

### Process CPU reveals OpenCV parallelism cost (V2_02, 4-thr)

| Mode | Wall | Proc CPU | Thread | CPU/Wall |
|------|------|----------|--------|----------|
| Serial | 11.2 | **17.8** | 11.1 | **1.59x** |
| Subscribe (worker) | 10.4 | **16.6** | 10.4 | **1.60x** |
| Subscribe (old dispatch) | 20.8 | **37.0** | 20.6 | **1.78x** |

*Source: Serial from `results/timing/x86/serial/bench_persistent_worker/V2_02_medium_4thr_{wall,cpu,thread}.txt`; Subscribe worker from `results/timing/x86/subscribe/bench_persistent_worker/V2_02_medium_4thr_run1_{wall,cpu,thread}.txt`; Subscribe old-dispatch from `results/timing/x86/subscribe/bench_5rep_3clock/V2_02_medium_4thr_run1_{wall,cpu,thread}.txt`*

CPU/Wall is now identical between serial and subscribe (1.59–1.60x) — purely the
OpenCV KLT thread pool. The old dispatch had 1.78x because executor threads burned
extra CPU during per-frame thread churn.

### RPi5 projections

| Sequence | x86 serial | x86 subscribe (worker) | RPi5 serial (×3.5) | Budget (20 Hz) |
|----------|-----------|----------------------|-------------------|---------------|
| V1_01_easy | 11.5ms | 11.9ms | ~40ms | 50ms |
| MH_03_medium | 11.5ms | 10.8ms | ~40ms | 50ms |
| V2_02_medium | 11.2ms | 10.6ms | ~39ms | 50ms |

*Source: x86 serial from `results/timing/x86/serial/bench_persistent_worker/*_4thr_wall.txt`; x86 subscribe (worker) from `results/timing/x86/subscribe/bench_persistent_worker/*_4thr_run{1..5}_wall.txt`; RPi5 column is a ×3.5 projection, not measured.*

Since subscribe now matches serial, the RPi5 projection for subscribe mode is the
same as serial: **~40ms, within the 50ms budget**. Previously, the 2x subscribe
overhead projected to ~74ms (over budget), requiring aggressive config optimization.
With the persistent worker, the default config may work on RPi5 without changes.
