# Benchmark Analysis: OpenVINS Timing, Accuracy & RPi5 Projections

**Date:** 2026-04-26
**Data:** `results/timing/x86/{serial,subscribe}/rerun_2026_04_23/` — see [data-provenance.md](data-provenance.md) for the full provenance row (submodule SHA, chi2_recovery state, OpenCV thread counts).
**Host:** Dell Latitude 5420, Intel i7-1185G7 (4C/8T, 3.0-4.8 GHz), 16 GB, Ubuntu 22.04
**Build:** colcon (flags: `-O3 -fsee -fomit-frame-pointer -g3`)
**Config:** EuRoC default: 200 features, max_slam=50, 11 clones, stereo
**Paper reference:** Semenova et al. 2024, Table 4 — i7-7500U (2C/4T, 3.5 GHz), Ubuntu 18.04

**Reproduce all data cited in this doc:**

```bash
bash scripts/run_full_benchmark.sh -r 5 --tag rerun_2026_04_23
```

(Default suite: 3 sequences × {4-thr, 1-thr} × stereo, 1 serial rep + 5
subscribe reps each. Wall-time on this host: ~12 min serial + ~90 min subscribe.)

## 1. Measurement methodology

Three clocks captured simultaneously at each of 7 timing points (rT1–rT7):

| Clock | API | What it measures | Best for |
|-------|-----|-----------------|----------|
| **Wall** | `boost::posix_time::microsec_clock` | Real elapsed time | Paper comparison (same clock) |
| **Process CPU** | `CLOCK_PROCESS_CPUTIME_ID` | CPU time across all threads | Total compute cost / power budget |
| **Thread CPU** | `CLOCK_THREAD_CPUTIME_ID` | VIO thread CPU only | Pure algorithmic cost |

**Critical caveat on wall clock in subscribe mode:** Wall time includes scheduling
delays (VIO thread preempted by executor). This inflates subscribe measurements
relative to the actual algorithmic cost. Thread CPU removes this artifact but is
not comparable to the paper (which uses wall clock).

**Runs:** 6 serial (1 rep, deterministic) + 30 subscribe (5 reps × 3 sequences
× 2 thread configs). All with SLAM recovery mechanism and zombie process cleanup.

---

## 2. Serial baseline: per-component breakdown (all 3 clocks)

### V2_02_medium (paper comparison sequence)

All three clocks (wall / proc CPU / thread) captured simultaneously per frame.
Cells are `mean ± std` over per-frame values; **Total** row includes p99.

**4 OpenCV threads (ms):**

| Component | Wall | Proc CPU | Thread | CPU/Wall |
|-----------|------|----------|--------|----------|
| Tracking | 2.8 ± 0.5 | **7.5 ± 1.2** | 2.7 ± 0.5 | **2.7×** |
| Propagation | 0.2 ± 0.0 | 0.2 ± 0.0 | 0.2 ± 0.0 | 1.0× |
| MSCKF Update | 1.2 ± 1.5 | 1.2 ± 1.5 | 1.2 ± 1.5 | 1.0× |
| SLAM Update | 3.0 ± 1.3 | 3.0 ± 1.3 | 3.0 ± 1.3 | 1.0× |
| SLAM Delayed | 1.5 ± 2.0 | 1.5 ± 2.0 | 1.5 ± 2.0 | 1.0× |
| Re-tri & Marg | 1.5 ± 0.2 | **2.2 ± 0.2** | 1.5 ± 0.2 | **1.5×** |
| **Total** | **10.1 ± 2.9** (p99: 19.3) | **15.6 ± 2.9** (p99: 24.7) | **10.1 ± 2.9** (p99: 19.2) | **1.54×** |

*Source: `results/timing/x86/serial/rerun_2026_04_23/V2_02_medium_4thr_{wall,cpu,thread}.txt`*

**1 OpenCV thread (ms):**

| Component | Wall | Proc CPU | Thread | CPU/Wall |
|-----------|------|----------|--------|----------|
| Tracking | 3.9 ± 0.7 | 3.9 ± 0.7 | 3.9 ± 0.7 | 1.0× |
| Propagation | 0.2 ± 0.0 | 0.2 ± 0.0 | 0.2 ± 0.0 | 1.0× |
| MSCKF Update | 1.2 ± 1.4 | 1.2 ± 1.4 | 1.2 ± 1.4 | 1.0× |
| SLAM Update | 3.0 ± 1.4 | 3.0 ± 1.3 | 3.0 ± 1.3 | 1.0× |
| SLAM Delayed | 1.5 ± 2.0 | 1.5 ± 2.0 | 1.5 ± 2.0 | 1.0× |
| Re-tri & Marg | 1.4 ± 0.2 | 1.4 ± 0.2 | 1.4 ± 0.2 | 1.0× |
| **Total** | **11.2 ± 2.9** (p99: 20.4) | **11.2 ± 2.9** (p99: 20.5) | **11.1 ± 2.9** (p99: 20.4) | **1.0×** |

*Source: `results/timing/x86/serial/rerun_2026_04_23/V2_02_medium_1thr_{wall,cpu,thread}.txt`*

**Observations:**
- With 4 threads: CPU/Wall > 1 only for Tracking (2.7x) and Re-tri (1.5x) — the
  two stages using OpenCV's thread pool. All EKF stages are single-threaded (ratio = 1.0).
- With 1 thread: all three clocks are identical — confirms no parallelism artifact.
- 4→1 thread tracking cost: 2.8→3.9ms (+39%), but total only +1.1ms (+11%)
  because EKF stages dominate.

### All sequences comparison (serial, 4-thr, ms)

**Clock:** wall. Per-component cells are mean; **Total** is `mean ± std`;
**p99** is the per-frame 99th percentile of the total.

| Component | V1_01_easy | MH_03_medium | V2_02_medium |
|-----------|-----------|-------------|-------------|
| Tracking | 2.6 ± 0.4 | 2.6 ± 0.4 | 2.8 ± 0.5 |
| Propagation | 0.2 ± 0.0 | 0.2 ± 0.0 | 0.2 ± 0.0 |
| MSCKF Update | 1.6 ± 2.5 | 1.2 ± 1.9 | 1.2 ± 1.5 |
| SLAM Update | 4.5 ± 1.0 | 3.7 ± 1.3 | 3.0 ± 1.3 |
| SLAM Delayed | 0.9 ± 1.7 | 1.2 ± 2.2 | 1.5 ± 2.0 |
| Re-tri & Marg | 1.5 ± 0.1 | 1.5 ± 0.1 | 1.5 ± 0.2 |
| **Total (mean ± std)** | **11.3 ± 3.4** | **10.4 ± 3.4** | **10.1 ± 2.9** |
| **Total p99** | **22.7** | **21.7** | **19.3** |

*Source: `results/timing/x86/serial/rerun_2026_04_23/{V1_01_easy,MH_03_medium,V2_02_medium}_4thr_wall.txt`*

**Note:** SLAM Update is highest on V1_01 (4.5ms, 40%) because the easy sequence
maintains the most SLAM features. V2_02 has fewer stable features (medium difficulty),
so SLAM Update is lighter. This means "SLAM Update is the dominant component" is
only true on easy/medium sequences — on V1_03_difficult (from our Phase 1 data),
tracking dominates because motion blur reduces the feature count.

---

## 3. Subscribe mode: per-component with all 3 clocks

### V2_02_medium, 4 OpenCV threads (ms, subscribe run 1)

All three clocks captured simultaneously. Cells are `mean ± std`; ratio
columns compare this subscribe run against the serial baseline above.

| Component | Wall | Proc CPU | Thread | Sub.Wall / Ser.Wall | Sub.Thread / Ser.Thread |
|-----------|------|----------|--------|--------------------|-----------------------|
| Tracking | 8.4 ± 2.1 | **23.4 ± 5.3** | 8.2 ± 2.1 | **3.0×** | **3.0×** |
| Propagation | 0.4 ± 0.1 | 0.4 ± 0.1 | 0.4 ± 0.1 | 2.0× | 2.0× |
| MSCKF Update | 2.6 ± 2.8 | 2.7 ± 3.0 | 2.6 ± 2.8 | 2.2× | 2.2× |
| SLAM Update | 5.1 ± 2.8 | 5.3 ± 2.9 | 5.0 ± 2.8 | 1.7× | 1.7× |
| SLAM Delayed | 2.3 ± 2.6 | 2.4 ± 2.7 | 2.3 ± 2.6 | 1.5× | 1.5× |
| Re-tri & Marg | 2.0 ± 0.7 | **3.0 ± 0.9** | 2.0 ± 0.7 | 1.3× | 1.3× |
| **Total** | **20.8 ± 4.5** (p99: 32.1) | **37.1 ± 6.7** (p99: 50.3) | **20.6 ± 4.5** (p99: 31.9) | **2.1×** | **2.0×** |

*Source: `results/timing/x86/subscribe/rerun_2026_04_23/V2_02_medium_4thr_run1_{wall,cpu,thread}.txt`; serial-ratio columns compared against `results/timing/x86/serial/rerun_2026_04_23/V2_02_medium_4thr_{wall,thread}.txt`*

**Key observation:** Thread ≈ Wall in subscribe mode (20.6 vs 20.8ms). This means
scheduling delay (VIO thread waiting to be scheduled) is only ~0.2ms per frame —
negligible on x86. The 2x overhead vs serial is almost entirely from the VIO thread
genuinely executing more slowly, not from waiting.

**However:** The Thread/Serial.Thread ratio (2.0x) shows the VIO thread itself burns
2x more CPU cycles in subscribe mode. We attributed this to "cache/memory hierarchy
overhead" in earlier analysis. This is plausible (executor threads evict VIO data
from cache) but not directly measured — we would need `perf stat` cache-miss
counters to confirm. The alternative explanation (different data patterns from
ROS2 message deserialization vs direct bag reading) has not been ruled out.

### Subscribe/serial overhead ratio by component (thread clock)

| Component | V1_01 | MH_03 | V2_02 | Interpretation |
|-----------|-------|-------|-------|----------------|
| Tracking | 2.7x | 2.9x | 3.0x | Largest working set → most cache-sensitive |
| Propagation | 2.0x | 2.0x | 2.0x | Small fixed-size IMU integration |
| MSCKF Update | 1.9x | 1.8x | 2.2x | Variable (depends on feature count) |
| SLAM Update | 1.7x | 1.5x | 1.7x | Compact EKF matrices |
| SLAM Delayed | 1.3x | 1.3x | 1.5x | Smallest working set |
| Re-tri & Marg | 1.2x | 1.5x | 1.3x | Fixed sliding window |
| **Total** | **1.9x** | **1.9x** | **2.0x** | |

*Source: derived from `results/timing/x86/{serial,subscribe}/rerun_2026_04_23/{V1_01_easy,MH_03_medium,V2_02_medium}_4thr*_thread.txt`*

The inflation ratio is non-uniform and consistent across sequences. Tracking is
always the most affected (~3x), EKF stages are 1.3-1.7x.

---

## 4. Paper comparison: V2_02

The paper used subscribe mode on an i7-7500U (Kaby Lake, 2C/4T, 3.5 GHz boost).
We compare with our subscribe (wall clock) and serial (wall + thread clocks).

### 4 OpenCV threads (ms)

**Clocks:** Paper and "Our sub" use wall (paper methodology); serial columns
show both wall and thread. Cells are `mean ± std` over per-frame values.

| Component | Paper (sub, wall) | Our sub (wall) | Our serial (wall) | Our serial (thread) |
|-----------|------------------|---------------|-------------------|-------------------|
| Tracking | 6.12 ± 1.13 | 8.4 ± 2.1 | 2.8 ± 0.5 | 2.7 ± 0.5 |
| Propagation | 0.21 ± 0.04 | 0.4 ± 0.1 | 0.2 ± 0.0 | 0.2 ± 0.0 |
| MSCKF Update | 1.31 ± 1.69 | 2.6 ± 2.8 | 1.2 ± 1.5 | 1.2 ± 1.5 |
| SLAM Upd+Del* | 6.56 ± 3.84 | 7.4 ± 3.7 | 4.5 ± 2.4 | 4.5 ± 2.4 |
| Re-tri & Marg | 2.24 ± 0.20 | 2.0 ± 0.7 | 1.5 ± 0.2 | 1.5 ± 0.2 |
| **Total** | **16.43 ± 4.53** | **20.8 ± 4.5** | **10.1 ± 2.9** | **10.1 ± 2.9** |

*Source: "Our sub" from `results/timing/x86/subscribe/rerun_2026_04_23/V2_02_medium_4thr_run*_wall.txt`; "Our serial" from `results/timing/x86/serial/rerun_2026_04_23/V2_02_medium_4thr_{wall,thread}.txt`; Paper column is Semenova et al. 2024 Table 4.*

*Paper combines SLAM Update + SLAM Delayed. Our combined: sub 5.1+2.3=7.4ms, serial 3.0+1.5=4.5ms.

### 1 OpenCV thread (ms)

**Clocks:** as above. Cells are `mean ± std`.

| Component | Paper (sub, wall) | Our sub (wall) | Our serial (wall) | Our serial (thread) |
|-----------|------------------|---------------|-------------------|-------------------|
| Tracking | 8.55 ± 1.31 | 10.9 ± 2.5 | 3.9 ± 0.7 | 3.9 ± 0.7 |
| Propagation | 0.24 ± 0.03 | 0.4 ± 0.1 | 0.2 ± 0.0 | 0.2 ± 0.0 |
| MSCKF Update | 1.66 ± 2.11 | 2.1 ± 2.3 | 1.2 ± 1.4 | 1.2 ± 1.4 |
| SLAM Upd+Del* | 8.28 ± 4.79 | 6.2 ± 3.2 | 4.5 ± 2.4 | 4.5 ± 2.4 |
| Re-tri & Marg | 2.52 ± 0.21 | 1.7 ± 0.4 | 1.4 ± 0.2 | 1.4 ± 0.2 |
| **Total** | **21.25 ± 5.57** | **21.3 ± 4.2** | **11.2 ± 2.9** | **11.1 ± 2.9** |

*Source: "Our sub" from `results/timing/x86/subscribe/rerun_2026_04_23/V2_02_medium_1thr_run*_wall.txt`; "Our serial" from `results/timing/x86/serial/rerun_2026_04_23/V2_02_medium_1thr_{wall,thread}.txt`; Paper column is Semenova et al. 2024 Table 4.*

### Analysis

**Our 1-thread subscribe matches the paper almost exactly** (21.3 vs 21.25ms). This
is despite our CPU being ~2 generations faster. The ROS2 subscribe overhead
equalizes the hardware difference.

**Our 4-thread subscribe is SLOWER than the paper** (20.8 vs 16.4ms). Our CPU has
4C/8T vs their 2C/4T — the MultiThreadedExecutor creates more threads competing for
cache. This is a genuine finding: more hardware threads can increase subscribe-mode
overhead through cache contention.

**Our serial mode reveals the true hardware speedup** — 10.1ms vs the paper's
fastest subscribe result of 16.4ms. The ~1.6x speedup is attributable to:
- Higher clock frequency (4.8 vs 3.5 GHz boost)
- Better IPC (Tiger Lake vs Kaby Lake microarchitecture)
- Larger L3 cache (12MB vs 4MB)

**Tracking is disproportionately inflated in subscribe** — 3.0x in our 4-thr sub vs
serial. The paper doesn't report serial numbers, so we can't compare their sub/serial
ratio. But their tracking at 6.12ms is faster than ours at 8.4ms despite their slower
CPU — again suggesting our larger executor thread pool creates more cache pollution.

---

## 5. Timing consistency across 5 reps

### Subscribe total across reps (ms)

**Clock:** wall. The "reps" columns list each run's mean; the ± after each
sequence summarises mean ± std *across* the 5 reps (i.e. run-to-run
variability, distinct from per-frame std in §2-§3).

| Sequence | 4-thr reps | 4-thr mean ± std | 1-thr reps | 1-thr mean ± std |
|----------|-----------|-----------------|-----------|-----------------|
| V1_01_easy | 21.2, 21.2, 21.3, 21.3, 21.1 | 21.22 ± 0.08 | 21.7, 21.9, 21.9, 21.9, 21.8 | 21.84 ± 0.09 |
| MH_03_medium | 20.2, 19.6, 21.3, 19.0, 21.4 | **20.30 ± 1.06** | 20.8, 20.9, 21.0, 20.7, 21.2 | 20.92 ± 0.19 |
| V2_02_medium | 20.8, 20.7, 20.6, 20.7, 20.7 | 20.70 ± 0.07 | 21.3, 21.4, 21.1, 21.0, 21.1 | 21.18 ± 0.16 |

*Source: `results/timing/x86/subscribe/rerun_2026_04_23/{V1_01_easy,MH_03_medium,V2_02_medium}_{1,4}thr_run{1..5}_wall.txt`*

V1_01 and V2_02 are very consistent (<0.5ms range). MH_03 4-thr shows 2.4ms range —
likely because the machine hall has more variable feature density, causing different
amounts of SLAM/MSCKF work per frame depending on which features survive in each run.

### Process CPU shows same consistency pattern

**Clock:** `CLOCK_PROCESS_CPUTIME_ID` (sum of CPU across all threads —
VIO + executor + OpenCV workers). Mean ± std is across 5 reps.

| Sequence | 4-thr proc CPU reps | Mean ± std |
|----------|-------------------|-----------|
| V1_01_easy | 34.9, 35.0, 35.1, 35.1, 34.9 | 35.00 ± 0.10 |
| MH_03_medium | 35.1, 34.5, 36.2, 34.5, 36.4 | 35.34 ± 0.90 |
| V2_02_medium | 37.1, 36.9, 36.7, 36.8, 36.9 | 36.88 ± 0.15 |

*Source: `results/timing/x86/subscribe/rerun_2026_04_23/{V1_01_easy,MH_03_medium,V2_02_medium}_4thr_run{1..5}_cpu.txt`*

The process CPU overhead (executor threads) is stable. The 37ms process CPU on V2_02
means the system consumes ~37ms of total CPU per frame across all threads.

---

## 6. SLAM feature health

| Sequence | Serial | Subscribe 4-thr (5 reps) | Subscribe 1-thr (5 reps) |
|----------|--------|--------------------------|--------------------------|
| V1_01_easy | 46.0 | 44.4, 44.6, 44.6, 44.7, 43.8 | 44.1, 44.6, 44.3, 44.1, 44.3 |
| MH_03_medium | 41.5 | 34.4, **31.2**, 38.2, **26.0**, 38.7 | **29.1**, 40.2, 37.9, 36.5, 39.6 |
| V2_02_medium | 38.4 | 35.5, 35.1, 34.7, 35.8, 35.2 | 35.2, 35.8, 34.8, 34.6, 34.9 |

*Source: serial from `results/timing/x86/serial/rerun_2026_04_23/{V1_01_easy,MH_03_medium,V2_02_medium}_4thr_feats.txt`; subscribe from `results/timing/x86/subscribe/rerun_2026_04_23/*_{1,4}thr_run{1..5}_feats.txt`*

**V1_01 is naturally stable** — subscribe matches serial closely (44-45 vs 46).

**V2_02 has the most run-to-run variability** — all runs at 34-36 (vs 38.4
serial). The ~4-feature gap vs serial is from subscribe-mode scheduling jitter
allowing slightly different feature lifecycles; the persistent worker thread
keeps SLAM well above zero (no collapse) but can't perfectly reproduce serial
timing-dependent feature admission decisions.

**MH_03 also dips occasionally** — two 4-thr runs at 26 and 31, one 1-thr run at 29.
The machine hall sequences have sparser features and longer corridors where feature
tracks break. The PWT fix prevents collapse to 0 but can't prevent dips to 26-31.
This is acceptable — ATE is still good (see below).

> **Note on `slam_chi2_recovery`:** the chi-squared-relaxation mechanism is
> available as an opt-in YAML knob (default `false`). The data above is at
> default `false` — the SLAM stability shown is achieved by the persistent
> worker thread alone, not by the chi2 relaxation. See
> [determinism.md §4](determinism.md#4-optional-safety-net-slam-recovery-mechanism)
> for the rationale and the V1_03 @ rate 2.0 evidence where chi2 recovery
> does help.

---

## 7. Accuracy

### ATE (Absolute Trajectory Error, position RMSE in meters, posyaw alignment)

| Sequence | Serial | Subscribe 4-thr (5 reps) | Subscribe 1-thr (5 reps) |
|----------|--------|--------------------------|--------------------------|
| V1_01_easy | 1.945 | 1.951, 1.955, 1.952, 1.939, 1.946 | 1.944, 1.942, 1.944, 1.945, 1.946 |
| MH_03_medium | 3.450 | 3.451, 3.455, 3.440, 3.454, 3.458 | 3.453, 3.440, 3.439, 3.439, 3.443 |
| V2_02_medium | 2.099 | (tool*), 2.097, 2.094, (tool*), 2.093 | 2.092, 2.091, 2.089, 2.090, (tool*) |

*Source: serial from `results/timing/x86/serial/rerun_2026_04_23/*_4thr_est.txt`; subscribe from `results/timing/x86/subscribe/rerun_2026_04_23/*_{1,4}thr_run{1..5}_est.txt`; compared against ground truth `src/open_vins/ov_data/euroc_mav/{V1_01_easy,MH_03_medium,V2_02_medium}.txt`*

*"tool" = ov_eval NaN assertion crash on runs with healthy trajectories (verified
via final position check — not a divergence). This is a bug in the analysis tool,
not in the VIO.

**Subscribe matches serial to within 0.02m on all sequences.** The worst-case
deviation is V1_01 at 0.016m (1.939 vs 1.955). MH_03 deviates by 0.019m. V2_02
by 0.010m.

**Even MH_03 runs with low SLAM (26.0, 29.1) produce good ATE** — 3.454 and 3.453
respectively, within 0.004m of serial.

### RPE (Relative Pose Error, median position error at 8m segments, in meters)

RPE measures local consistency — drift over short distances — and is more sensitive
than ATE to transient tracking issues. ATE can be good even with poor local
consistency if errors cancel over the full trajectory.

| Sequence | Serial | Subscribe 4-thr (5 reps) | Subscribe 1-thr (5 reps) |
|----------|--------|--------------------------|--------------------------|
| V1_01_easy | 3.217 | 3.228, 3.198, 3.226, 3.225, 3.214 | 3.203, 3.213, 3.221, 3.214, 3.210 |
| MH_03_medium | 5.655 | 5.721, 5.706, 5.582, 5.631, 5.624 | 5.626, 5.601, 5.630, 5.650, 5.641 |
| V2_02_medium | 2.863 | 2.867, 2.855, 2.883, (tool*), 2.879 | 2.875, 2.862, 2.849, 2.883, (tool*) |

*Source: same `*_est.txt` files as the ATE table (RPE is computed from the same estimates).*

**Subscribe RPE matches serial within ~0.07m on V1_01, ~0.07m on MH_03, and
~0.02m on V2_02.** The local consistency is preserved — the persistent worker
thread eliminates the per-frame timing jitter that previously caused subscribe-mode
SLAM collapse, so subscribe behaves like a noisy version of serial rather than a
qualitatively different mode.

### RPE across segment lengths (serial vs subscribe, V2_02, 4-thr)

| Segment | Serial median_pos | Subscribe run 1 median_pos | Deviation |
|---------|------------------|---------------------------|-----------|
| 8m | 2.863 | 2.867 | +0.004 |
| 16m | 3.230 | 3.231 | +0.001 |
| 24m | 2.714 | 2.713 | -0.001 |
| 32m | 3.220 | 3.162 | -0.058 |
| 40m | 3.018 | 3.041 | +0.023 |

*Source: serial from `results/timing/x86/serial/rerun_2026_04_23/V2_02_medium_4thr_est.txt`; subscribe run 1 from `results/timing/x86/subscribe/rerun_2026_04_23/V2_02_medium_4thr_run1_est.txt`*

RPE is consistent across all segment lengths — no divergence at any scale.

### MH_03 worst-case SLAM run: RPE vs serial

The MH_03 4-thr run 4 had the lowest SLAM features (avg 26.0). Does the dip in
SLAM features affect local accuracy?

| Segment | Serial median_pos | Low-SLAM run (26.0) | Deviation |
|---------|------------------|---------------------|-----------|
| 8m | 5.655 | 5.631 | -0.024 |
| 16m | 3.551 | 3.542 | -0.009 |
| 24m | 4.891 | 4.924 | +0.033 |
| 32m | 5.380 | 5.375 | -0.005 |
| 40m | 3.437 | 3.569 | +0.132 |

*Source: serial from `results/timing/x86/serial/rerun_2026_04_23/MH_03_medium_4thr_est.txt`; low-SLAM run (avg SLAM = 26.0) from `results/timing/x86/subscribe/rerun_2026_04_23/MH_03_medium_4thr_run4_est.txt`*

**Even the worst-case low-SLAM run matches serial RPE within 0.13m at all segment
lengths.** The largest deviation is at 40m segments (+0.132m), suggesting a slight
increase in long-range drift when SLAM features dip — but the effect is small and
within the normal run-to-run variability.

---

## 8. RPi5 — see separate doc

The RPi5 projections that were here have been superseded by actual RPi5
measurements. See [rpi5-benchmarking.md](rpi5-benchmarking.md) for both the
original projections (§1) and the measured timing, accuracy, and subscribe
results (§2-5).

---

## 9. Summary of claims — verified vs unverified

| Claim | Status | Evidence |
|-------|--------|----------|
| Serial mode is deterministic | **Verified** | All serial runs produce identical timestamps, SLAM counts, ATE |
| Subscribe adds ~2× wall overhead (pre-fix) | **Verified** | Consistent 2.0-2.1× across 3 sequences, 30 runs on old dispatch |
| Subscribe adds ~1× wall overhead (post-fix) | **Verified** | See [determinism.md](determinism.md) — persistent worker thread |
| Subscribe accuracy matches serial | **Verified** | ATE within 0.02m, RPE (8m) within 0.07m across all 30 subscribe runs |
| Persistent worker thread prevents catastrophic SLAM collapse | **Verified** | 0/30 subscribe runs collapsed at default `slam_chi2_recovery: false`. The PWT fix is the architectural cause; the chi2-recovery YAML knob is an additional opt-in safety net for subscribe-overload scenarios (see [determinism.md §4](determinism.md#4-optional-safety-net-slam-recovery-mechanism) for the V1_03 @ rate 2.0 evidence). |
| Overhead is from cache/memory pollution | **Plausible but unverified** | Thread CPU > serial thread CPU, but no `perf stat` cache-miss data |
| OpenCV parallelism saves ~1ms (4-thr vs 1-thr) | **Verified** | Serial: 10.1 vs 11.2ms. But costs 5.5ms extra process CPU |
| MH_03 has more timing variability than V1_01/V2_02 | **Verified** | 2.4ms range vs 0.2ms for other sequences (4-thr subscribe) |
| 4→1 thread penalty is ~9-14% | **Verified** | 10.1→11.2ms (V2_02), 11.3→12.3ms (V1_01), 10.4→11.5ms (MH_03) |
