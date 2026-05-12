# Cross-Platform Performance & Accuracy

How does the same OpenVINS build behave on four target platforms? Runtime per
VIO stage, end-to-end ATE/RPE, and the OS/distro effect isolated where the
hardware is held constant.

All numbers below come from a single benchmark invocation per platform:

```bash
bash scripts/run_full_benchmark.sh -r 5 --tag run_for_benchmark_analysis_11_05
```

→ serial + subscribe, V1_01_easy / MH_03_medium / V2_02_medium, stereo, 4 and
1 OpenCV threads, 5 subscribe reps. Per-stage numbers come from
`parse_results.py --detailed` on each platform's `serial/` and `subscribe/`
directory.

Status: **x86-J filled. x86-H, RPi5-U, RPi5-T pending** — placeholder cells
shown as `—` until the corresponding platform's benchmark + `--detailed`
output is pasted in. Findings (§9) is written once 3+ platforms are present.

Reference platform for all ratio columns: **x86-J** (x86 Ubuntu 24.04 / Jazzy).

## 1. Setup matrix

| Field | x86-J | x86-H | RPi5-U | RPi5-T |
|---|---|---|---|---|
| CPU | Intel i7-11850H (8C/16T, 4.8 GHz) | — | Cortex-A76 ×4 @ 2.4 GHz | Cortex-A76 ×4 @ 2.4 GHz |
| Arch | x86_64 | x86_64 | aarch64 | aarch64 |
| OS | Ubuntu 24.04 Noble | Ubuntu 22.04 Jammy | Ubuntu (RPi5) | Debian 13 Trixie |
| ROS distro | jazzy | humble | jazzy or humble | humble (bench_lib default) |
| `<arch>/<env>` path | `x86/native_jazzy` | `x86/native_humble` | `rpi5/native_<distro>` | `rpi5/native_humble` |

*Common provenance for every table below:* submodule SHA `a7781f4`,
`slam_chi2_recovery: false`, `num_pts: 200`, `max_slam: 50`, `max_clones: 11`,
`num_opencv_threads: 4` (or 1 where noted), `multi_threading_subs: true`,
stereo (`max_cameras: 2`, `use_stereo: true`).

Per-table citations are abbreviated to the directory form
`results/<arch>/<env>/run_for_benchmark_analysis_11_05/{serial,subscribe}/`.

## 2. Headline: realtime + accuracy snapshot

Canonical operating point for everything in this section: **subscribe mode,
stereo, 4 threads, 5 reps aggregated, wall clock**.

### 2.1 Summary across sequences — the single at-a-glance table

One row per platform. Cells are `wall ms / ATE pos rmse m` (mean per
sequence). The **Mean** column is the unweighted average over the three
sequences — a single ranking number per platform.

| Platform | V1_01_easy<br>wall ms / ATE m | MH_03_medium<br>wall ms / ATE m | V2_02_medium<br>wall ms / ATE m | Mean<br>wall ms / ATE m | vs x86-J<br>(wall) |
|---|---|---|---|---|---|
| x86-J | 14.4 / 0.091 | 12.2 / 0.172 | 10.6 / 0.060 | 12.4 / 0.108 | 1.00× |
| x86-H | — / — | — / — | — / — | — / — | — |
| RPi5-U | — / — | — / — | — / — | — / — | — |
| RPi5-T | — / — | — / — | — / — | — / — | — |

*Source: results/<arch>/<env>/run_for_benchmark_analysis_11_05/subscribe/{V1_01_easy,MH_03_medium,V2_02_medium}_4thr_{wall,est}.txt*

### 2.2 Per-sequence detail — with std and p99

Same data as §2.1 but expanded with std and p99 so the variance of each
cell is visible.

| Platform | Sequence | Total wall ms (mean ± std, p99) | ATE pos rmse (m, mean ± std) | vs x86-J |
|---|---|---|---|---|
| x86-J | V1_01_easy | 14.4 ± 4.4 (p99: 29.0) | 0.091 ± 0.022 | 1.00× |
| x86-J | MH_03_medium | 12.2 ± 4.1 (p99: 26.4) | 0.172 ± 0.039 | 1.00× |
| x86-J | V2_02_medium | 10.6 ± 3.3 (p99: 20.5) | 0.060 ± 0.003 | 1.00× |
| x86-H | V1_01_easy | — | — | — |
| x86-H | MH_03_medium | — | — | — |
| x86-H | V2_02_medium | — | — | — |
| RPi5-U | V1_01_easy | — | — | — |
| RPi5-U | MH_03_medium | — | — | — |
| RPi5-U | V2_02_medium | — | — | — |
| RPi5-T | V1_01_easy | — | — | — |
| RPi5-T | MH_03_medium | — | — | — |
| RPi5-T | V2_02_medium | — | — | — |

*Source: same as §2.1.*

## 3. Per-sequence totals

Same metric (total wall ms) but with **serial vs subscribe** side by side per
sequence, stereo, 4 threads. The Sub/Serial ratio in serial mode is `—` (it's
the same number) — the column compares platforms instead.

### 3.1 V1_01_easy

| Mode | x86-J | x86-H | RPi5-U | RPi5-T | RPi5-T vs x86-J |
|---|---|---|---|---|---|
| Serial total | 10.3 ± 3.5 | — | — | — | — |
| Subscribe total | 14.4 ± 4.4 | — | — | — | — |
| Serial p99 | 22.1 | — | — | — | — |
| Subscribe p99 | 29.0 | — | — | — | — |

*Source: results/<arch>/<env>/run_for_benchmark_analysis_11_05/{serial,subscribe}/V1_01_easy_4thr_wall.txt*

### 3.2 MH_03_medium

| Mode | x86-J | x86-H | RPi5-U | RPi5-T | RPi5-T vs x86-J |
|---|---|---|---|---|---|
| Serial total | 14.5 ± 4.8 | — | — | — | — |
| Subscribe total | 12.2 ± 4.1 | — | — | — | — |
| Serial p99 | 30.9 | — | — | — | — |
| Subscribe p99 | 26.4 | — | — | — | — |

*Source: results/<arch>/<env>/run_for_benchmark_analysis_11_05/{serial,subscribe}/MH_03_medium_4thr_wall.txt*

### 3.3 V2_02_medium

| Mode | x86-J | x86-H | RPi5-U | RPi5-T | RPi5-T vs x86-J |
|---|---|---|---|---|---|
| Serial total | 14.0 ± 3.8 | — | — | — | — |
| Subscribe total | 10.6 ± 3.3 | — | — | — | — |
| Serial p99 | 25.4 | — | — | — | — |
| Subscribe p99 | 20.5 | — | — | — | — |

*Source: results/<arch>/<env>/run_for_benchmark_analysis_11_05/{serial,subscribe}/V2_02_medium_4thr_wall.txt*

> Note on serial ≥ subscribe on x86-J for V1_01_easy: serial is the
> 1-run-pooled-frames number from a single deterministic pass; subscribe is
> the across-5-reps frame-pooled mean. The serial pass is slightly faster
> here in 4-thread mode (fewer pipeline stalls, no IMU queueing overhead).
> On MH/V2 the order flips — subscribe is lower because the bag-relative
> rate gives the worker more idle headroom between frames.

## 4. Per-stage breakdown (V1_01_easy)

V1_01_easy chosen as the canonical sequence (others available in raw CSVs and
parser output — see §10). Cells are `mean ± std` ms.

### 4.1 Stage means — subscribe, 4 threads, wall

| Stage | x86-J | x86-H | RPi5-U | RPi5-T |
|---|---|---|---|---|
| Tracking | 3.9 ± 0.7 | — | — | — |
| Propagation | 0.2 ± 0.0 | — | — | — |
| MSCKF Update | 2.0 ± 3.1 | — | — | — |
| SLAM Update | 5.3 ± 1.5 | — | — | — |
| SLAM Delayed | 1.2 ± 2.2 | — | — | — |
| Re-tri & Marg | 1.8 ± 0.2 | — | — | — |
| **Total** | **14.4 ± 4.4** | **—** | **—** | **—** |

*Source: results/<arch>/<env>/run_for_benchmark_analysis_11_05/subscribe/V1_01_easy_4thr_wall.txt*

### 4.2 Per-stage slowdown vs x86-J

The "which step suffers most where" table. Each cell is the platform's mean /
x86-J mean for the same stage. Same slice as §4.1.

| Stage | x86-H / x86-J | RPi5-U / x86-J | RPi5-T / x86-J | RPi5-T / x86-H |
|---|---|---|---|---|
| Tracking | — | — | — | — |
| Propagation | — | — | — | — |
| MSCKF Update | — | — | — | — |
| SLAM Update | — | — | — | — |
| SLAM Delayed | — | — | — | — |
| Re-tri & Marg | — | — | — | — |
| **Total** | **—** | **—** | **—** | **—** |

*Likely-cause prose section will live here once data is in — see
[rpi5-benchmarking.md](../rpi5-benchmarking.md#per-component-slowdown) for the
style: assign each outlier ratio to NEON width vs AVX2, cache size, Eigen/glibc
version, or BLAS implementation differences.*

### 4.3 Single-thread sensitivity — subscribe, 1 thread, wall

Same shape as §4.1; lets us see whether platforms diverge more under thread
pressure.

| Stage | x86-J | x86-H | RPi5-U | RPi5-T |
|---|---|---|---|---|
| Tracking | 4.5 ± 0.7 | — | — | — |
| Propagation | 0.2 ± 0.0 | — | — | — |
| MSCKF Update | 1.8 ± 2.8 | — | — | — |
| SLAM Update | 5.0 ± 1.4 | — | — | — |
| SLAM Delayed | 1.1 ± 2.1 | — | — | — |
| Re-tri & Marg | 1.6 ± 0.2 | — | — | — |
| **Total** | **14.1 ± 4.1** | **—** | **—** | **—** |

*Source: results/<arch>/<env>/run_for_benchmark_analysis_11_05/subscribe/V1_01_easy_1thr_wall.txt*

### 4.4 Wall vs CPU vs thread — subscribe, 4 threads

Three sub-tables (one per clock) showing the same stages. The Wall sub-table
is the same data as §4.1, repeated here for direct comparison. Pattern mirrors
[benchmark-analysis.md](../benchmark-analysis.md) §"3-clock comparison".

**Wall clock** (repeat of §4.1):

| Stage | x86-J | x86-H | RPi5-U | RPi5-T |
|---|---|---|---|---|
| Tracking | 3.9 ± 0.7 | — | — | — |
| Propagation | 0.2 ± 0.0 | — | — | — |
| MSCKF Update | 2.0 ± 3.1 | — | — | — |
| SLAM Update | 5.3 ± 1.5 | — | — | — |
| SLAM Delayed | 1.2 ± 2.2 | — | — | — |
| Re-tri & Marg | 1.8 ± 0.2 | — | — | — |
| **Total** | **14.4 ± 4.4** | **—** | **—** | **—** |

**Process CPU clock**:

| Stage | x86-J | x86-H | RPi5-U | RPi5-T |
|---|---|---|---|---|
| Tracking | 12.6 ± 1.8 | — | — | — |
| Propagation | 0.2 ± 0.1 | — | — | — |
| MSCKF Update | 2.0 ± 3.1 | — | — | — |
| SLAM Update | 5.3 ± 1.6 | — | — | — |
| SLAM Delayed | 1.2 ± 2.2 | — | — | — |
| Re-tri & Marg | 2.7 ± 0.3 | — | — | — |
| **Total** | **24.0 ± 4.8** | **—** | **—** | **—** |

**Thread CPU clock** (calling thread only):

| Stage | x86-J | x86-H | RPi5-U | RPi5-T |
|---|---|---|---|---|
| Tracking | 3.9 ± 0.7 | — | — | — |
| Propagation | 0.2 ± 0.0 | — | — | — |
| MSCKF Update | 2.0 ± 3.1 | — | — | — |
| SLAM Update | 5.3 ± 1.5 | — | — | — |
| SLAM Delayed | 1.2 ± 2.2 | — | — | — |
| Re-tri & Marg | 1.8 ± 0.2 | — | — | — |
| **Total** | **14.4 ± 4.4** | **—** | **—** | **—** |

**CPU/Wall ratio** — proxy for multithreading effectiveness. Ratio = 1.0
means the stage is single-threaded; ratio > 1.0 means CPU time pooled across
threads exceeds wall time, i.e. the stage parallelizes.

| Stage | x86-J | x86-H | RPi5-U | RPi5-T |
|---|---|---|---|---|
| Tracking | 3.23× | — | — | — |
| Propagation | 1.00× | — | — | — |
| MSCKF Update | 1.00× | — | — | — |
| SLAM Update | 1.00× | — | — | — |
| SLAM Delayed | 1.00× | — | — | — |
| Re-tri & Marg | 1.50× | — | — | — |
| **Total** | **1.67×** | **—** | **—** | **—** |

On x86-J the tracking stage uses ~3.2 cores worth of CPU per frame
(`num_opencv_threads: 4` splitting KLT pyramids across threads), while EKF
update stages stay strictly serial (CPU ≈ wall). Once RPi5 numbers land,
compare these ratios — if the RPi5 tracking ratio is closer to 1.0× than
3.2× the four-core NEON parallelism isn't paying off and tracking is the
first stage to look at when tuning.

*Source: results/<arch>/<env>/run_for_benchmark_analysis_11_05/subscribe/V1_01_easy_4thr_{wall,cpu,thread}.txt*

## 5. Accuracy

### 5.1 ATE per sequence

Subscribe mode, stereo, 4 threads, 5 reps aggregated.

| Sequence | Metric | x86-J | x86-H | RPi5-U | RPi5-T |
|---|---|---|---|---|---|
| V1_01_easy | pos rmse (m) | 0.091 ± 0.022 (range: 59 mm) | — | — | — |
| V1_01_easy | ori rmse (°) | 0.82 | — | — | — |
| MH_03_medium | pos rmse (m) | 0.172 ± 0.039 (range: 95 mm) | — | — | — |
| MH_03_medium | ori rmse (°) | 1.06 | — | — | — |
| V2_02_medium | pos rmse (m) | 0.060 ± 0.003 (range: 8 mm) | — | — | — |
| V2_02_medium | ori rmse (°) | 1.26 | — | — | — |

*Source: results/<arch>/<env>/run_for_benchmark_analysis_11_05/subscribe/{seq}_4thr_run{1..5}_est.txt vs ov_data/euroc_mav/{seq}.txt*

### 5.2 RPE per segment (median)

Subscribe mode, stereo, 4 threads, segments 8/16/24/32/40 s. Cell format
`ori° / pos m`.

| Sequence | Segment | x86-J | x86-H | RPi5-U | RPi5-T |
|---|---|---|---|---|---|
| V1_01_easy | 8 s | 0.73 / 0.099 | — | — | — |
| V1_01_easy | 16 s | 0.75 / 0.097 | — | — | — |
| V1_01_easy | 24 s | 0.81 / 0.128 | — | — | — |
| V1_01_easy | 32 s | 0.85 / 0.129 | — | — | — |
| V1_01_easy | 40 s | 0.73 / 0.146 | — | — | — |
| MH_03_medium | 8 s | 0.49 / 0.152 | — | — | — |
| MH_03_medium | 16 s | 0.65 / 0.170 | — | — | — |
| MH_03_medium | 24 s | 0.76 / 0.187 | — | — | — |
| MH_03_medium | 32 s | 0.90 / 0.206 | — | — | — |
| MH_03_medium | 40 s | 0.96 / 0.249 | — | — | — |
| V2_02_medium | 8 s | 1.20 / 0.054 | — | — | — |
| V2_02_medium | 16 s | 1.21 / 0.071 | — | — | — |
| V2_02_medium | 24 s | 1.44 / 0.074 | — | — | — |
| V2_02_medium | 32 s | 1.57 / 0.085 | — | — | — |
| V2_02_medium | 40 s | 1.56 / 0.091 | — | — | — |

*Source: same as §5.1; RPE computed by `ov_eval/error_singlerun` with custom segment lengths.*

## 6. Subscribe overhead

Subscribe mean / Serial mean ratio per platform per sequence. > 1.0 means
the persistent-worker dispatch adds cost vs the deterministic serial reader.
Frame counts are listed for sanity — if any sub run drops frames at 1× rate,
flag it in Findings (§9).

| Platform | Sequence | Serial wall (mean) | Subscribe wall (mean) | Sub/Serial | Frames (sub mean) |
|---|---|---|---|---|---|
| x86-J | V1_01_easy | 10.3 | 14.4 | 1.40× | 2800 |
| x86-J | MH_03_medium | 14.5 | 12.2 | 0.84× | 2311 |
| x86-J | V2_02_medium | 14.0 | 10.6 | 0.76× | 2267 |
| x86-H | V1_01_easy | — | — | — | — |
| x86-H | MH_03_medium | — | — | — | — |
| x86-H | V2_02_medium | — | — | — | — |
| RPi5-U | V1_01_easy | — | — | — | — |
| RPi5-U | MH_03_medium | — | — | — | — |
| RPi5-U | V2_02_medium | — | — | — | — |
| RPi5-T | V1_01_easy | — | — | — | — |
| RPi5-T | MH_03_medium | — | — | — | — |
| RPi5-T | V2_02_medium | — | — | — | — |

*Source: results/<arch>/<env>/run_for_benchmark_analysis_11_05/{serial,subscribe}/{seq}_4thr_wall.txt*

## 7. SLAM feature health

Mean active SLAM features in the state (target ≈ `max_slam: 50`). Subscribe
mode, stereo, 4 threads. Cell = `mean (range across 5 reps)`.

| Platform | V1_01_easy | MH_03_medium | V2_02_medium |
|---|---|---|---|
| x86-J | 45.3 (45.1–45.6) | 41.0 (40.5–41.6) | 37.9 (37.5–38.3) |
| x86-H | — | — | — |
| RPi5-U | — | — | — |
| RPi5-T | — | — | — |

*Source: results/<arch>/<env>/run_for_benchmark_analysis_11_05/subscribe/{seq}_4thr_run{1..5}_feats.txt*

## 8. OS-version isolation

Holding hardware constant and varying the distro/ROS version. Each table is
4 rows for one hardware platform, subscribe / 4 threads / V1_01_easy.

### 8.1 x86 Ubuntu 24/Jazzy vs Ubuntu 22/Humble

| Metric | x86-J (Noble / Jazzy) | x86-H (Jammy / Humble) | Δ |
|---|---|---|---|
| Total wall ms | 14.4 ± 4.4 | — | — |
| ATE pos rmse (m) | 0.091 | — | — |
| Biggest-delta stage | _pending_ | _pending_ | _pending_ |
| Second biggest-delta stage | _pending_ | _pending_ | _pending_ |

### 8.2 RPi5 Ubuntu vs Debian Trixie

| Metric | RPi5-U | RPi5-T | Δ |
|---|---|---|---|
| Total wall ms | — | — | — |
| ATE pos rmse (m) | — | — | — |
| Biggest-delta stage | _pending_ | _pending_ | _pending_ |
| Second biggest-delta stage | _pending_ | _pending_ | _pending_ |

## 9. Findings

_Written once 3+ platforms have data. The expected outline:_

- Headline ranking: which platform is fastest / most accurate?
- Which VIO stage suffers most going from x86 to ARM, and likely cause?
- ROS/OS version effect (§8) — is humble vs jazzy noise-level or material?
- Subscribe-vs-serial overhead — does the persistent-worker hold the 1× ratio across all four platforms (vs the upstream 2×)?
- Real-time feasibility per platform (p99 vs 20 Hz / 30 Hz frame budgets)
- Any anomalies (frame drops, ATE divergence, missing data points worth re-running)

## 10. Raw data index

Per-platform result trees (each holds `serial/` and `subscribe/`
sub-directories with the standard
`{SEQ}_{N}thr[_run{N}]_{wall,cpu,thread,feats,est}.txt` files):

| Platform | Path |
|---|---|
| x86-J | `~/results/x86/native_jazzy/run_for_benchmark_analysis_11_05/` |
| x86-H | `~/results/x86/native_humble/run_for_benchmark_analysis_11_05/` (pending) |
| RPi5-U | `~/results/rpi5/native_<distro>/run_for_benchmark_analysis_11_05/` (pending) |
| RPi5-T | `~/results/rpi5/native_humble/run_for_benchmark_analysis_11_05/` (pending) |

To regenerate the per-component / ATE / RPE numbers cited above on any
platform:

```bash
python3 scripts/parse_results.py \
  ~/results/<arch>/<env>/run_for_benchmark_analysis_11_05/serial --detailed
python3 scripts/parse_results.py \
  ~/results/<arch>/<env>/run_for_benchmark_analysis_11_05/subscribe --detailed
```

That single command per directory covers every cell in §§2–7 for one
platform's column.
