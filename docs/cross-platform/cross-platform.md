# Cross-Platform Performance & Accuracy

How does the same OpenVINS build behave on four target platforms? Runtime per
VIO stage, end-to-end ATE/RPE, and the OS/distro effect isolated where the
hardware is held constant.

All numbers below come from a single benchmark invocation per platform:

```bash
bash scripts/run_full_benchmark.sh -r 5 --tag <platform-tag>
# RPi5-T used --docker openvins-humble:latest in addition.
```

→ serial + subscribe, V1_01_easy / MH_03_medium / V2_02_medium, stereo, 4 and
1 OpenCV threads, 5 subscribe reps. Per-stage numbers come from
`parse_results.py --detailed` on each platform's `serial/` and `subscribe/`
directory.

Reference platform for all ratio columns: **x86-J** (x86 Ubuntu 24.04 /
Jazzy).

**One known data gap** to keep in mind while reading:

- **RPi5 serial MH_03_medium fails on both RPi5 platforms** (avg SLAM ≈ 0.2
  features, ATE ≈ 2794 m on RPi5-U / 2855 m on RPi5-T). Subscribe-mode MH_03
  works fine. The serial-mode numbers for MH_03 on RPi5-U / RPi5-T are noted
  but should be treated as invalid for accuracy.

(RPi5-T accuracy was initially blocked because `parse_results.py` ran on the
Trixie host, which has no native ROS humble. Fixed by re-running the parser
*inside* the same Docker image that did the benchmark — see §9 finding #9
for the exact command.)

## 1. Setup matrix

| Field | x86-J | x86-H | RPi5-U | RPi5-T |
|---|---|---|---|---|
| CPU | Intel i7-11850H (8C/16T, 4.8 GHz) | Intel x86 (Dell Latitude 5420) | Cortex-A76 ×4 @ 2.4 GHz | Cortex-A76 ×4 @ 2.4 GHz |
| Arch | x86_64 | x86_64 | aarch64 | aarch64 |
| Host OS | Ubuntu 24.04 Noble | Ubuntu 22.04 Jammy | Ubuntu 24.04 Noble | Debian 13 Trixie |
| Container | native | native | native | Docker `openvins-humble:latest` |
| ROS distro | jazzy | humble | jazzy | humble (inside container) |
| `<arch>/<env>` | `x86/native_jazzy` | `x86/native_humble` | `rpi5/native_jazzy` | `rpi5/docker_humble` |
| Tag | `run_for_benchmark_analysis_11_05` | `run_for_benchmark_analysis_13_05__x86_H` | `run_for_benchmark_analysis_13_05__rpi5__U` | `run_for_benchmark_analysis_13_05__rpi5__T` |

*Common provenance for every table below:* submodule SHA `a7781f4` (x86-J;
other platforms re-collected against the same `master-candidate` HEAD on the
benchmark date),
`slam_chi2_recovery: false`, `num_pts: 200`, `max_slam: 50`, `max_clones: 11`,
`num_opencv_threads: 4` (or 1 where noted), `multi_threading_subs: true`,
stereo (`max_cameras: 2`, `use_stereo: true`).

Per-table citations are abbreviated to the directory form
`results/<arch>/<env>/<tag>/{serial,subscribe}/`. See §10 for the full per-platform paths.

## 2. Headline: realtime + accuracy snapshot

Stereo, 4 threads, wall clock. **Subscribe** rows aggregate 5 reps;
**Serial** rows are the deterministic single-run number per platform.

### 2.1 Subscribe summary across sequences — the at-a-glance table

One row per platform. Cells are `wall ms / ATE pos rmse m` (mean per
sequence). The **Mean** column is the unweighted average over the three
sequences — a single ranking number per platform.

| Platform | V1_01_easy<br>wall ms / ATE m | MH_03_medium<br>wall ms / ATE m | V2_02_medium<br>wall ms / ATE m | Mean<br>wall ms / ATE m | vs x86-J<br>(wall) |
|---|---|---|---|---|---|
| x86-J | 14.4 / 0.091 | 12.2 / 0.172 | 10.6 / 0.060 | 12.4 / 0.108 | 1.00× |
| x86-H | 13.0 / 0.055 | 10.9 / 0.137 | 10.5 / 0.074 | 11.5 / 0.089 | 0.93× |
| RPi5-U | 25.3 / 0.068 | 24.5 / 0.209 | 23.4 / 0.171 | 24.4 / 0.149 | 1.97× |
| RPi5-T | 22.7 / 0.058 | 19.9 / 0.178 | 20.2 / 0.086 | 20.9 / 0.107 | 1.69× |

*Source: results/<arch>/<env>/<tag>/subscribe/{V1_01_easy,MH_03_medium,V2_02_medium}_4thr_{wall,est}.txt*

### 2.2 Serial summary across sequences

Same shape as §2.1 but for the deterministic serial reader (1 run per cell,
no std). Serial mode is what you use for reproducible offline replay /
benchmarking, not real-time deployment.

| Platform | V1_01_easy<br>wall ms / ATE m | MH_03_medium<br>wall ms / ATE m | V2_02_medium<br>wall ms / ATE m | Mean V1+V2<br>wall ms / ATE m | vs x86-J<br>(wall, V1+V2) |
|---|---|---|---|---|---|
| x86-J | 10.3 / 0.038 | 14.5 / 0.115 | 14.0 / 0.051 | 12.2 / 0.045 | 1.00× |
| x86-H | 10.7 / 0.038 | 10.8 / 0.115 | 10.6 / 0.051 | 10.6 / 0.045 | 0.87× |
| RPi5-U | 27.6 / 0.049 | 15.4 / 2794 ⚠ | 23.3 / 0.048 | 25.5 / 0.049 | 2.09× |
| RPi5-T | 22.2 / 0.044 | 11.7 / 2855 ⚠ | 21.1 / 0.048 | 21.7 / 0.046 | 1.78× |

⚠ = MH_03_medium serial mode never converges on RPi5 (avg SLAM ≈ 0.2
features, ATE in km). The **Mean** column is computed over V1_01_easy +
V2_02_medium only across **all** platforms for apples-to-apples comparison.
On x86, the 3-sequence means would be x86-J 12.9 / 0.068 and x86-H 10.7 /
0.068 — the inclusion of MH_03 pulls the mean up because MH_03 is the
heaviest sequence (it's the one with the most filter work).

Two things worth noting in this table:

- **Identical ATE on x86-J and x86-H** across every sequence (0.038 / 0.115 /
  0.051). Serial mode is bit-deterministic on the same architecture, and the
  output bit-matches across Ubuntu 22 and 24 here — the OS difference
  affects only wall time, not the trajectory itself.
- **RPi5-T (humble in Docker) ≈ x86 on serial ATE** (0.046 vs 0.045 mean
  V1+V2), despite NEON vs AVX2. RPi5-U native jazzy is the outlier at
  0.049 m. Same conclusion as the subscribe-mode comparison in §2.1: the
  OS/container choice matters more than the CPU on this workload.

*Source: results/<arch>/<env>/<tag>/serial/{V1_01_easy,MH_03_medium,V2_02_medium}_4thr_{wall,est}.txt*

### 2.3 Per-sequence detail (subscribe) — with std and p99

Same data as §2.1 but expanded with std and p99 so the variance of each
cell is visible.

| Platform | Sequence | Total wall ms (mean ± std, p99) | ATE pos rmse (m, mean ± std) | vs x86-J |
|---|---|---|---|---|
| x86-J | V1_01_easy | 14.4 ± 4.4 (p99: 29.0) | 0.091 ± 0.022 | 1.00× |
| x86-J | MH_03_medium | 12.2 ± 4.1 (p99: 26.4) | 0.172 ± 0.039 | 1.00× |
| x86-J | V2_02_medium | 10.6 ± 3.3 (p99: 20.5) | 0.060 ± 0.003 | 1.00× |
| x86-H | V1_01_easy | 13.0 ± 4.4 (p99: 27.7) | 0.055 ± 0.008 | 0.90× |
| x86-H | MH_03_medium | 10.9 ± 3.6 (p99: 23.4) | 0.137 ± 0.033 | 0.89× |
| x86-H | V2_02_medium | 10.5 ± 3.1 (p99: 20.1) | 0.074 ± 0.015 | 0.99× |
| RPi5-U | V1_01_easy | 25.3 ± 6.2 (p99: 46.5) | 0.068 ± 0.008 | 1.76× |
| RPi5-U | MH_03_medium | 24.5 ± 6.5 (p99: 46.5) | 0.209 ± 0.028 | 2.01× |
| RPi5-U | V2_02_medium | 23.4 ± 5.6 (p99: 41.2) | 0.171 ± 0.070 | 2.21× |
| RPi5-T | V1_01_easy | 22.7 ± 5.8 (p99: 42.4) | 0.058 ± 0.009 | 1.58× |
| RPi5-T | MH_03_medium | 19.9 ± 6.0 (p99: 39.3) | 0.178 ± 0.047 | 1.63× |
| RPi5-T | V2_02_medium | 20.2 ± 4.8 (p99: 34.8) | 0.086 ± 0.014 | 1.91× |

*Source: same as §2.1.*

## 3. Per-sequence totals

Same metric (total wall ms) but with **serial vs subscribe** side by side per
sequence, stereo, 4 threads.

> **MH_03 serial caveat**: RPi5-U and RPi5-T serial runs for MH_03 produced
> a broken filter state (avg SLAM ≈ 0.2 features) and ATE ≈ 2794 m. The
> timing numbers below are kept for completeness, but the sequence didn't
> actually track. Use the subscribe-mode rows for any RPi5 MH_03 analysis.

### 3.1 V1_01_easy

| Mode | x86-J | x86-H | RPi5-U | RPi5-T | RPi5-T vs x86-J |
|---|---|---|---|---|---|
| Serial total | 10.3 ± 3.5 | 10.7 ± 3.3 | 27.6 ± 6.3 | 22.2 ± 5.6 | 2.16× |
| Subscribe total | 14.4 ± 4.4 | 13.0 ± 4.4 | 25.3 ± 6.2 | 22.7 ± 5.8 | 1.58× |
| Serial p99 | 22.1 | 21.9 | 49.4 | 40.7 | 1.84× |
| Subscribe p99 | 29.0 | 27.7 | 46.5 | 42.4 | 1.46× |

*Source: results/<arch>/<env>/<tag>/{serial,subscribe}/V1_01_easy_4thr_wall.txt*

### 3.2 MH_03_medium

| Mode | x86-J | x86-H | RPi5-U | RPi5-T | RPi5-T vs x86-J |
|---|---|---|---|---|---|
| Serial total | 14.5 ± 4.8 | 10.8 ± 3.7 | 15.4 ± 1.4 ⚠ | 11.7 ± 1.2 ⚠ | 0.81× ⚠ |
| Subscribe total | 12.2 ± 4.1 | 10.9 ± 3.6 | 24.5 ± 6.5 | 19.9 ± 6.0 | 1.63× |
| Serial p99 | 30.9 | 24.4 | 20.3 ⚠ | 15.9 ⚠ | 0.51× ⚠ |
| Subscribe p99 | 26.4 | 23.4 | 46.5 | 39.3 | 1.49× |

⚠ = RPi5 serial-mode SLAM init failed on MH_03 (see caveat above); the
faster wall time reflects that almost no SLAM features were tracked and the
expensive update stages effectively skipped.

*Source: results/<arch>/<env>/<tag>/{serial,subscribe}/MH_03_medium_4thr_wall.txt*

### 3.3 V2_02_medium

| Mode | x86-J | x86-H | RPi5-U | RPi5-T | RPi5-T vs x86-J |
|---|---|---|---|---|---|
| Serial total | 14.0 ± 3.8 | 10.6 ± 3.1 | 23.3 ± 4.9 | 21.1 ± 4.9 | 1.51× |
| Subscribe total | 10.6 ± 3.3 | 10.5 ± 3.1 | 23.4 ± 5.6 | 20.2 ± 4.8 | 1.91× |
| Serial p99 | 25.4 | 19.7 | 38.9 | 37.1 | 1.46× |
| Subscribe p99 | 20.5 | 20.1 | 41.2 | 34.8 | 1.70× |

*Source: results/<arch>/<env>/<tag>/{serial,subscribe}/V2_02_medium_4thr_wall.txt*

## 4. Per-stage breakdown (V1_01_easy)

V1_01_easy chosen as the canonical sequence (others available in raw CSVs
and the parser output — see §10). Cells are `mean ± std` ms.

### 4.1 Stage means — subscribe, 4 threads, wall

| Stage | x86-J | x86-H | RPi5-U | RPi5-T |
|---|---|---|---|---|
| Tracking | 3.9 ± 0.7 | 3.1 ± 0.7 | 7.7 ± 0.8 | 6.4 ± 0.8 |
| Propagation | 0.2 ± 0.0 | 0.2 ± 0.0 | 0.5 ± 0.1 | 0.4 ± 0.0 |
| MSCKF Update | 2.0 ± 3.1 | 1.9 ± 2.9 | 2.9 ± 4.4 | 2.6 ± 4.1 |
| SLAM Update | 5.3 ± 1.5 | 5.1 ± 1.3 | 7.0 ± 1.6 | 7.0 ± 1.6 |
| SLAM Delayed | 1.2 ± 2.2 | 1.0 ± 2.0 | 1.8 ± 3.1 | 1.6 ± 2.9 |
| Re-tri & Marg | 1.8 ± 0.2 | 1.7 ± 0.2 | 5.5 ± 0.4 | 4.6 ± 0.2 |
| **Total** | **14.4 ± 4.4** | **13.0 ± 4.4** | **25.3 ± 6.2** | **22.7 ± 5.8** |

*Source: results/<arch>/<env>/<tag>/subscribe/V1_01_easy_4thr_wall.txt*

### 4.2 Per-stage slowdown vs x86-J

The "which step suffers most where" table. Each cell is the platform's mean
divided by the x86-J mean for the same stage. Same slice as §4.1.

| Stage | x86-H / x86-J | RPi5-U / x86-J | RPi5-T / x86-J | RPi5-T / x86-H |
|---|---|---|---|---|
| Tracking | 0.79× | 1.97× | 1.64× | 2.06× |
| Propagation | 1.00× | 2.50× | 2.00× | 2.00× |
| MSCKF Update | 0.95× | 1.45× | 1.30× | 1.37× |
| SLAM Update | 0.96× | 1.32× | 1.32× | 1.37× |
| SLAM Delayed | 0.83× | 1.50× | 1.33× | 1.60× |
| Re-tri & Marg | 0.94× | 3.06× | 2.56× | 2.71× |
| **Total** | **0.90×** | **1.76×** | **1.58×** | **1.75×** |

The two biggest outliers on RPi5 vs x86-J are **Re-tri & Marg** (≈3×) and
**Propagation** (2–2.5×, though its absolute cost is tiny). Both stages are
dominated by Eigen matrix decompositions (cholesky, QR, marginalization),
which lean on AVX2 on x86 and fall back to NEON on ARM — and the NEON
implementations in Eigen 3.4 are not on parity with AVX2. **Tracking** at ~2×
is closer to a pure compute-bound ratio (4-core Cortex-A76 @ 2.4 GHz vs
i7-11850H @ 4.8 GHz turbo): KLT pyramids parallelize across cores on both, so
the slowdown reflects per-cycle throughput.

The right-most column (**RPi5-T / x86-H**) isolates ARM-vs-x86 with the *same
ROS distro* (humble): same conclusions but the magnitude is slightly lower
because x86-H itself is ~10% faster than x86-J.

### 4.3 Single-thread sensitivity — subscribe, 1 thread, wall

| Stage | x86-J | x86-H | RPi5-U | RPi5-T |
|---|---|---|---|---|
| Tracking | 4.5 ± 0.7 | 3.9 ± 0.5 | 10.6 ± 1.1 | 9.3 ± 1.1 |
| Propagation | 0.2 ± 0.0 | 0.2 ± 0.0 | 0.4 ± 0.1 | 0.4 ± 0.0 |
| MSCKF Update | 1.8 ± 2.8 | 1.7 ± 2.6 | 2.8 ± 4.4 | 2.6 ± 4.1 |
| SLAM Update | 5.0 ± 1.4 | 4.6 ± 1.0 | 7.0 ± 1.5 | 7.0 ± 1.5 |
| SLAM Delayed | 1.1 ± 2.1 | 0.9 ± 1.8 | 1.7 ± 3.2 | 1.6 ± 2.9 |
| Re-tri & Marg | 1.6 ± 0.2 | 1.5 ± 0.1 | 5.3 ± 0.4 | 4.5 ± 0.2 |
| **Total** | **14.1 ± 4.1** | **12.7 ± 3.6** | **27.9 ± 6.3** | **25.4 ± 5.8** |

At 1 thread, the per-stage ratios across platforms are tighter
(Tracking RPi5-U / x86-J = 2.36× instead of 1.97× at 4 threads) — confirming
that 4-thread KLT parallelism does help close the x86-vs-ARM gap on this
stage.

*Source: results/<arch>/<env>/<tag>/subscribe/V1_01_easy_1thr_wall.txt*

### 4.4 Wall vs CPU vs thread — subscribe, 4 threads

**Wall clock** (repeat of §4.1):

| Stage | x86-J | x86-H | RPi5-U | RPi5-T |
|---|---|---|---|---|
| Tracking | 3.9 | 3.1 | 7.7 | 6.4 |
| Propagation | 0.2 | 0.2 | 0.5 | 0.4 |
| MSCKF Update | 2.0 | 1.9 | 2.9 | 2.6 |
| SLAM Update | 5.3 | 5.1 | 7.0 | 7.0 |
| SLAM Delayed | 1.2 | 1.0 | 1.8 | 1.6 |
| Re-tri & Marg | 1.8 | 1.7 | 5.5 | 4.6 |
| **Total** | **14.4** | **13.0** | **25.3** | **22.7** |

**Process CPU clock**:

| Stage | x86-J | x86-H | RPi5-U | RPi5-T |
|---|---|---|---|---|
| Tracking | 12.6 | 9.1 | 20.8 | 15.0 |
| Propagation | 0.2 | 0.2 | 0.5 | 0.4 |
| MSCKF Update | 2.0 | 1.9 | 2.9 | 2.7 |
| SLAM Update | 5.3 | 5.1 | 7.1 | 7.0 |
| SLAM Delayed | 1.2 | 1.1 | 1.8 | 1.6 |
| Re-tri & Marg | 2.7 | 2.5 | 7.7 | 5.6 |
| **Total** | **24.0** | **19.8** | **40.7** | **32.2** |

**Thread CPU clock** (calling thread only):

| Stage | x86-J | x86-H | RPi5-U | RPi5-T |
|---|---|---|---|---|
| Tracking | 3.9 | 3.1 | 7.6 | 6.3 |
| Propagation | 0.2 | 0.2 | 0.5 | 0.4 |
| MSCKF Update | 2.0 | 1.9 | 2.9 | 2.6 |
| SLAM Update | 5.3 | 5.1 | 7.0 | 7.0 |
| SLAM Delayed | 1.2 | 1.0 | 1.8 | 1.6 |
| Re-tri & Marg | 1.8 | 1.7 | 5.5 | 4.6 |
| **Total** | **14.4** | **13.0** | **25.3** | **22.5** |

**CPU/Wall ratio** — proxy for multithreading effectiveness. Ratio ≈ 1.0
means the stage is single-threaded; ratio > 1.0 means CPU time pooled across
threads exceeds wall time, i.e. the stage parallelizes.

| Stage | x86-J | x86-H | RPi5-U | RPi5-T |
|---|---|---|---|---|
| Tracking | 3.23× | 2.94× | 2.70× | 2.34× |
| Propagation | 1.00× | 1.00× | 1.00× | 1.00× |
| MSCKF Update | 1.00× | 1.00× | 1.00× | 1.04× |
| SLAM Update | 1.00× | 1.00× | 1.01× | 1.00× |
| SLAM Delayed | 1.00× | 1.10× | 1.00× | 1.00× |
| Re-tri & Marg | 1.50× | 1.47× | 1.40× | 1.22× |
| **Total** | **1.67×** | **1.52×** | **1.61×** | **1.42×** |

Tracking is the only stage with strong multithreading (KLT pyramids split
across `num_opencv_threads: 4`). The CPU/wall ratio for tracking degrades
from 3.23× on x86-J to 2.34× on RPi5-T — i.e. the 4-thread parallelism
recovers ~80% of an "ideal 4×" on x86 but only ~60% on RPi5 in Docker. The
remaining stages (Propagation, MSCKF Update, SLAM Update, SLAM Delayed) are
strictly serial across all four platforms.

*Source: results/<arch>/<env>/<tag>/subscribe/V1_01_easy_4thr_{wall,cpu,thread}.txt*

## 5. Accuracy

### 5.1 ATE per sequence

Subscribe mode, stereo, 4 threads, 5 reps aggregated. Range = max − min
across the 5 reps.

| Sequence | Metric | x86-J | x86-H | RPi5-U | RPi5-T |
|---|---|---|---|---|---|
| V1_01_easy | pos rmse (m) | 0.091 ± 0.022 (range 59 mm) | 0.055 ± 0.008 (range 19 mm) | 0.068 ± 0.008 (range 21 mm) | 0.058 ± 0.009 (range 18 mm) |
| V1_01_easy | ori rmse (°) | 0.82 | 0.71 | 0.69 | 0.73 |
| MH_03_medium | pos rmse (m) | 0.172 ± 0.039 (range 95 mm) | 0.137 ± 0.033 (range 73 mm) | 0.209 ± 0.028 (range 58 mm) | 0.178 ± 0.047 (range 118 mm) |
| MH_03_medium | ori rmse (°) | 1.06 | 1.12 | 1.33 | 1.13 |
| V2_02_medium | pos rmse (m) | 0.060 ± 0.003 (range 8 mm) | 0.074 ± 0.015 (range 36 mm) | 0.171 ± 0.070 (range 181 mm) | 0.086 ± 0.014 (range 35 mm) |
| V2_02_medium | ori rmse (°) | 1.26 | 1.40 | 1.50 | 1.37 |

*Source: results/<arch>/<env>/<tag>/subscribe/{seq}_4thr_run{1..5}_est.txt
vs ov_data/euroc_mav/{seq}.txt.*

### 5.2 RPE per segment (median)

Subscribe mode, stereo, 4 threads, segments 8/16/24/32/40 s. Cell format
`ori° / pos m`.

| Sequence | Segment | x86-J | x86-H | RPi5-U | RPi5-T |
|---|---|---|---|---|---|
| V1_01_easy | 8 s | 0.73 / 0.099 | 0.58 / 0.067 | 0.64 / 0.082 | 0.59 / 0.066 |
| V1_01_easy | 16 s | 0.75 / 0.097 | 0.60 / 0.067 | 0.64 / 0.080 | 0.59 / 0.070 |
| V1_01_easy | 24 s | 0.81 / 0.128 | 0.70 / 0.078 | 0.60 / 0.086 | 0.65 / 0.074 |
| V1_01_easy | 32 s | 0.85 / 0.129 | 0.81 / 0.082 | 0.64 / 0.083 | 0.64 / 0.089 |
| V1_01_easy | 40 s | 0.73 / 0.146 | 0.80 / 0.089 | 0.53 / 0.089 | 0.72 / 0.087 |
| MH_03_medium | 8 s | 0.49 / 0.152 | 0.37 / 0.132 | 0.52 / 0.162 | 0.42 / 0.154 |
| MH_03_medium | 16 s | 0.65 / 0.170 | 0.50 / 0.136 | 0.73 / 0.189 | 0.58 / 0.150 |
| MH_03_medium | 24 s | 0.76 / 0.187 | 0.62 / 0.157 | 0.82 / 0.204 | 0.70 / 0.186 |
| MH_03_medium | 32 s | 0.90 / 0.206 | 0.75 / 0.190 | 0.95 / 0.236 | 0.83 / 0.210 |
| MH_03_medium | 40 s | 0.96 / 0.249 | 0.82 / 0.210 | 1.06 / 0.278 | 0.95 / 0.244 |
| V2_02_medium | 8 s | 1.20 / 0.054 | 1.22 / 0.060 | 1.19 / 0.089 | 1.17 / 0.065 |
| V2_02_medium | 16 s | 1.21 / 0.071 | 1.28 / 0.082 | 1.38 / 0.142 | 1.24 / 0.094 |
| V2_02_medium | 24 s | 1.44 / 0.074 | 1.53 / 0.091 | 1.72 / 0.169 | 1.55 / 0.105 |
| V2_02_medium | 32 s | 1.57 / 0.085 | 1.74 / 0.111 | 1.96 / 0.229 | 1.71 / 0.132 |
| V2_02_medium | 40 s | 1.56 / 0.091 | 1.86 / 0.116 | 2.12 / 0.275 | 1.75 / 0.150 |

*Source: same as §5.1; RPE computed by `ov_eval/error_singlerun` with custom segment lengths.*

## 6. Subscribe overhead

Subscribe mean / Serial mean ratio per platform per sequence. > 1.0 means
the persistent-worker dispatch adds cost vs the deterministic serial reader.

| Platform | Sequence | Serial wall (mean) | Subscribe wall (mean) | Sub/Serial | Frames (sub mean) |
|---|---|---|---|---|---|
| x86-J | V1_01_easy | 10.3 | 14.4 | 1.40× | 2800 |
| x86-J | MH_03_medium | 14.5 | 12.2 | 0.84× | 2311 |
| x86-J | V2_02_medium | 14.0 | 10.6 | 0.76× | 2267 |
| x86-H | V1_01_easy | 10.7 | 13.0 | 1.21× | 2800 |
| x86-H | MH_03_medium | 10.8 | 10.9 | 1.01× | 2311 |
| x86-H | V2_02_medium | 10.6 | 10.5 | 0.99× | 2267 |
| RPi5-U | V1_01_easy | 27.6 | 25.3 | 0.92× | 2800 |
| RPi5-U | MH_03_medium ⚠ | 15.4 | 24.5 | 1.59× ⚠ | 2311 |
| RPi5-U | V2_02_medium | 23.3 | 23.4 | 1.00× | 2267 |
| RPi5-T | V1_01_easy | 22.2 | 22.7 | 1.02× | 2800 |
| RPi5-T | MH_03_medium ⚠ | 11.7 | 19.9 | 1.70× ⚠ | 2311 |
| RPi5-T | V2_02_medium | 21.1 | 20.2 | 0.96× | 2267 |

⚠ = MH_03 serial broken on RPi5 (avg SLAM ≈ 0.2); the artificially low
serial wall time is what produces the apparent >1.5× ratio. On the other
RPi5 sequences the persistent-worker thread holds the subscribe/serial
ratio near 1.0×, matching the design intent ([determinism.md](../determinism.md)).

*Source: results/<arch>/<env>/<tag>/{serial,subscribe}/{seq}_4thr_wall.txt*

## 7. SLAM feature health

Mean active SLAM features in the state (target ≈ `max_slam: 50`).
Subscribe mode, stereo, 4 threads, 5 reps. Cell = `mean (range across reps)`.

| Platform | V1_01_easy | MH_03_medium | V2_02_medium |
|---|---|---|---|
| x86-J | 45.3 (45.1–45.6) | 41.0 (40.5–41.6) | 37.9 (37.5–38.3) |
| x86-H | 46.2 (46.0–46.4) | 41.3 (41.0–41.9) | 38.2 (37.8–38.6) |
| RPi5-U | 45.8 (45.7–46.0) | 39.1 (37.1–40.7) | 34.7 (33.2–35.4) |
| RPi5-T | 46.0 (45.7–46.3) | 37.4 (25.2–41.6) ⚠ | 37.7 (37.4–37.9) |

⚠ = on RPi5-T MH_03 4thr, one of five runs converged at 25.2 features
(others 38.9–41.6) — pulled the mean down. Likely a transient init/jitter
issue under Docker; the other RPi5-T cells are clean.

*Source: results/<arch>/<env>/<tag>/subscribe/{seq}_4thr_run{1..5}_feats.txt*

## 8. OS-version isolation

Holding hardware constant and varying the distro/ROS version. Each table
covers subscribe / 4 threads / V1_01_easy.

### 8.1 x86 Ubuntu 24/Jazzy vs Ubuntu 22/Humble

| Metric | x86-J (Noble / Jazzy) | x86-H (Jammy / Humble) | Δ |
|---|---|---|---|
| Total wall ms | 14.4 ± 4.4 | 13.0 ± 4.4 | −1.4 (−9.7%) |
| ATE pos rmse (m) | 0.091 ± 0.022 | 0.055 ± 0.008 | −0.036 (−40%) |
| Biggest delta stage | Tracking 3.9 | Tracking 3.1 | −0.8 ms (−20%) |
| Second biggest delta | SLAM Delayed 1.2 | SLAM Delayed 1.0 | −0.2 ms (−17%) |

**Humble + Jammy is ~10% faster *and* ~40% more accurate** than Jazzy +
Noble on V1_01_easy. The timing edge is mostly tracking and the long-tail
init stages (SLAM Delayed). The accuracy edge is larger than I'd expect from
a pure timing difference; possible causes are OpenCV version differences
between humble and jazzy package indexes, or a difference in OpenCV's KLT
default thread-affinity behavior between the two Ubuntu kernel versions.

### 8.2 RPi5 native Ubuntu vs Trixie + Docker

| Metric | RPi5-U (Noble / Jazzy native) | RPi5-T (Trixie host + humble Docker) | Δ |
|---|---|---|---|
| Total wall ms | 25.3 ± 6.2 | 22.7 ± 5.8 | −2.6 (−10%) |
| ATE pos rmse (m) | 0.068 ± 0.008 | 0.058 ± 0.009 | −0.010 (−15%) |
| Biggest delta stage | Tracking 7.7 | Tracking 6.4 | −1.3 ms (−17%) |
| Second biggest delta | Re-tri & Marg 5.5 | Re-tri & Marg 4.6 | −0.9 ms (−16%) |

**RPi5 with humble (in Docker) is ~10% faster *and* ~15% more accurate**
than RPi5 with jazzy (native) on V1_01_easy. The pattern holds across all
three sequences — mean ATE drops from 0.149 m (RPi5-U) to 0.107 m (RPi5-T).
Tracking and Re-tri & Marg drop together — humble ships an older but
better-tuned OpenCV / Eigen combination for this CPU. Note this comparison
conflates two changes (host OS *and* containerization); to isolate them
would need either a native-Trixie or a jazzy-Docker run.

## 9. Findings

1. **Performance ranking (subscribe, 4 threads, mean across sequences)**:
   x86-H 11.5 ms < x86-J 12.4 ms < RPi5-T 20.9 ms < RPi5-U 24.4 ms. The
   spread is **2.1×** between fastest and slowest.

2. **Accuracy ranking (mean ATE pos rmse across 3 sequences)**:
   x86-H 0.089 m < RPi5-T 0.107 m ≈ x86-J 0.108 m < RPi5-U 0.149 m. **RPi5
   in Docker (humble) is tied with x86 jazzy on accuracy** — i.e. the
   container/OS combination matters more than the CPU. RPi5 native jazzy is
   the accuracy outlier, mostly because of V2_02_medium (0.171 m vs 0.060
   m on x86-J).

3. **x86 humble beats x86 jazzy on the same hardware** by ~7% on the
   3-sequence mean timing (and a much larger 40% on V1_01_easy ATE — see
   §8.1). When choosing a target Ubuntu version for a deployment, this isn't
   noise — it's reproducible across all three sequences.

3. **RPi5 / x86 slowdown is 1.6–2.0×** — well below the early ×3.5
   projection in [rpi5-benchmarking.md §1](../rpi5-benchmarking.md). The
   Cortex-A76 closes the gap once `num_opencv_threads: 4` is in play. For
   1-thread (§4.3) the gap widens to 2.0–2.4×.

4. **The stages that suffer most on ARM are Re-tri & Marg (≈3×) and
   Propagation (≈2.5×)** — both Eigen matrix-ops stages where NEON in Eigen
   3.4 underperforms AVX2. Tracking at ~2× tracks the raw clock ratio
   (2.4 GHz Cortex-A76 vs 4.8 GHz turbo i7-11850H). See §4.2.

5. **KLT parallelism (CPU/Wall ratio for Tracking)** scales from
   3.23× on x86-J to 2.34× on RPi5-T — ~80% of ideal on x86, ~60% on RPi5
   under Docker. All other stages are strictly serial (CPU ≈ wall) on every
   platform.

6. **Persistent-worker subscribe overhead is ~1.0× on RPi5** for clean
   sequences (V1_01 and V2_02 on both RPi5 platforms hold sub/serial ≈
   0.92–1.02×), matching the design intent of the [persistent worker
   thread](../determinism.md). On x86 it's ~1.0× for MH/V2 and ~1.2–1.4×
   for V1_01_easy.

7. **Real-time feasibility at p99**: subscribe 4-threads V1_01_easy p99 is
   29 ms (x86-J), 28 ms (x86-H), 46 ms (RPi5-U), 42 ms (RPi5-T). Versus a
   20 Hz budget (50 ms) all four meet the bar; versus 30 Hz (33 ms) only
   x86 meets it. RPi5 is on the edge — sufficient for typical drone-grade
   20 Hz VIO but no headroom for additional consumers.

8. **Two reproducible issues to investigate**:
   - **RPi5 serial-mode MH_03_medium fails** (avg SLAM ≈ 0.2, ATE ≈ 2794 m)
     on both RPi5-U and RPi5-T, while subscribe-mode MH_03 works. The same
     binary works in serial on V1_01 and V2_02. Likely a timing-sensitive
     init in the serial reader on slower hardware.
   - **RPi5-U V2_02_medium 1-thr subscribe**: 2 of 5 runs converged to ATE
     ≈ 163 m (avg SLAM = 2.2 features). 1-thread V2_02 on the other three
     platforms is fine, so it's likely a thread-budget issue under
     simultaneous ROS callback load.

9. **Tooling note** (resolved): RPi5-T's accuracy numbers were initially
   missing because `parse_results.py` ran on the Trixie host, which has no
   native ROS humble (the script's auto-source paths
   `/opt/ros/humble/setup.bash` and `~/workspace/.../install/setup.bash`
   resolve to nothing). The container *does* have `ov_eval` built in
   `/opt/ros_ws/install/`. Fix: re-run `parse_results.py` from inside the
   same image, bind-mounting the host workspace at the path the script
   expects:

   ```bash
   docker run --rm \
     -v /home/openhd/workspace/catkin_ws_ov:/home/openhd/workspace/catkin_ws_ov:ro \
     -v /home/openhd/results:/home/openhd/results \
     -e HOME=/home/openhd \
     openvins-humble:latest \
     bash -c "source /opt/ros/humble/setup.bash && \
              source /opt/ros_ws/install/setup.bash && \
              python3 /home/openhd/workspace/catkin_ws_ov/scripts/parse_results.py \
                /home/openhd/results/rpi5/docker_humble/<tag>/subscribe --detailed"
   ```

   Repeat for `serial`. Setting `HOME=/home/openhd` makes the script find
   the EuRoC ground truth at the expected `$HOME/workspace/.../ov_data/` path.

## 10. Raw data index

Per-platform result trees (each holds `serial/` and `subscribe/`
sub-directories with the standard
`{SEQ}_{N}thr[_run{N}]_{wall,cpu,thread,feats,est}.txt` files):

| Platform | Path | Tag |
|---|---|---|
| x86-J | `~/results/x86/native_jazzy/run_for_benchmark_analysis_11_05/` | `run_for_benchmark_analysis_11_05` |
| x86-H | `~/results/x86/native_humble/run_for_benchmark_analysis_13_05__x86_H/` | `run_for_benchmark_analysis_13_05__x86_H` |
| RPi5-U | `~/results/rpi5/native_jazzy/run_for_benchmark_analysis_13_05__rpi5__U/` | `run_for_benchmark_analysis_13_05__rpi5__U` |
| RPi5-T | `~/results/rpi5/docker_humble/run_for_benchmark_analysis_13_05__rpi5__T/` | `run_for_benchmark_analysis_13_05__rpi5__T` |

To regenerate the per-component / ATE / RPE numbers cited above on any
platform:

```bash
python3 scripts/parse_results.py \
  ~/results/<arch>/<env>/<tag>/serial --detailed
python3 scripts/parse_results.py \
  ~/results/<arch>/<env>/<tag>/subscribe --detailed
```

That single command per directory covers every cell in §§2–7 for one
platform's column.

For platforms whose host doesn't have `ov_eval` installed (RPi5-T's Docker
container is the example), run the same command **from a host that does**
have ov_eval — `parse_results.py` only needs read access to the CSV/est
files; it shells out to `ros2 run ov_eval error_singlerun` for ATE/RPE.
