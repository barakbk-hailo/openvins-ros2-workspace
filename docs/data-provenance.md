# Data Provenance

Each benchmark CSV and trajectory estimate under `results/` was produced by one
of the runs catalogued below. The `*Source: results/...*` lines elsewhere in the
docs cite a tag from this table; metadata for that tag (platform, code, config)
is given here so individual citations stay short.

## Hardware

| Label | Machine | OS | CPU | Notes |
|---|---|---|---|---|
| **x86 (Latitude)** | Dell Latitude 5420 | Ubuntu 22.04 (Jammy) | Intel i7-1185G7, 4 cores / 8 threads, Iris Xe iGPU | All `results/timing/x86/` and `results/{stereo,mono}/` data is from this machine |
| **RPi5 (openhd)** | Raspberry Pi 5 (`openhd@192.168.200.81`, hostname `openhdair`) | Debian 13 Trixie, kernel 6.12.62 aarch64 | Broadcom BCM2712, 4× Cortex-A76 @ 2.4 GHz, 8 GB RAM | All `results/rpi5/` and `results/timing/rpi5/` data is from this machine. Runs are inside a Docker container (`openvins-humble:latest`) for reproducibility. |

## x86 tags

> **Submodule SHA note:** All rows below cite `master-candidate / 2a50450` because that's the commit the data was actually collected against. The outer repo's `master-candidate` submodule pointer has since advanced to `064c717` via two commits: `f2fe5b9` (ReadMe-only — rewrites the `slam_chi2_recovery` rationale) and `064c717` itself (Dockerfile-only — flips the default-clone branch). Neither touches the filter, YAML defaults, or the build, so reproducing against either tip yields the same numbers.

| Tag (under `results/timing/x86/`) | Submodule | Outer commit | Date | chi2_recovery | OpenCV threads | Notes |
|---|---|---|---|---|---|---|
| `serial/rerun_2026_04_23/` | `master-candidate` / `2a50450` | `8424c9e` | 2026-04-26 | `false` (shipping default) | 4 + 1 | Main 3-clock suite — V1_01_easy / MH_03_medium / V2_02_medium × stereo + mono × `4-thr` & `1-thr`, 1 serial rep per config |
| `subscribe/rerun_2026_04_23/` | same | same | 2026-04-26 | `false` | 4 + 1 | Subscribe portion of the main suite — same 3 sequences × `4-thr` & `1-thr` × 5 reps each |
| `subscribe/rerun_2026_04_23/V1_01_easy_rate{1.0,2.0,5.0}*` | same | same | 2026-04-26 | `false` | 4 | Subscribe realtime feasibility — V1_01_easy at 3 playback rates |
| `serial/sweep/rerun_2026_04_23/` | same | same | 2026-04-26 | `false` | varies (E variant uses 1) | 5 config-sensitivity variants on V1_01_easy stereo serial: A_downsample, B_num_pts_100, C_num_pts_300, D_no_slam, E_opencv_1thread |
| `serial/rerun_2026_04_23_paper/` | same | same | 2026-04-26 | `false` | 4 | 10 EuRoC × stereo+mono serial — paper Table II/III reproduction (`bag_start=0` for all sequences). The TUM-converted estimates were promoted to `results/{stereo,mono}/estimate_*.txt`. |
| `subscribe/rerun_2026_04_23_recovery_on/` | same | same | 2026-04-23 | overridden via temp config (`true`) | 4 | V1_03_difficult subscribe @ rate 1.0, 3 reps — chi2 A/B with recovery enabled |
| `subscribe/rerun_2026_04_23_recovery_off/` | same | same | 2026-04-23 | overridden (`false`) | 4 | V1_03_difficult subscribe @ rate 1.0, 3 reps — chi2 A/B with recovery disabled |
| `subscribe/rerun_2026_04_23_rate2_recovery_on/` | same | same | 2026-04-26 | overridden (`true`) | 4 | V1_03_difficult subscribe **@ rate 2.0**, 3 reps — overload-case chi2 A/B (the scenario where recovery actually helps) |
| `subscribe/rerun_2026_04_23_rate2_recovery_off/` | same | same | 2026-04-26 | overridden (`false`) | 4 | V1_03_difficult subscribe @ rate 2.0, 3 reps — overload-case A/B with recovery disabled |

## RPi5 tags

| Tag (under `results/rpi5/`) | Submodule | Outer commit | Date | chi2_recovery | Docker flags | Notes |
|---|---|---|---|---|---|---|
| `rerun_2026_04_26_pwt_baseline/` | `master-candidate` / `2a50450` (via Docker `openvins-humble:latest` rebuilt 2026-04-26) | `8424c9e` | 2026-04-26 | `false` | (none, only `-e HOME=/tmp` for `~/.ros/log` workaround) | 2 serial + 10 subscribe reps × V1_01_easy stereo via `run_pwt_benchmark_v2.sh` |
| `rerun_2026_04_26_pwt_rtflags/` | same | same | 2026-04-26 | `false` | `--cap-add=SYS_NICE --ulimit rtprio=99 --ulimit memlock=-1 --cpuset-cpus=0-3 -e HOME=/tmp` | RT scheduling flags applied to the Docker container |
| `rerun_2026_04_26_pwt_maxinterval/` | same | same | 2026-04-26 | `false` | (same as baseline) | **Identical code/flags to `pwt_baseline` under `master-candidate`** (max-interval is baked into the consolidated branch); kept as a separate-session run for cross-session stability comparison |
| `rerun_2026_04_26_pwt_combined/` | same | same | 2026-04-26 | `false` | (same as `pwt_rtflags`) | **Identical to `pwt_rtflags`** under `master-candidate`; cross-session comparison |
| `rerun_2026_04_26_pwt_final_maxinterval/` | same | same | 2026-04-26 | `false` | (same as baseline) | Back-to-back rerun of `pwt_maxinterval` — controlled session for between-session noise estimation |
| `rerun_2026_04_26_pwt_final_combined/` | same | same | 2026-04-26 | `false` | (same as `pwt_rtflags`) | Back-to-back rerun of `pwt_combined` — controlled session pair with the row above |
| `rerun_2026_04_26_paper/` | same | same | 2026-04-26 | `false` | `-e HOME=/tmp` only | 5 Vicon × stereo + mono serial via `serial.launch.py` — RPi5 paper-repro for x86-vs-RPi5 cross-platform comparison |

## Inherited config (all rows above)

Unless overridden in the "Notes" column, every run uses the YAML config at
`src/open_vins/config/euroc_mav/estimator_config.yaml` from submodule commit `2a50450`:

- `num_pts: 200`, `max_slam: 50`, `max_slam_in_update: 25`, `max_msckf_in_update: 40`
- `slam_chi2_recovery: false` (default since 2026-04-26)
- `num_opencv_threads: 4` (modified to `1` for the `1thr` rows by `make_config()` in `run_full_benchmark.sh`)
- `record_timing_information`, `record_timing_cpu_time`, `record_timing_thread_time`, `record_feature_counts`: all set to `true` in the temp config used at runtime so the 5-CSV output is produced; the source-tree YAML keeps them `false`.

EuRoC ground truth files used for ATE/RPE evaluation: `src/open_vins/ov_data/euroc_mav/{V1_01_easy,V1_02_medium,V1_03_difficult,V2_01_easy,V2_02_medium,MH_01_easy,MH_02_easy,MH_03_medium,MH_04_difficult,MH_05_difficult}.txt` from the same submodule commit.

## File-format conventions

Every benchmark run writes 5 CSVs per rep:

| File suffix | Format | Clock / contents |
|---|---|---|
| `_wall.txt` | `# timestamp(s),tracking,propagation,msckf,slam,slam_delayed,retri_marg,total` | `boost::posix_time` wall-clock (ms) per VIO stage |
| `_cpu.txt` | same columns | `CLOCK_PROCESS_CPUTIME_ID` (ms) — sums CPU time across all threads |
| `_thread.txt` | same columns | `CLOCK_THREAD_CPUTIME_ID` (ms) — VIO thread alone |
| `_feats.txt` | `# timestamp(s),slam_feats_in_state,msckf_feats_used,slam_feats_updated,slam_feats_delayed_init,clones` | per-frame feature counts |
| `_est.txt` (x86) / `_pose.txt` (RPi5 PWT) | `_est.txt` is full state dump (`timestamp qx qy qz qw px py pz vx vy vz bgx ...`); `_pose.txt` is TUM (`timestamp tx ty tz qx qy qz qw`) | trajectory estimate. Convert state-dump to TUM with `awk '!/^#/ && NF>=8 {print $1,$6,$7,$8,$2,$3,$4,$5}'` before invoking `ros2 run ov_eval error_singlerun`. |

The TUM conversion is critical — feeding the state-dump format directly to `error_singlerun` reads quaternion components as if they were position, producing wildly inflated ATE values (e.g. 1.94 m instead of 0.04 m for V1_01_easy). See [determinism.md §3](determinism.md#3-root-cause-fix-persistent-worker-thread) "Format note".

## Retired tags

These tag directories were deleted from `results/` (preserved in git history under outer commits `44b4cfa` for x86 and `8424c9e` for RPi5):

| Retired tag | Reason | Replacement |
|---|---|---|
| `results/timing/x86/serial/bench_5rep_3clock/` and `subscribe/bench_5rep_3clock/` | Pre-`68eee80` data; serial portion had subtle non-determinism from the spurious PWT thread spawned in serial mode | `rerun_2026_04_23` (same sequences, same reps, fresh measurements) |
| `results/timing/x86/serial/bench_persistent_worker/` and `subscribe/bench_persistent_worker/` | Same as above (the rerun and persistent-worker tags both pre-date the gating fix) | `rerun_2026_04_23` |
| `results/rpi5/{pwt_baseline,pwt_rtflags,pwt_maxinterval,pwt_combined,pwt_final_maxinterval,pwt_final_combined}/` | Pre-`master-candidate` Docker images (PWT-only and max-interval-only branches); cross-run timing variability was wider (5-25 ms range) due to image rebuild drift between sessions | `rerun_2026_04_26_pwt_*` (single consolidated `openvins-humble:latest` image with PWT + max-interval baked in; cross-run variability now ±0.08-0.15 ms) |
