# Data Provenance

Each benchmark CSV and trajectory estimate under `results/` was produced by one
of the runs catalogued below. The `*Source: results/...*` lines elsewhere in
the docs cite a tag from this table; metadata for that tag (platform, code,
config) is given here so individual citations stay short.

## Hardware

| Label | Machine | OS | CPU | Notes |
|---|---|---|---|---|
| **x86 (Latitude)** | Dell Latitude 5420 | Ubuntu 22.04 (Jammy) | Intel i7-1185G7, 4 cores / 8 threads, Iris Xe iGPU | All `results/x86/native_humble/` data is from this machine. |
| **RPi5 (openhd)** | Raspberry Pi 5 (`openhd@192.168.200.81`, hostname `openhdair`) | Debian 13 Trixie, kernel 6.12.62 aarch64 | Broadcom BCM2712, 4× Cortex-A76 @ 2.4 GHz, 8 GB RAM | All `results/rpi5/docker_humble/` data is from this machine. Runs are inside a Docker container (`openvins-humble:latest`) for reproducibility. |

## Environment-layer convention

Result paths are `results/<arch>/<env>/<tag>/<mode>/<files>` where:
- **`<arch>`** ∈ `{x86, rpi5}` — derived from `uname -m` (`x86_64` → `x86`, `aarch64` → `rpi5`) by `arch_results_base()` in `scripts/bench_lib.sh`.
- **`<env>`** = `<runtime>_<distro>` — defaults to `native_<distro>` (distro from `/etc/os-release`); when `--docker <image>` is passed to `run_full_benchmark.sh`, the env auto-flips to `docker_<distro>` (humble/jazzy parsed from the image tag). Override with `BENCH_ENV=<custom>` to force a name.
- **`<tag>`** is the `--tag` argument; **`<mode>`** ∈ `{serial, subscribe}` is the orchestrator mode. The tag-before-mode shape lets one experimental session emit serial+subscribe under one tag dir.

To date the only segments materialized on disk are `x86/native_humble/` and
`rpi5/docker_humble/`. Other combinations (`x86/docker_humble/`,
`x86/native_jazzy/`, `rpi5/native_humble/`) are reserved by the convention and
will appear if a run is performed in that environment.

## x86 tags

> **Submodule SHA note:** All rows below cite the `master-candidate` head at the time of collection (`2a50450` for the original `rerun_2026_04_23*` series, `009518c` for `rerun_2026_04_27*` after small ReadMe/default-flip follow-ups). Neither follow-up touches the filter, YAML defaults, or build, so reproducing against either tip yields the same numbers.

These tags were re-collected on **2026-04-27** against the post-PWT submodule,
superseding the prior `rerun_2026_04_23*` series (now under `_archive/`).

| Tag (under `results/x86/native_humble/`) | Submodule | Date | chi2_recovery | Threads | Notes |
|---|---|---|---|---|---|
| `rerun_2026_04_27_main/{serial,subscribe}/` | `master-candidate` head | 2026-04-27 | `false` (shipping default) | 4 + 1 | Main 3-clock suite — V1_01_easy / MH_03_medium / V2_02_medium × stereo + mono × {4-thr, 1-thr}, 1 serial rep + 5 subscribe reps. **Both modes share one tag dir.** Invocation: `bash scripts/run_full_benchmark.sh -m both -s V1_01_easy,MH_03_medium,V2_02_medium -t 4,1 -c both -r 5 --tag rerun_2026_04_27_main`. 210 files. |
| `rerun_2026_04_27_paper/serial/` | same | 2026-04-27 | `false` | 4 + 1 | 10 EuRoC × stereo + mono serial — paper Table II/III reproduction (`bag_start=0` for all sequences; MH_03 stereo converges on x86 with this default — see `rpi5-benchmarking.md:227` for why the RPi5 equivalent needs `bag_start=5`). Invocation: `bash scripts/run_full_benchmark.sh -m serial -c both -r 1 -s V1_01_easy,V1_02_medium,V1_03_difficult,V2_01_easy,V2_02_medium,MH_01_easy,MH_02_easy,MH_03_medium,MH_04_difficult,MH_05_difficult --tag rerun_2026_04_27_paper`. 200 files. |
| `rerun_2026_04_27_paper_recovery_on/serial/` | same | 2026-04-27 | overridden via temp config (`true`) | 4 + 1 | Same 10×2 paper-repro **with `--slam-chi2-recovery true`**. Used by `determinism.md` §4 as the chi2_recovery=true reference vs `rerun_2026_04_27_paper/` (recovery=false). 200 files. |
| `rerun_2026_04_27_sweep/serial/` | same | 2026-04-27 | `false` | varies (E variant uses 1) | 5 config-sensitivity variants on V1_01_easy stereo serial: A_downsample, B_num_pts_100, C_num_pts_300, D_no_slam, E_opencv_1thread. Invocation: `bash scripts/run_timing_sweep.sh --tag rerun_2026_04_27_sweep`. 15 files (3 clocks × 5 variants). |
| `rerun_2026_04_27_v103/serial/` | same | 2026-04-27 | `false` | 4 + 1 | V1_03_difficult serial stereo + mono — fills the V1_03 row in the timing tables (`rerun_2026_04_27_main/` covers V1_01/MH_03/V2_02). 20 files. |
| `rerun_2026_04_27_rate_sweep/subscribe/` | same | 2026-04-27 | `false` | 4 + 1 | V1_01_easy subscribe @ rates 1.0, 2.0, 5.0 × 5 reps — rate-feasibility / persistent-worker headroom probe. Filenames: `V1_01_easy_<thr>thr[_rate<R>]_run<N>_*.txt` (the `_rate<R>` segment is omitted when `R==1.0`). 150 files. |
| `rerun_2026_04_27_chi2_rate1_{on,off}/subscribe/` | same | 2026-04-27 | overridden (`true`/`false`) | 4 + 1 | V1_03_difficult subscribe @ rate 1.0, 3 reps each — chi2 A/B at the standard playback rate. 30 files per side. |
| `rerun_2026_04_27_chi2_rate2_{on,off}/subscribe/` | same | 2026-04-27 | overridden (`true`/`false`) | 4 + 1 | V1_03_difficult subscribe **@ rate 2.0**, 3 reps each — overload-case chi2 A/B (the scenario where recovery actually helps). 30 files per side. |

## RPi5 tags

> **File-layout note for the PWT-investigation rows.** The `rerun_2026_04_26_pwt_*` data on disk uses the legacy filename convention from the now-deleted `run_pwt_benchmark_v2.sh` — `{serial,sub}_run<N>_*.txt` directly under `results/rpi5/docker_humble/<tag>/`, with `_pose.txt` (TUM) trajectories. The "Reproduce" commands below use the consolidated orchestrator and write the standard convention (`<seq>_<thr>thr[_run<N>]_*.txt` with `_est.txt` state-dump trajectories). The two layouts are **analytically equivalent** — `parse_results.py` reads both — but a re-run will not bit-overwrite the archived files.

| Tag (under `results/rpi5/docker_humble/`) | Submodule | Date | chi2_recovery | Docker flags | Notes |
|---|---|---|---|---|---|
| `rerun_2026_04_26_pwt_baseline/` | `master-candidate` / `2a50450` (via Docker `openvins-humble:latest` rebuilt 2026-04-26) | 2026-04-26 | `false` | (none, only `-e HOME=/tmp` for `~/.ros/log` workaround — applied automatically by `docker_wrap`) | 2 serial reps + 10 subscribe reps × V1_01_easy stereo. The 2 serial reps were a determinism check (consumed by `docs/determinism.md` §6). Reproduce: `bash scripts/run_full_benchmark.sh -m subscribe -s V1_01_easy -t 4 -r 10 --docker openvins-humble:latest --tag rerun_2026_04_26_pwt_baseline`. |
| `rerun_2026_04_26_pwt_rtflags/` | same | 2026-04-26 | `false` | `--cap-add=SYS_NICE --ulimit rtprio=99 --ulimit memlock=-1 --cpuset-cpus=0-3` | Same as baseline plus RT scheduling flags. Reproduce: `bash scripts/run_full_benchmark.sh -m subscribe -s V1_01_easy -t 4 -r 10 --docker openvins-humble:latest --docker-flags '<RT-flags>' --tag rerun_2026_04_26_pwt_rtflags`. |
| `rerun_2026_04_26_pwt_maxinterval/` | same | 2026-04-26 | `false` | (same as baseline) | **Identical code/flags to `pwt_baseline` under `master-candidate`**; kept as a separate-session run for cross-session stability. |
| `rerun_2026_04_26_pwt_combined/` | same | 2026-04-26 | `false` | (same as `pwt_rtflags`) | **Identical to `pwt_rtflags`** under `master-candidate`; cross-session comparison. |
| `rerun_2026_04_26_pwt_final_maxinterval/`, `rerun_2026_04_26_pwt_final_combined/` | same | 2026-04-26 | `false` | (same as baseline / rtflags resp.) | Back-to-back rerun of `pwt_maxinterval` / `pwt_combined` — controlled session pair for between-session noise estimation. |
| `rerun_2026_04_26_paper/serial/` | same | 2026-04-26 | `false` | (none) | 5 Vicon × stereo + mono serial — RPi5 paper-repro for x86↔RPi5 cross-platform comparison. Reproduce: `bash scripts/run_full_benchmark.sh -m serial -c both -r 1 -s V1_01_easy,V1_02_medium,V1_03_difficult,V2_01_easy,V2_02_medium --docker openvins-humble:latest --tag rerun_2026_04_26_paper`. |

## Inherited config (all rows above)

Unless overridden in the "Notes" column, every run uses the YAML config at
`src/open_vins/config/euroc_mav/estimator_config.yaml`:

- `num_pts: 200`, `max_slam: 50`, `max_slam_in_update: 25`, `max_msckf_in_update: 40`
- `slam_chi2_recovery: false` (default since 2026-04-26)
- `num_opencv_threads: 4` (modified to `1` for the `1thr` rows by `make_bench_config()` in `scripts/bench_lib.sh`)
- `record_timing_information`, `record_timing_cpu_time`, `record_timing_thread_time`, `record_feature_counts`: all set to `true` in the temp config used at runtime so the 5-CSV output is produced; the source-tree YAML keeps them `false`.

EuRoC ground truth files used for ATE/RPE evaluation:
`src/open_vins/ov_data/euroc_mav/{V1_01_easy,V1_02_medium,V1_03_difficult,V2_01_easy,V2_02_medium,MH_01_easy,MH_02_easy,MH_03_medium,MH_04_difficult,MH_05_difficult}.txt`.

## File-format conventions

Every benchmark run writes 5 CSVs per rep:

| File suffix | Format | Clock / contents |
|---|---|---|
| `_wall.txt` | `# timestamp(s),tracking,propagation,msckf,slam,slam_delayed,retri_marg,total` | `boost::posix_time` wall-clock (ms) per VIO stage |
| `_cpu.txt` | same columns | `CLOCK_PROCESS_CPUTIME_ID` (ms) — sums CPU time across all threads |
| `_thread.txt` | same columns | `CLOCK_THREAD_CPUTIME_ID` (ms) — VIO thread alone |
| `_feats.txt` | `# timestamp(s),slam_feats_in_state,msckf_feats_used,slam_feats_updated,slam_feats_delayed_init,clones` | per-frame feature counts |
| `_est.txt` | full state dump (`timestamp qx qy qz qw px py pz vx vy vz bgx ...`) | trajectory estimate. Convert to TUM with `awk '!/^#/ && NF>=8 {print $1,$6,$7,$8,$2,$3,$4,$5}'` before invoking `ros2 run ov_eval error_singlerun`. The legacy RPi5 PWT tags use `_pose.txt` (TUM directly). |

The TUM conversion is critical — feeding the state-dump format directly to
`error_singlerun` reads quaternion components as if they were position,
producing wildly inflated ATE values (e.g. 1.94 m instead of 0.04 m for
V1_01_easy). `parse_results.py` does the conversion automatically; the
`*_est.txt`-based ATE numbers in the orchestrator's quick summary are
state-dump-format and not directly comparable to paper numbers.

## Archive

On 2026-04-27 the legacy `~/results/timing/x86/` tree (710 MB) and assorted
root-level orphans were moved to `~/results/_archive/2026_04_27/<original-relative-path>/`
to make room for the post-PWT re-collection campaign. Manifest at
`~/results/MIGRATION_2026_04_27.log`. Anything cited in this doc as
`results/x86/native_humble/rerun_2026_04_27_*/...` was produced fresh on
2026-04-27; everything else (pre-PWT data, single-rep `stereo/`/`mono/`
collections, `thread_rewrite/`, retired `bench_*` and `pwt_*` tags) is in the
archive and not directly cited from the analysis docs.

## Retired tags

These tag directories no longer exist under `results/` (they were either
deleted from git history or archived to `_archive/2026_04_27/`):

| Retired tag | Reason | Replacement |
|---|---|---|
| `results/timing/x86/serial/{stereo,mono}/<seq>.txt` (pre-tag, single-rep, no 3-clock) | Pre-PWT, instrumentation-incomplete | `rerun_2026_04_27_main/serial/<seq>_4thr[_mono]_*.txt` (V1_01/MH_03/V2_02) and `rerun_2026_04_27_v103/serial/V1_03_difficult_*` |
| `results/timing/x86/serial/thread_rewrite/`, `subscribe/thread_rewrite/`, `serial/sweep/thread_rewrite/` | Pre-PWT (outer `03fd95f`); cited as "before-PWT" comparison data | `rerun_2026_04_27_main/` (totals + mono baseline), `rerun_2026_04_27_rate_sweep/` (rate sweep), `rerun_2026_04_27_sweep/` (config sweep) |
| `results/{stereo,mono}/estimate_*.txt` (root-level paper-repro promotion, Mar 30) | Pre-orchestrator one-off; cited paths never matched on-disk reality | `rerun_2026_04_27_paper/serial/<seq>_4thr[_mono]_est.txt` |
| `results/timing/x86/serial/rerun_2026_04_2{1,3}/` and the matching subscribe/recovery/rate2 tags | Superseded by `rerun_2026_04_27_*` re-collection | `rerun_2026_04_27_main`, `rerun_2026_04_27_paper`, `rerun_2026_04_27_chi2_rate{1,2}_{on,off}` |
| `results/timing/x86/{serial,subscribe}/{bench_5rep_3clock,bench_persistent_worker,bench,sanity*,paper_repro,repro_eval,mh05_*,v2}/` | Pre-PWT-gating-fix or pre-orchestrator exploratory data; subtle non-determinism in the spurious-PWT period | None (exploratory; the canonical tags above are the references now) |
| `results/rpi5/{pwt_baseline,pwt_rtflags,pwt_maxinterval,pwt_combined,pwt_final_maxinterval,pwt_final_combined}/` (in git history at outer `8424c9e`) | Pre-`master-candidate` Docker images; cross-run timing variability was wider (5–25 ms range) due to image rebuild drift | `rerun_2026_04_26_pwt_*` (consolidated `openvins-humble:latest` image; cross-run variability ±0.08–0.15 ms) |
