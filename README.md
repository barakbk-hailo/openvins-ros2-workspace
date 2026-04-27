# OpenVINS ROS 2 Workspace

This repo ([NadavHHailo/openvins-ros2-workspace](https://github.com/NadavHHailo/openvins-ros2-workspace))
is a deployment workspace for our fork of OpenVINS
([NadavHHailo/open_vins](https://github.com/NadavHHailo/open_vins)) running with ROS 2 Humble
on Ubuntu 22.04 or ROS 2 Jazzy on Ubuntu 24.04. No GPU required — OpenVINS is a CPU-based MSCKF/EKF
algorithm using OpenCV and Eigen.

Our fork adds:
- **Serial deterministic VIO node** (`ros2_serial_msckf`) — ROS 2 port of the ROS 1 serial reader. Processes bag frames sequentially with blocking updates, eliminating message drops from ROS 2 middleware. Produces bit-identical results between runs on the same platform.
- **Persistent worker thread** — replaces the upstream per-frame `detach()` dispatch in subscribe mode with a single persistent processing thread. Fixes a TOCTOU race, dangling-reference UB, and non-deterministic IMU triggering. Reduces subscribe-mode overhead from 2× serial to 1× serial.
- **SLAM recovery mechanism** — relaxes the chi-squared gate for delayed feature init when SLAM features drop below 25% of max, preventing an irrecoverable empty-state feedback loop (defense-in-depth).
- **3-clock timing instrumentation** — process CPU (`CLOCK_PROCESS_CPUTIME_ID`), thread CPU (`CLOCK_THREAD_CPUTIME_ID`), and wall-clock timing recorded per frame in separate CSVs. Also records per-frame feature counts (SLAM, MSCKF, delayed init). All disabled by default.
- **Configurable `multi_threading_subs`** — moved from hardcoded to a YAML parameter, allowing async vs inline VIO dispatch without recompiling.
- **Custom RPE segment lengths** — `error_singlerun` accepts optional segment lengths from the command line.
- **Launch file improvements** — `filepath_est`/`filepath_std` args for configurable output paths; `on_exit=Shutdown()` for clean exit in automated benchmark loops.
- **Docker images** for ROS 2 Humble (RPi5 / Debian Trixie) and ROS 2 Jazzy (WIP).
- **Benchmarking scripts** — orchestrate serial/subscribe runs across sequences, config sweeps, and 3-clock collection. Stored results (CSV timing + trajectory estimates) for x86 and RPi5.

## Quick install

```bash
git clone --recursive git@github.com:NadavHHailo/openvins-ros2-workspace.git ~/workspace/catkin_ws_ov
cd ~/workspace/catkin_ws_ov
bash install.sh
```

## Workspace structure

```
catkin_ws_ov/
├── src/open_vins/              fork submodule (algorithm + nodes + launch files)
├── install.sh                  one-shot Ubuntu 22.04 / 24.04 setup (auto-detects)
├── scripts/
│   ├── bench_lib.sh            shared library (arch detection, config templating, docker_wrap)
│   ├── run_full_benchmark.sh   orchestrator: serial+subscribe × seqs × threads × cams × reps × rates × native/docker
│   ├── run_timing_sweep.sh     Phase 2: config sensitivity sweeps on V1_01_easy serial
│   ├── record_poses.py         subscribe-mode pose recorder (manual evaluation)
│   └── parse_results.py        post-hoc results aggregator (cross-run timing/SLAM/ATE stats)
├── results/
│   ├── stereo/  mono/          x86 EuRoC trajectory estimates (paper reproduction)
│   ├── rpi5/stereo/            RPi5 EuRoC trajectory estimates
│   └── timing/{x86,rpi5}/      per-frame timing CSVs (serial + subscribe)
└── docs/                       (see index below)
```

Every committed timing CSV and trajectory estimate is referenced from the
docs below. Tables that aggregate data from `results/` carry a
`*Source: ...*` citation line identifying the underlying file(s); see
[docs/data-provenance.md](docs/data-provenance.md) for the canonical
tag → (platform, submodule commit, config) lookup.

## Recent changes

- **`slam_chi2_recovery` default `false`** (in `config/euroc_mav/estimator_config.yaml`). Leave at the default for offline serial replay and paper-repro reproducibility — the always-on chi2 relaxation interfered with stereo init on dark sequences (MH_05_difficult). Set to `true` for subscribe-mode deployment at >1× realtime under load (V1_03_difficult @ rate 2.0 shows ATE 3.7 m with `true` vs >50 m collapse with `false` in 2/3 runs). See [docs/determinism.md §4](docs/determinism.md#4-optional-safety-net-slam-recovery-mechanism).
- **`--slam-chi2-recovery <true|false>`** CLI flag on `scripts/run_full_benchmark.sh` and `scripts/run_timing_sweep.sh` for ad-hoc overrides without editing the YAML.
- **`--rate <csv>` sweep dimension** on `scripts/run_full_benchmark.sh` — single invocation runs the matrix once per rate (e.g. `--rate 1.0,2.0,5.0` for Phase-3 rate-feasibility). Replaces the dedicated `run_timing_subscribe.sh` script (deleted).
- **`--docker <image>` / `--docker-flags '<args>'`** on `scripts/run_full_benchmark.sh` — runs the OpenVINS launches inside a Docker container instead of natively. Same orchestrator handles RPi5+Docker (where the PWT investigation lives) and any future x86+Docker workflows. Replaces the dedicated `run_pwt_benchmark_v2.sh` and `run_pwt_final_ab.sh` scripts (deleted). RT-flag comparison is reproduced via two invocations differing only in `--docker-flags` — the exact flags are documented in `docs/rpi5-benchmarking.md`.
- **Cross-platform `RESULTS_BASE`** — auto-detected from `uname -m` (`x86_64` → `results/timing/x86`, `aarch64` → `results/timing/rpi5`). Override via `--results-base` or the `RESULTS_BASE` env var.
- **Serial-mode timing improved 8-15 %** — the persistent-worker thread is now properly gated on `use_multi_threading_subs`, so it's no longer spawned in serial mode. Also makes serial bit-deterministic across reps.
- **Paper-reproduction estimates regenerated** under the consolidated `master-candidate` submodule (`results/{stereo,mono}/estimate_*.txt`); ATE values bit-reproduce the prior committed numbers for every sequence × mode.
- **Latest benchmark tags:** `rerun_2026_04_23` (x86 main suite + paper repro + chi2 A/B) and `rerun_2026_04_26_pwt_*` + `rerun_2026_04_26_paper` (RPi5 PWT variants + cross-platform paper repro). Retired tags (`bench_5rep_3clock`, `bench_persistent_worker`, `pwt_*`) are removed from `results/` but preserved in git history.

## Documentation

Follow the flow top-to-bottom — each doc builds on the previous ones.

| Guide | Description |
|---|---|
| [Installation](docs/installation.md) | Native build on Ubuntu 22.04 (ROS 2 Humble) or Ubuntu 24.04 (ROS 2 Jazzy) |
| [Running](docs/running.md) | Run the VIO pipeline on EuRoC — serial (deterministic) and subscribe (realtime) modes |
| [Evaluation](docs/evaluation.md) | ATE/RPE benchmarks vs the OpenVINS paper + reproduction script |
| [Determinism](docs/determinism.md) | Subscribe-mode determinism: persistent worker thread root-cause fix + SLAM recovery safety net + other fork changes |
| [Timing](docs/timing.md) | Per-component timing breakdown, config sensitivity, realtime feasibility (x86) |
| [Benchmark Analysis](docs/benchmark-analysis.md) | 3-clock timing, paper comparison, accuracy and consistency under subscribe mode (x86) |
| [RPi5 Setup](docs/rpi5-setup.md) | Build on RPi5 (Docker or native) + run the EuRoC benchmark; live-sensor deployment is WIP |
| [RPi5 Benchmarking](docs/rpi5-benchmarking.md) | Phase 4 RPi5 timing, accuracy vs x86 (NEON/AVX determinism caveat), config sweeps, subscribe mode, WIP list |
| [Data Provenance](docs/data-provenance.md) | Canonical lookup: each `results/` tag → (platform, submodule commit, config, threads, dates). Cross-reference for every `*Source: ...*` citation in the docs above. |
