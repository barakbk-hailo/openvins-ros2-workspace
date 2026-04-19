# Subscribe-Mode Reliability: SLAM Recovery

## Problem

When running OpenVINS in subscribe mode (`run_subscribe_msckf`) **without the fix
described below**, the SLAM feature state can collapse to zero within the first
~50 frames and never recover.

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

## Root cause

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

## Root-cause fix: persistent worker thread

The architectural root cause — per-frame `detach()` thread dispatch in
`callback_inertial` — is addressed by the **persistent worker thread** change
(see [persistent-worker.md](persistent-worker.md)). That fix eliminates the
TOCTOU race, dangling reference UB, non-deterministic IMU triggering, and
per-frame thread churn. With it, subscribe mode runs at serial speed and the
SLAM collapse no longer occurs.

## Defense-in-depth: SLAM recovery mechanism

The SLAM recovery mechanism below remains as a safety net. With the persistent
worker thread, it rarely activates — but it protects against edge cases where
SLAM features might dip due to other factors (difficult sequences, sensor noise,
real hardware timing).

**File:** `ov_msckf/src/core/VioManager.cpp` (before the `updaterSLAM->delayed_init()` call)

When the SLAM feature count drops below `max_slam / 4` (default: 12 out of 50),
the chi-squared multiplier for `delayed_init` is temporarily increased by 3x. This
relaxes the consistency gate, allowing features to enter the SLAM state even if
their residuals are slightly elevated from drift during the low-feature period.
Once the SLAM state recovers above the threshold, the gate returns to the
configured value.

```cpp
// SLAM recovery: relax chi-squared gate when SLAM state is critically low
double original_chi2 = updaterSLAM->_options_slam.chi2_multipler;
int slam_recovery_threshold = state->_options.max_slam_features / 4;
if (state->_options.max_slam_features > 0 &&
    (int)state->_features_SLAM.size() < slam_recovery_threshold) {
  updaterSLAM->_options_slam.chi2_multipler = original_chi2 * 3.0;
}
updaterSLAM->delayed_init(state, feats_slam_DELAYED);
updaterSLAM->_options_slam.chi2_multipler = original_chi2;
```

This is a conservative change:
- Only activates when the SLAM state is critically low (<25% of max)
- Only affects the `delayed_init` gate, not the SLAM update itself
- Automatically deactivates once features recover
- Does not change behavior in serial mode (SLAM state never drops that low)

## Results

### Initial validation (V2_02_medium, 10 runs)

Tested on V2_02_medium, stereo, 10 subscribe runs (5 x 4-thread + 5 x 1-thread),
compared against a clean baseline without the recovery mechanism:

| Metric | Without recovery | With recovery |
|--------|-----------------|---------------|
| Degradation rate | 3/10 (30%) | **0/10 (0%)** |
| ATE failures (diverged) | 1/10 | **0/10** |
| Avg SLAM feature range | 2.3 - 36.2 | **34.0 - 36.0** |
| ATE position RMSE range | 2.088m - FAILED | **2.089 - 2.102m** |
| Serial ATE reference | 2.101m | 2.099m |

*Source: ad-hoc 10-run V2_02_medium baseline (pre-recovery debugging); the numbers are preserved here for the regression comparison but the raw CSVs were superseded by `results/timing/x86/subscribe/bench_5rep_3clock/V2_02_medium_*_run{1..5}_{est,feats}.txt` (with recovery enabled).*

### Full benchmark (3 sequences × 5 reps, 30 runs total)

Validated across V1_01_easy, MH_03_medium, and V2_02_medium with 5 repetitions
per configuration (see [benchmark-analysis.md](benchmark-analysis.md) for details):

| Metric | Result |
|--------|--------|
| Total subscribe runs | 30 |
| SLAM collapses | **0** |
| ATE within 0.02m of serial | **All runs** (where ov_eval didn't crash) |
| RPE (8m) within 0.07m of serial | **All runs** |
| Worst-case SLAM dip | MH_03 avg SLAM = 26.0 (still produced ATE within 0.004m and RPE within 0.13m of serial) |

*Source: `results/timing/x86/subscribe/bench_5rep_3clock/{V1_01_easy,MH_03_medium,V2_02_medium}_{1,4}thr_run{1..5}_{est,feats}.txt` vs `results/timing/x86/serial/bench_5rep_3clock/*_{1,4}thr_{est,feats}.txt`; see [benchmark-analysis.md](benchmark-analysis.md) for per-run numbers.*

The recovery mechanism maintains accuracy identical to serial mode as measured by
both global trajectory error (ATE) and local consistency (RPE at 8-40m segments).

## Other changes

These changes were made alongside the SLAM recovery work:

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

**File:** `run_timing_combined.sh`, `run_timing_subscribe.sh`

Subscribe test scripts now kill any stale `run_subscribe_msckf` processes before
and after each run. Stale nodes on the same DDS domain steal messages from active
subscribers, causing non-deterministic data loss — a critical issue we discovered
during testing that invalidated several earlier measurement batches.
