# Path Finder: Trajectory Estimation Under Sensor Degradation

**XR Hackathon 2026, Track 03.** Takes a degraded IMU stream (`imu0/data.csv`, EuRoC format or anything the organizers' `degrade_imu.py` produces) and writes a 6-DoF trajectory (`estimated_trajectory.txt`, TUM format) with **exactly one pose per input row**, including dead-reckoned poses for the post-failure blind window.

## Run

Python 3.8+ standard library only: no `pip install`, single file.

```bash
python main.py evaluation_dataset.zip -o estimated_trajectory.txt --metadata metadata.json
```

The input can be the zipped evaluation folder, the dataset folder (it searches for `imu0/data.csv`), or the CSV itself.

| Option | Meaning |
|---|---|
| `-o FILE` | Output trajectory (default `estimated_trajectory.txt`) |
| `--metadata FILE` | Write run statistics as JSON: runtime, detected input format, dropouts, spikes, ... |
| `--gyro-deg` | Gyro columns are in deg/s (only needed if the header doesn't say so) |
| `--no-smooth` | Single-pass causal output (disables the offline two-pass smoother) |
| `--extrapolate` | Append 5 s of poses after the last row. **Off by default**: the Track 3 FAQ forbids appended poses. |
| `--set key=val ...` | Override a tuning parameter (see `DEFAULTS`) |
| `--profile-memory` | Report the Python heap peak (tracemalloc; slower) |

## Output contract

- Space-separated `timestamp tx ty tz qx qy qz qw`, with one `#` header line.
- **Exactly one pose per input IMU row, in input order. Nothing is appended.** The timestamp is the row's time converted exactly to seconds (integer arithmetic, 9 decimals).
- Rows that are all-zero (the blind window written by `degrade_imu.py`), NaN, empty, out of range or frozen still get a pose, from dead reckoning. A duplicated row (same timestamp) gets the same pose again.
- **Frame:** z-up and gravity-aligned. The origin is the start position and yaw is 0 at the start. The ground-truth world frame can't be observed from an IMU alone, and evo's SE(3) alignment maps between them.

## Input format detection

Reported under `input_format` in the metadata.

| Property | How it's detected |
|---|---|
| Column order | The accelerometer triplet is the one with median norm about 1 g. EuRoC is gyro-first; `degrade_imu.py` documents accel-first. The header is cross-checked, not trusted blindly. |
| Timestamp unit | s / ms / µs / ns from the median spacing (same thresholds as `degrade_imu.py`), converted exactly to ns |
| Accelerometer in g | Median norm between 0.5 and 2 means g, converted to m/s² |
| Gyro in deg/s | `deg` in the gyro header names, or `--gyro-deg` |

Standard EuRoC input passes through unchanged, bit for bit.

## Method

The design is a streaming complementary filter. It reads one row, updates the state in O(1), writes the pose and keeps nothing else in memory.

1. **Robust front-end.** These cases are treated as dropouts:
   - NaN or empty fields
   - all-zero rows
   - saturated values (|ω| > 40 rad/s, |a| > 200 m/s²)
   - frozen sensors (the same row ≥ 3 times)

   An impulsive spike is replaced by the last accepted value when it jumps more than `k ×` the channel's running difference-noise. Timestamp gaps over 2.5 × the nominal period are bridged by blind propagation. Duplicate or out-of-order stamps keep the previous pose.
2. **Gravity alignment and bias at start.** The first second of data sets the initial roll and pitch from the shortest rotation that takes the mean accelerometer vector to +z. If the platform is static, it also sets the gyro bias and local |g|. Rows further than the spike thresholds from the per-channel median are left out first, so a single spike in that second can't void the static test and leave the bias uncalibrated for the whole flight. This handles the "unknown gravity alignment": no IMU mounting is assumed.
3. **Attitude.** Exact quaternion exponential integration of the bias-corrected gyro. A gravity correction (Mahony-style), weighted by `exp(-((|a|-g)/0.15g)²)`, runs while the platform is static. In motion it is off by default (`kp=0`): on a multirotor the accelerometer points along thrust, so tilting the attitude toward the accelerometer erases the very tilt that encodes horizontal acceleration. Tuning confirmed this. For handheld data, `--set kp=0.02` can be tried.
4. **Static segment and ZUPT.** Static detection uses the gyro energy and the deviation of |a| from g, with thresholds that grow with the noise estimate. While static: velocity → 0, gyro bias ← mean gyro, accel bias ← residual, and the gravity correction is stiffened. By default this applies **only to the initial static segment** (`static_hold_s=1e9`). On real EuRoC data, rotor vibration made slow flight pass the in-flight static test for 10.6 s, and each detection pulled the gyro bias toward the real rotation rate (2.6°/s rotation error). Calibrating once at start-up gives 0.46°/s, the same as an oracle that knows the true bias. For handheld data with real pauses, `--set static_hold_s=0.3` re-enables in-flight ZUPT.
5. **Translation as band-pass double integration.** The steps are:
   - Remove the gravity-compensated specific force's slow body-frame mean, which absorbs accel bias drift and residual tilt error.
   - Integrate into velocity with a leak `τ_v`, so a residual bias gives a bounded velocity error instead of a growing one.
   - Integrate into position with a weak prior `τ_p` toward the mission origin.

   - Optionally scale the output position by a noise-adaptive factor `β = (σ²_best / σ²_now)^(γ/2)`. It's off by default (`shrink_gamma=0`): with the official degradation and the smoother, trusting the IMU fully scored better.

   Raw double integration turns a bias into error growing with t², and gyro angle random walk alone tilts gravity into about 1 m of error in 20 s at EuRoC noise levels. Low-frequency position therefore can't be observed from an IMU alone. This cascade keeps the trajectory bounded and keeps the short-term motion the IMU does measure.
6. **Rotor-drag velocity aiding (multirotors).** Thrust acts along a body-fixed axis, so the accelerometer's component perpendicular to it is rotor drag, about `−k · v_body`. That is a drift-free velocity measurement from the IMU alone.
   - The thrust axis is a slow mean of the specific-force direction.
   - Lateral force is taken in the plane perpendicular to it, with a tracked offset removed, then low-passed.
   - The drag velocity is `drag_gain · f_lateral`. The gain (−2.45 s) was fitted on EuRoC V1_01 ground truth; a larger gain looked better on V1_01 but worse on the held-out V1_02, so it was rejected. The low-pass is 0.6 s. With the final fusion settings, drag is worth about 21% of the objective on V1_01.
   - The velocity leak pulls toward this drag velocity instead of toward zero.
   - A **multirotor detector** gates it. On a multirotor the specific force stays within about 5° (rms) of the thrust axis; handheld motion swings it by tens of degrees. The gate is fully on below 8° and off above 15°. Gate closed means exactly the plain filter.
   - Self-calibrating the gain online was tried and rejected: the integrated velocity it would calibrate against tracks true velocity worse than drag itself does.
7. **Dead reckoning (dropouts and post-failure).** With no valid measurement, the state is extrapolated from the last smoothed velocity and body rate, each decaying exponentially (`τ_vb`=0.5 s, `τ_wb`=0.1 s). This is the same code path for a single dropped frame, the 2-second signal loss and the final 5-second blind segment. When data comes back, integration resumes from the dead-reckoned state with no reset jump.
8. **Offline two-pass smoothing (default).** The track is offline, so each estimate can use future samples as well as past ones.
   - The forward pass above streams one fixed-size record per filter step: dt, gravity-free world acceleration, drag target velocity, and static and drag flags. It also writes one per output row: timestamp, attitude, shrink factor and forward position. All of this goes to a temp file.
   - Pass B1 integrates velocity forward. Pass B2 reads the file **backwards**, integrating velocity backward in time, averaging it with the forward velocity, and integrating position backward. Pass C integrates position forward, averages it with the backward position, and writes the poses.
   - The result is a zero-phase version of the same leaky filter (`tau_vs`=0.75 s, `tau_ps`=15 s), without the lag the causal leaks put on real motion.
   - Attitude is unchanged; it's already at the gyro's limit. After the last real measurement (the post-failure blind segment), the forward dead reckoning is kept, anchored to the smoothed position at failure.
   - Memory stays O(1). The temp file holds about 110 bytes per IMU sample and is read in 1,024-record chunks. If no temp directory is writable, the run falls back to the causal pass with a warning.

## Budgets

Measured on Windows 11 with Python 3.11 on real EuRoC flights degraded with `degrade_imu.py`, two-pass smoother on (the default). The Track 3 FAQ measures the 8 MB at heap level, not process RSS.

| Budget | Limit | Measured |
|---|---|---|
| Peak RAM (heap: data structures, buffers, estimation) | ≤ 8 MB | **1.6–2.0 MB**, constant in sequence length (streaming) |
| Compute per 1 s of IMU | ≤ 20 ms | **2.4–2.5 ms** |
| Threads | 1 | 1 |

## Validation tools (`tools/`)

| Script | Purpose |
|---|---|
| `simulate.py` | Generates EuRoC-style degraded sequences (MAV and handheld profiles) with matching ground truth. The degradation is progressive noise ×10, bias random walk and drift, spikes, frame drops, a 2 s signal loss, and IMU death (rows cut, or NaN). |
| `evaluate.py` | evo-equivalent ATE (SE(3) Umeyama), RPE over 1 s, plus checkpoint, blind-5 s and dropout-window errors. |
| `tune.py` | Coordinate-descent parameter tuning on train seeds, reported on held-out seeds. |
| `benchmark.py` | Runtime per IMU-second, Python heap peak and whole-process peak RSS. |
| `fetch_euroc.py` | Fetches only the IMU and ground-truth CSVs of a EuRoC sequence (HTTP range requests, or a streamed inflate for nested zips). |
| `make_official_runs.py` | Applies the organizers' `degrade_imu.py` to a real EuRoC flight in three input modes, writing ground truth and metadata for `evaluate.py`. |
| `degrade_real.py` | My own degradation model applied to real EuRoC data (used before the official script was released). |
| `eval_runs.py` | Scores a folder of runs against the stand-still and raw-strapdown baselines. |
| `test_contract.py` | 24 output-contract checks (row count, formats, failure rows, dropouts). |
