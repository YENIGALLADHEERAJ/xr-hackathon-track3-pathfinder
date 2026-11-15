# Path Finder: Trajectory Estimation Under Sensor Degradation

**XR Hackathon 2026, Track 03.** Takes a degraded EuRoC-format IMU stream (`imu0/data.csv`) and produces a 6-DoF trajectory (`estimated_trajectory.txt`, TUM format). It writes one pose per IMU frame, then 5 seconds of dead-reckoned poses after the IMU fails.

## Build and run

The primary implementation is `main.cpp` (C++17, no dependencies, about 3.8 MB peak RAM). `main.py` is a reference port that produces **byte-identical** output, using the Python 3.8+ standard library only.

Build on Linux / macOS / MinGW:

```bash
sh build.sh
```

Or build on Windows with MSVC (it finds Visual Studio itself):

```bash
build.bat
```

Or build with CMake:

```bash
cmake -S . -B build && cmake --build build --config Release
```

Then run:

```bash
./pathfinder path/to/imu0/data.csv -o estimated_trajectory.txt
```

The input can be the CSV itself, the dataset folder (it searches for `imu0/data.csv`), or the zipped evaluation folder (stored or deflate, read by a built-in streaming inflate):

```bash
./pathfinder evaluation_dataset.zip -o estimated_trajectory.txt --metadata metadata.json
```

The Python reference takes the same arguments:

```bash
python main.py evaluation_dataset.zip -o estimated_trajectory.txt
```

| Option | Meaning |
|---|---|
| `-o FILE` | Output trajectory (default `estimated_trajectory.txt`) |
| `--metadata FILE` | Write run statistics (runtime, peak RSS, dropouts, spikes, ...) as JSON |
| `--no-extrapolate` | Don't append the 5 s post-failure poses after the last IMU row (see below) |
| `--no-smooth` | Single-pass causal output (disables the offline two-pass smoother) |
| `--set key=val ...` | Override a tuning parameter (see `Params` in `main.cpp` / `DEFAULTS` in `main.py`) |
| `--profile-memory` | Python only: report Python heap peak (tracemalloc) |

## Output contract

- Space-separated `timestamp tx ty tz qx qy qz qw`, with one `#` header line.
- **Exactly one pose per input IMU row, in input order.** The timestamp is the row's nanosecond stamp converted exactly to seconds (integer arithmetic, 9 decimals, no float rounding). Rows that are NaN, empty, all-zero, out of range or frozen still get a pose, which comes from dead reckoning.
- **IMU failure.** If the file ends in ≥ 4 s of invalid rows, the failure segment is already in the file and gets dead-reckoned in place. Otherwise the IMU stopped reporting, so 5 s of poses at the nominal IMU rate are appended after the last row. evo matches poses by timestamp, so these extra poses only add to the score on the blind tier. Pass `--no-extrapolate` if the evaluator requires the row count to equal the input exactly.
- **Frame.** z-up and gravity-aligned. The origin is the start position and yaw is 0 at the start. The ground-truth world frame (position and heading) can't be observed from an IMU alone, so score with SE(3) alignment: `evo_ape tum gt.txt estimated_trajectory.txt -a`.

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

   - Scale the output position by a noise-adaptive factor `β = (σ²_best / σ²_now)^(γ/2)`, so integrated motion is trusted less as the sensor degrades.

   Raw double integration turns a bias into error growing with t², and gyro angle random walk alone tilts gravity into about 1 m of error in 20 s at EuRoC noise levels. Low-frequency position therefore can't be observed from an IMU alone. This cascade keeps the trajectory bounded and keeps the short-term motion the IMU does measure.
6. **Rotor-drag velocity aiding (multirotors).** Thrust acts along a body-fixed axis, so the accelerometer's component perpendicular to it is rotor drag, about `−k · v_body`. That is a drift-free velocity measurement from the IMU alone.
   - The thrust axis is a slow mean of the specific-force direction.
   - Lateral force is taken in the plane perpendicular to it, with a tracked offset removed, then low-passed.
   - The drag velocity is `drag_gain · f_lateral` (gain −2.45 s, fitted on EuRoC V1_01 and validated on the held-out V1_02).
   - The velocity leak pulls toward this drag velocity instead of toward zero.
   - A **multirotor detector** gates it. On a multirotor the specific force stays within about 5° (rms) of the thrust axis; handheld motion swings it by tens of degrees. The gate is fully on below 8° and off above 15°. Gate closed means exactly the plain filter.
   - Self-calibrating the gain online was tried and rejected: the integrated velocity it would calibrate against tracks true velocity worse than drag itself does.
7. **Dead reckoning (dropouts and post-failure).** With no valid measurement, the state is extrapolated from the last smoothed velocity and body rate, each decaying exponentially (`τ_vb`, `τ_wb`). This is the same code path for a single dropped frame, the 2-second signal loss and the final 5-second blind segment. When data comes back, integration resumes from the dead-reckoned state with no reset jump.
8. **Offline two-pass smoothing (default).** The track is offline, so each estimate can use future samples as well as past ones.
   - The forward pass above streams one fixed-size record per filter step: dt, gravity-free world acceleration, drag target velocity, and static and drag flags. It also writes one per output row: timestamp, attitude, shrink factor and forward position. All of this goes to a temp file.
   - Pass B1 integrates velocity forward. Pass B2 reads the file **backwards**, integrating velocity backward in time, averaging it with the forward velocity, and integrating position backward. Pass C integrates position forward, averages it with the backward position, and writes the poses.
   - The result is a zero-phase version of the same leaky filter (`tau_vs`=4 s, `tau_ps`=30 s), without the lag the causal leaks put on real motion. ATE improves by about 2.5× relative to stand-still: −3.4% → −9.1% on the held-out V1_02 flight.
   - Attitude is unchanged; it's already at the gyro's limit. After the last real measurement (the post-failure blind segment), the forward dead reckoning is kept, anchored to the smoothed position at failure.
   - Memory stays O(1). The temp file holds about 110 bytes per IMU sample and is read in 1,024-record chunks. If no temp directory is writable, the run falls back to the causal pass with a warning.

## Budgets

Measured on Windows 11 with a single thread and the two-pass smoother on (the default). Peak RSS is the OS peak working set of the whole process.

| Budget | Limit | C++ `pathfinder` (MSVC `/O2 /MT`) | Python `main.py` (3.11) |
|---|---|---|---|
| Whole-process peak RAM | ≤ 8 MB | **4.09 MB**, the same for 84 s and 30 min of data, zipped or not | about 18 MB (13 MB is the bare interpreter; the algorithm's heap peak is 1.7 MB) |
| Compute per 1 s of IMU | ≤ 20 ms | **0.43–0.52 ms** | 2.4 ms |
| Threads | 1 | 1 | 1 |

## Validation tools (`tools/`)

| Script | Purpose |
|---|---|
| `simulate.py` | Generates EuRoC-style degraded sequences (MAV and handheld profiles) with matching ground truth. The degradation is progressive noise ×10, bias random walk and drift, spikes, frame drops, a 2 s signal loss, and IMU death (rows cut, or NaN). |
| `evaluate.py` | evo-equivalent ATE (SE(3) Umeyama), RPE over 1 s, plus checkpoint, blind-5 s and dropout-window errors. |
| `tune.py` | Coordinate-descent parameter tuning on train seeds, reported on held-out seeds. |
| `benchmark.py` | Runtime per IMU-second, Python heap peak and whole-process peak RSS. |
