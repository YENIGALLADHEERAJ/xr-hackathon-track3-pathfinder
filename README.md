# Track 03: Path Finder (submission bundle)

This bundle turns a degraded IMU stream into a 6-DoF trajectory, including the 5 seconds of dead reckoning after the IMU fails. It runs on a single thread, streams the input in constant memory and has no third-party dependencies.

There are two interchangeable implementations of the same algorithm. Their output is **byte-identical** (checked by `tools/parity.py`):

| | Entry point | Needs | Peak RAM (whole process) |
|---|---|---|---|
| **Primary** | `src/main.cpp` → `pathfinder` | any C++17 compiler | **4.1 MB** |
| Reference | `src/main.py` | Python 3.8+ (stdlib only) | 17.8 MB (13 MB is the interpreter itself) |

```
team_name.zip
├── estimated_trajectory.txt   # generated on the organizer's evaluation dataset (see below)
├── metadata.json              # run statistics incl. runtime and peak RSS
├── src/
│   ├── main.cpp               # entry point (C++17, no dependencies, built-in zip reader)
│   ├── main.py                # reference implementation (same output)
│   ├── build.sh  build.bat  CMakeLists.txt
│   └── README.md              # method, output contract, budgets
└── tools/                     # validation only (needs numpy); not used at inference
    ├── simulate.py  evaluate.py  tune.py  benchmark.py  test_contract.py  parity.py
    ├── fetch_euroc.py  degrade_real.py  eval_runs.py      # real-data validation
```

## Build and run

Linux / macOS:

```bash
sh src/build.sh
```

```bash
src/pathfinder path/to/evaluation_dataset.zip -o estimated_trajectory.txt --metadata metadata.json
```

Windows (with Visual Studio's C++ tools installed, `build.bat` finds them itself):

```bash
src\build.bat
```

```bash
src\pathfinder.exe path\to\evaluation_dataset.zip -o estimated_trajectory.txt --metadata metadata.json
```

Docker (Linux build with g++; the image also contains the Python reference):

```bash
docker build -t pathfinder .
```

```bash
docker run --rm -v "$PWD/data:/data" pathfinder /data/evaluation_dataset.zip -o /data/estimated_trajectory.txt --metadata /data/metadata.json
```

CMake (`cmake -S src -B build && cmake --build build --config Release`) also works. CI (`.github/workflows/ci.yml`) builds on Ubuntu (g++) and Windows (MSVC). On Linux it also runs the contract tests, C++/Python parity, the 8 MB and 20 ms budget checks, and the Docker image. The input can be the `.zip` (stored or deflate), the unzipped folder, or `imu0/data.csv` itself. `python src/main.py <same arguments>` produces the identical file.

Then zip the folder as `team_name.zip`, using your registered `CollegeName_TeamName`.

## Budgets

The C++ build was measured on Windows 11 with MSVC 19.51 (`/O2 /MT`) on a single thread. Peak RSS is the OS peak working set of the whole process, reported in `metadata.json`.

| Budget | Limit | 84 s flight (V1_02) | 30 min sequence | 30 min, zipped input |
|---|---|---|---|---|
| Peak RAM | ≤ 8 MB | **4.09 MB** | **4.09 MB** | **4.08 MB** |
| Compute per 1 s of IMU | ≤ 20 ms | **0.45 ms** | **0.43 ms** | **0.52 ms** |
| Threads / network | 1 / offline | 1 / offline | 1 / offline | 1 / offline |

Measured with the two-pass smoother on (the default). Memory doesn't grow with sequence length: rows are parsed and filtered one at a time, zip members are inflated through a 32 KB window, and the smoother streams fixed-size records through a temp file (about 110 bytes per IMU sample, on disk) using 1,024-record buffers. Most of the 4 MB is the fixed baseline of any Windows process. On Linux, expect a lower figure. The Python reference runs at 2.4 ms per IMU-second.

## Results on real EuRoC data

Two real Vicon-room flights by the same vehicle:

| Flight | Use | Duration | Path length | Mean speed | Download |
|---|---|---|---|---|---|
| **V1_01_easy** | parameter selection | 145 s | 58.6 m | 0.41 m/s | 3.6 MB |
| **V1_02_medium** | **held out**, never used for fitting | 84 s | 75.9 m | 0.91 m/s (max 2.2) | 637 MB |

`tools/fetch_euroc.py` fetches only the IMU and ground-truth CSVs. It uses HTTP range requests when the sequence zip is hosted on its own (V1_01), and a streamed inflate when it is nested inside a room archive (V1_02). `tools/degrade_real.py` then applies the track's degradation on top of the real sensor: noise ramping to about ×10, bias drift, spikes, frame drops, a 2 s loss and IMU death (rows cut or NaN). Ground truth is the EuRoC state estimate, interpolated onto every IMU stamp. All values are in metres, except rotation in °/s.

**V1_02_medium, held out:**

| Run | Stand-still ATE | **Path Finder ATE** | Checkpoint err | Blind 5 s drift | Dropout-window err | Rotation RPE |
|---|---|---|---|---|---|---|
| clean | 1.82 | **1.66** | 1.56 | 0.73 | – | **0.20** |
| degraded s10 (cut) | 1.82 | **1.61** | 1.49 | 0.63 | 1.25 | 0.78 |
| degraded s11 (NaN) | 1.82 | **1.72** | 1.42 | 0.83 | 1.52 | 0.75 |
| degraded s12 (cut) | 1.82 | **1.74** | 1.53 | 0.75 | 1.49 | 0.65 |
| degraded s13 (NaN) | 1.82 | **1.56** | 1.42 | 0.60 | 1.31 | 0.63 |
| degraded s14 (cut) | 1.82 | **1.60** | 1.48 | 0.73 | 1.55 | 0.60 |
| degraded s15 (NaN) | 1.82 | **1.66** | 1.59 | 0.70 | 1.67 | 1.24 |
| **mean** | **1.816** | **1.650 (−9.1%)** | | | | |

**V1_01_easy, used for selection:**

| Run | Stand-still ATE | **Path Finder ATE** | Checkpoint err | Blind 5 s drift | Dropout-window err | Rotation RPE |
|---|---|---|---|---|---|---|
| clean | 1.86 | **1.72** | 1.77 | 0.30 | – | **0.46** |
| degraded s0 (cut) | 1.86 | **1.71** | 1.78 | 0.29 | 2.37 | 0.69 |
| degraded s1 (NaN) | 1.86 | **1.68** | 1.66 | 0.29 | 1.84 | 0.68 |
| degraded s2 (cut) | 1.86 | **1.67** | 1.74 | 0.32 | 3.14 | 0.90 |
| degraded s3 (NaN) | 1.86 | **1.70** | 1.74 | 0.35 | 1.70 | 0.71 |
| degraded s4 (cut) | 1.86 | **1.74** | 1.79 | 0.33 | 3.37 | 0.76 |
| degraded s5 (NaN) | 1.86 | **1.63** | 1.50 | 0.25 | 1.76 | 0.78 |
| **mean** | **1.857** | **1.693 (−8.9%)** | | | | |

For reference, raw double integration of the same data is **139–1,282 m** off.

### How the result was reached

Each step is an improvement in mean ATE against the stand-still baseline. Only the rows marked as validated were confirmed on the held-out V1_02.

| Step | V1_01 | V1_02 (held out) |
|---|---|---|
| Synthetic-tuned filter | −1.2% | −1.5% |
| + gyro bias calibrated only at start-up (fixes 2.6°/s → 0.46°/s rotation) | −1.2% | −1.5% |
| + robust start-up (median outlier rejection) | | (fixes one run's 4.4°/s → 0.65°/s) |
| + less shrinkage, faster velocity fusion, rotor-drag aiding *(validated)* | −3.5% | −3.4% |
| **+ offline two-pass zero-phase smoothing** *(validated)* | **−8.9%** | **−9.1%** |

What each change did:

1. **Gyro bias was corrupted by rotor vibration.** That made slow flight look stationary, and in-flight bias updates soaked up real rotation. The bias is now calibrated once, at start-up. The result is **0.46°/s on clean V1_01**, the same as an oracle that integrates the gyro with the *true* bias (0.45°/s), and **0.20°/s on clean V1_02**.
2. **A spike in the first second could disable calibration** for the whole flight. Median-based outlier rejection at start-up fixes it.
3. **The output was over-shrunk.** The parameters tuned on synthetic data crushed the output trajectory to about 10 cm of extent, so it effectively stood still.
4. **Rotor-drag velocity aiding.** On a multirotor, sideways specific force is rotor drag, which is proportional to body velocity: a drift-free velocity signal from the IMU alone. A multirotor detector gates it, using the specific-force direction's spread about the thrust axis (about 5° on a multirotor, tens of degrees handheld). Self-calibrating the gain online failed: the integrated velocity it calibrates against tracks true velocity worse than drag does. So the gain is a vehicle constant fitted on V1_01. Its held-out benefit is modest.
5. **Offline two-pass smoothing** was the biggest single gain. The track is offline, so every estimate can use future samples as well as past ones. Forward and backward leaky velocity and position integrations are averaged: a zero-phase version of the same filter, without the lag that causal leaks put on real motion. It runs in three extra streaming passes over a temp file, so memory stays O(1). The post-failure blind segment keeps the forward dead reckoning, since there's nothing after it to smooth with. `tau_vs`=4 s and `tau_ps`=30 s were picked on V1_01; V1_02 independently agrees, with 20–80 s all within 0.4%.

Two choices I tested and rejected on held-out data: an automatic tuner's parameter set (better on training runs, not on V1_02), and holding velocity across gaps in the smoother (better on real data, much worse on synthetic).

## Results on synthetic degraded sequences (held-out seeds)

The organizer's degradation script and evaluation data aren't public yet, so `tools/simulate.py` also generates EuRoC-format sequences of two kinds: a quadrotor with thrust-coupled attitude, and handheld motion with free 3-D rotation, the closest stand-in for TUM-VI. Each has the same progressive degradation. Values are for seeds 11–13, which were never used for fitting.

| Sequence | Length | Stand-still baseline ATE | **Path Finder ATE** | Checkpoint err | Blind 5 s drift | Dropout-window err |
|---|---|---|---|---|---|---|
| handheld 11 (cut) | 120 s | 1.33 | **1.12** | 0.92 | 1.04 | 1.48 |
| handheld 12 (NaN) | 180 s | 1.20 | **1.09** | 1.07 | 1.72 | 1.00 |
| handheld 13 (cut) | 120 s | 1.24 | **1.19** | 1.11 | 2.42 | 2.65 |
| MAV 11 (cut) | 120 s | 1.31 | **1.15** | 1.16 | 1.52 | 0.62 |
| MAV 12 (NaN) | 180 s | 1.45 | **1.29** | 1.36 | 1.57 | 1.17 |
| MAV 13 (cut) | 120 s | 1.53 | **1.37** | 1.16 | 1.66 | 2.41 |
| **mean** | | **1.345** | **1.203 (−10.5%)** | | | |

ATE is the RMSE after SE(3) alignment, the same as `evo_ape -a`. The "stand-still" baseline is a trajectory that never moves, and it's the honest bar for IMU-only estimation. Gyro angle random walk alone turns into about 1 m of position error within 20 s at EuRoC noise levels, so position over minutes is largely unobservable from an IMU. Raw double integration therefore drifts by hundreds of metres to kilometres. Path Finder beats stand-still on **all 20 evaluation runs**, by 9–10% on average, with attitude at the gyro's physical limit.

## Verification

The commands below assume a work folder (`<workdir>`) of simulated sequences, which `tools/tune.py` creates.

Contract tests: row count, exact timestamps, NaN/zero/garbage rows, gaps, CRLF, stored and deflate zip. Run them against the C++ build, then against `main.py`:

```bash
python tools/test_contract.py --cpp
```

```bash
python tools/test_contract.py
```

Check C++ against Python, byte for byte:

```bash
python tools/parity.py <workdir>/val_*
```

Regenerate the data, retune and print the results table:

```bash
python tools/tune.py <workdir>
```

Real EuRoC check: fetch the CSVs (about 4 MB), degrade them, then score:

```bash
python tools/fetch_euroc.py https://huggingface.co/datasets/pepijn223/euroc-mirror/resolve/main/V1_01_easy.zip euroc/V1_01_easy
```

```bash
python tools/degrade_real.py euroc/V1_01_easy runs/V101_deg_s0 --seed 0
```

```bash
python tools/eval_runs.py runs --pure
```

## Known limitations

- Real-data validation covers two EuRoC flights from one room and one vehicle (V1_01 for selection, V1_02 held out). The Machine Hall sequences and a handheld TUM-VI sequence haven't been tested. The drag gain is specific to the EuRoC vehicle; on other multirotors it would need refitting, and on non-multirotor data the detector switches it off.
- IMU-only position is about 9% better than stand-still on real data. That's a solid relative gain, but absolute position error is still metre-level, as it must be without an external reference.
- The smoother needs about 110 bytes of temp disk per IMU sample (20 MB for 30 minutes at 200 Hz). If no temp directory is writable, it falls back to the causal single pass with a warning. `--no-smooth` forces that mode.
- The smoother trades a little dropout-window accuracy on fast synthetic handheld motion for clearly better overall accuracy. The full track objective improves in every dataset group.
- The 5 s post-failure poses are appended after the last row only when the file doesn't already contain the failure segment as invalid rows. Pass `--no-extrapolate` if the evaluator requires exactly one pose per input row.
- The built-in zip reader handles standard (non-ZIP64) archives with stored or deflate members. Archives over 4 GB should be unzipped first, then pass the folder.
