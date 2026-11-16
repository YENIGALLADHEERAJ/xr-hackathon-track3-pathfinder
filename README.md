# Track 03: Path Finder (submission bundle)

`src/main.py` turns a degraded IMU stream into a 6-DoF trajectory, including dead reckoning through the 5 seconds after the IMU fails. It is a **standalone Python script** (standard library only, single thread, offline), as the Track 3 FAQ requires.

```
team_name.zip
├── estimated_trajectory.txt   # generated on the organizer's evaluation dataset (see below)
├── metadata.json              # run statistics: runtime, detected input format, dropouts, ...
└── src/
    ├── main.py                # entry point (Python 3.8+, stdlib only)
    └── README.md              # method, output contract, budgets
```

The repository also contains `tools/` (validation only; numpy), a `Dockerfile`, CI, and `DEMO_SCRIPT.md`.

## Run

```bash
python src/main.py path/to/evaluation_dataset.zip -o estimated_trajectory.txt --metadata metadata.json
```

The input can be the `.zip`, the unzipped folder, or `imu0/data.csv` itself. No `pip install` is needed.

Docker:

```bash
docker build -t pathfinder .
```

```bash
docker run --rm -v "$PWD/data:/data" pathfinder /data/evaluation_dataset.zip -o /data/estimated_trajectory.txt --metadata /data/metadata.json
```

Then zip the bundle as `team_name.zip`, using your registered `CollegeName_TeamName`.

## Compliance with the official Track 3 package

| Rule (slides 14–17 and the FAQ) | How `main.py` meets it |
|---|---|
| Standalone Python script; `main.cpp` not accepted | Python 3 stdlib only, one file |
| **Exactly one pose per IMU frame; do not append poses** | One output row per input row, in input order. The blind window's rows (all-zero in `degrade_imu.py`, or NaN) are dead-reckoned in place. Appending is opt-in via `--extrapolate`, off by default. |
| Timestamps ns → s | Exact integer arithmetic, 9 decimals |
| Dropouts are missing rows | Timestamp gaps are bridged by dead reckoning; every remaining row gets its pose |
| Estimate from the IMU file only | No other input is read |
| Offline; future samples may be used | Two-pass zero-phase smoother (see `src/README.md`) |
| ≤ 8 MB peak RAM, **measured at heap level** | **0.45–0.52 MB** heap peak (tracemalloc), constant in sequence length |
| ≤ 20 ms compute per 1 s of IMU, one thread | **2.3–2.7 ms** per IMU-second, single thread (the upper figure includes the rest-window pre-pass) |
| SE(3)-aligned ATE and RPE (evo) | Output frame: z-up, gravity aligned, starting at the origin |

### Input formats handled

The organizers' `degrade_imu.py` documents its columns as `timestamp, accel(3), gyro(3)`, while the slide-15 input spec (EuRoC) is `timestamp, gyro(3), accel(3)`. The script also accepts timestamps in s/ms/µs/ns and accelerometer data in g. `main.py` detects all of this from the data, and the result is reported under `input_format` in `metadata.json`:

| Property | Detection |
|---|---|
| Column order | The accelerometer triplet is the one whose median norm is about 1 g. The header is cross-checked. |
| Timestamp unit | From the median sample spacing (same thresholds as `degrade_imu.py`), converted exactly to ns |
| Accelerometer in g | Median norm between 0.5 and 2 means g, converted to m/s² |
| Gyro in deg/s | From the header (`deg`), or the `--gyro-deg` flag |

The script's other degradations are handled by the front-end:
- **Blind window:** its all-zero rows count as invalid.
- **Stuck samples:** three or more identical rows count as frozen.
- **Duplicated rows:** a repeated timestamp gets its own, identical pose.
- **Spikes:** gated against the running noise level.

## Results with the official degradation script

Four real sequences covering both datasets the jury uses: two Vicon-room flights, a Machine Hall flight in a much larger space, and a handheld TUM-VI recording with a different IMU. Each was degraded with the organizers' `degrade_imu.py` using `tools/make_official_runs.py`, giving 13 runs per sequence:
- **Severity:** `--max-level` 1.0 and 0.6, `--gamma` 1.7 and 1.0, plus one run with an extra 2 s dropout.
- **Input modes:**
  - `euroc`: the intended degradation, in slide-15 column order.
  - `accelfirst`: left in the script's own column order.
  - `direct`: the script applied straight to EuRoC files, so the gyro gets accelerometer-level noise and spikes.

| Sequence | Platform | Speed | Stand-still spread | Role |
|---|---|---|---|---|
| EuRoC V1_01_easy | multirotor, Vicon room | 0.41 m/s | 1.86 m | parameter selection |
| EuRoC V1_02_medium | multirotor, Vicon room | 0.91 m/s | 1.82 m | held out for parameter fitting |
| EuRoC MH_04_difficult | multirotor, large hall | 0.97 m/s | 6.94 m | exposed the bias bug (see below) |
| TUM-VI room1 | **handheld**, different IMU | 1.07 m/s | 1.01 m | exposed the drag-gating bug (see below) |

| Sequence | Mode | Stand-still ATE | **Path Finder ATE** | Checkpoint err | Blind 5 s drift | Dropout-window err | Rotation RPE |
|---|---|---|---|---|---|---|---|
| V1_01 | euroc | 1.857 | **1.522 (−18.0%)** | 1.406 | 0.296 | 1.248 | 0.64 °/s |
| | accelfirst | 1.857 | **1.519 (−18.2%)** | 1.405 | 0.297 | 1.313 | 0.63 °/s |
| | direct | 1.857 | **1.463 (−21.2%)** | 1.397 | 0.330 | 1.717 | 2.12 °/s |
| V1_02 | euroc | 1.816 | **1.473 (−18.9%)** | 1.148 | 0.680 | 1.588 | 0.75 °/s |
| | accelfirst | 1.816 | **1.458 (−19.7%)** | 1.155 | 0.692 | 1.322 | 0.69 °/s |
| | direct | 1.816 | **1.440 (−20.7%)** | 1.128 | 0.702 | 1.277 | 0.69 °/s |
| MH_04 | euroc | 6.942 | **5.609 (−19.2%)** | 4.981 | 0.427 | 6.752 | 0.23 °/s |
| | accelfirst | 6.942 | **5.572 (−19.7%)** | 4.894 | 0.422 | 6.540 | 0.23 °/s |
| | direct | 6.942 | **5.641 (−18.7%)** | 4.922 | 0.774 | 6.401 | 1.45 °/s |
| TUM-VI room1 | euroc | 1.013 | **0.836 (−17.5%)** | 0.763 | 0.832 | 0.771 | 1.78 °/s |
| | accelfirst | 1.013 | **0.831 (−18.0%)** | 0.760 | 0.841 | 0.781 | 1.78 °/s |
| | direct | 1.013 | **0.835 (−17.6%)** | 0.770 | 0.860 | 0.800 | 1.84 °/s |

Values are means over the runs in each group, in metres except rotation. **Every group beats the stand-still baseline by 17.5–21.2%.** ATE is the RMSE after SE(3) alignment, the same as `evo_ape -a`. The stand-still baseline is a trajectory that never moves, the honest bar for IMU-only estimation; raw double integration of the same data is 132–1,511 m off. Ground truth is the EuRoC state estimate or the TUM-VI motion capture on every IMU stamp, excluding stamps inside mocap dropouts (TUM-VI has gaps of up to 1 s).

**What was fitted where.** The smoother, shrinkage and drag parameters were selected on V1_01 alone and confirmed on V1_02. MH_04 and TUM-VI were brought in as unseen data. They exposed the two bugs described below, whose fixes are general detectors with physically motivated thresholds, not fitted parameters. The one parameter chosen across all four sequences is the blind-segment velocity decay (0.25 s). So MH_04 and TUM-VI are no longer pure holdouts, and the next validation step would be another unseen sequence (for example MH_05 or TUM-VI room2).

**Leave-one-sequence-out check of the position parameters.** The smoother's position prior (`tau_ps`) and velocity fusion (`tau_vs`) were re-swept on all four sequences, which disagree: the Vicon room wants a tighter prior, the large hall a looser one. For each sequence, parameters chosen on the other three made the held-out sequence **worse in 3 of 4 cases (+8% to +16% objective)**, while the current values (`tau_ps` 15 s, `tau_vs` 0.75 s) are the best single choice across all four. They sit in the robust middle and are kept; re-tuning per dataset would overfit.

### How it got there

| Step | V1_02 ATE vs stand-still |
|---|---|
| First version, tuned on synthetic data | −1.5% |
| + gyro bias calibrated only at rest, robust start-up | −1.5% (rotation 2.6 → 0.46 °/s) |
| + less shrinkage, rotor-drag velocity aiding | −3.4% |
| + offline two-pass zero-phase smoother | −9.1% |
| + **official package**: format detection, no appended rows, retuned on `degrade_imu.py` data | **−18.9%** |

The first four rows were measured with my own degradation model, the last with the organizers'.

| Fix from the new sequences | Before | After |
|---|---|---|
| Drag gated on rotor vibration (TUM-VI, handheld) | **+74.5%** (worse than stand-still) | **−17.5%** |
| Gyro bias from a detected rest window anywhere in the file (MH_04) | −12.0%, rotation 4.59 °/s | **−19.2%**, rotation **0.23 °/s** |

Key findings along the way:
1. **Rotor vibration corrupted the gyro bias.** It made slow flight look stationary, and in-flight bias updates soaked up real rotation. The bias is calibrated only at rest, giving 0.46 °/s on clean V1_01, the same as an oracle that knows the true bias.
2. **One spike in the first second could disable calibration** for the whole flight. Start-up now uses median-based outlier rejection.
3. **Not every recording starts at rest.** MH_04 starts moving, so its 0.078 rad/s gyro bias was never calibrated. Since the task is offline, a pre-pass now finds the quietest 1 s window anywhere in the recording. It accepts the window as rest only if:
   - the gyro is white noise around its mean (the means of four sub-windows agree within 3× the noise expectation), and
   - the gravity direction is steady (under 0.25°).

   It recovers the true bias on every EuRoC run, to about 0.002 rad/s, even under `direct`-mode gyro noise. It correctly finds no rest window in any of the 14 TUM-VI runs: a hand-held device is never still, and TUM-VI's bias is already compensated.
4. **Rotor-drag velocity aiding.** On a multirotor, sideways specific force is rotor drag, which is proportional to body velocity: a drift-free velocity signal from the IMU alone. It's worth 21% of the objective on V1_01. On the handheld TUM-VI the original detector (specific-force direction spread) was fooled: a phone carried roughly level also keeps its specific force near vertical. Drag is now also gated on **rotor vibration** (sample-to-sample accelerometer energy): at least 1.4 m/s² for multirotors in flight, under 0.4 for handheld even at maximum official degradation. The gain (−2.45 s) was fitted on V1_01 ground truth. A larger gain looked better on V1_01 but worse on V1_02, so it was rejected.
5. **Offline zero-phase smoothing.** Forward and backward leaky integrations are averaged, streamed through a temp file so memory stays O(1). The post-failure tail keeps the forward dead reckoning, since there's nothing after it to smooth with.
6. **The official degradation favours trusting the IMU more:** no output shrinkage, and faster velocity fusion than my own degradation model suggested.

## Verification

Contract tests cover: one pose per row, no appended poses, all-zero and NaN blind rows, accel-first and headerless input, s/ms timestamps, accelerometer in g, duplicated, stuck and garbage rows, dropouts, zip and CRLF.

```bash
python tools/test_contract.py
```

Official-degradation evaluation. EuRoC CSVs are fetched by `tools/fetch_euroc.py` (about 4 MB for V1_01; a nested sequence such as V1_02 or MH_04 needs its whole 0.6–0.7 GB zip), and TUM-VI CSVs by `tools/fetch_tumvi.py` (range reads into the uncompressed tar). `degrade_imu.py` is the organizers' script from the Track 3 student package.

```bash
python tools/make_official_runs.py euroc/V1_02_medium runs_official --script degrade_imu.py --modes euroc accelfirst direct --levels 1.0 0.6 --gammas 1.7 1.0
```

```bash
python tools/eval_runs.py runs_official/<run> --pure
```

Heap and runtime:

```bash
python src/main.py <input> -o est.txt --metadata m.json --profile-memory
```

CI (`.github/workflows/ci.yml`) runs the contract tests on Ubuntu and Windows. On Ubuntu it also checks the heap and runtime budgets on a simulated run, and that the Docker image gives the same trajectory.

## Known limitations

- Validated on four real sequences (three EuRoC flights, one handheld TUM-VI). Fast TUM-VI sequences (corridor, outdoors) and the other Machine Hall flights are untested. The drag gain is specific to the EuRoC vehicle; the vibration and spread detectors switch drag off for non-multirotor data.
- IMU-only position is 17–21% better than stand-still on these sequences, but still metre-level in absolute terms (about 5.6 m on the large Machine Hall).
- If the evaluation data's gyro is in deg/s without a header saying so, pass `--gyro-deg`. That can't be detected reliably from the data alone.
- The smoother needs about 110 bytes of temp disk per IMU sample (20 MB for 30 minutes). If no temp directory is writable, it falls back to a causal single pass with a warning.
