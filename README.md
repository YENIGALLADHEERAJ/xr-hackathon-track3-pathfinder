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
| ≤ 8 MB peak RAM, **measured at heap level** | **2.0 MB** heap peak (tracemalloc), constant in sequence length |
| ≤ 20 ms compute per 1 s of IMU, one thread | **2.4 ms** per IMU-second, single thread |
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

Real EuRoC flights were degraded with the organizers' `degrade_imu.py` using `tools/make_official_runs.py`. Each flight gets 13 runs:
- **Severity:** `--max-level` 1.0 and 0.6, `--gamma` 1.7 and 1.0, plus one run with an extra 2 s dropout.
- **Input modes:**
  - `euroc`: the intended degradation, in slide-15 column order.
  - `accelfirst`: left in the script's own column order.
  - `direct`: the script applied straight to EuRoC files, so the gyro gets accelerometer-level noise and spikes.

Ground truth is the EuRoC state estimate on every IMU stamp. Parameters were selected on **V1_01** only; **V1_02 is held out** (a faster flight, mean 0.91 m/s).

| Flight | Mode | Stand-still ATE | **Path Finder ATE** | Checkpoint err | Blind 5 s drift | Dropout-window err | Rotation RPE |
|---|---|---|---|---|---|---|---|
| **V1_02 (held out)** | euroc | 1.816 | **1.473 (−18.9%)** | 1.148 | 0.575 | 1.588 | 0.75 °/s |
| | accelfirst | 1.816 | **1.458 (−19.7%)** | 1.155 | 0.602 | 1.322 | 0.69 °/s |
| | direct | 1.816 | **1.440 (−20.7%)** | 1.128 | 0.614 | 1.277 | 0.69 °/s |
| V1_01 (selection) | euroc | 1.857 | **1.522 (−18.0%)** | 1.406 | 0.308 | 1.248 | 0.64 °/s |
| | accelfirst | 1.857 | **1.519 (−18.2%)** | 1.405 | 0.310 | 1.313 | 0.63 °/s |
| | direct | 1.857 | **1.463 (−21.2%)** | 1.397 | 0.458 | 1.717 | 2.12 °/s |

Values are means over the runs in each group, in metres except rotation. ATE is the RMSE after SE(3) alignment, the same as `evo_ape -a`. The stand-still baseline is a trajectory that never moves: the honest bar for IMU-only estimation, since raw double integration of the same data is hundreds of metres off.

### How it got there

| Step | V1_02 held-out ATE vs stand-still |
|---|---|
| First version, tuned on synthetic data | −1.5% |
| + gyro bias calibrated only at rest, robust start-up | −1.5% (rotation 2.6 → 0.46 °/s) |
| + less shrinkage, rotor-drag velocity aiding | −3.4% |
| + offline two-pass zero-phase smoother | −9.1% |
| + **official package**: format detection, no appended rows, retuned on `degrade_imu.py` data | **−18.9%** |

All numbers in the table are on the same held-out flight. The first four rows were measured with my own degradation model, the last with the organizers'.

Key findings along the way:
1. **Rotor vibration corrupted the gyro bias.** It made slow flight look stationary, and in-flight bias updates soaked up real rotation. The bias is now calibrated once, at rest. That gives 0.46 °/s on clean V1_01, the same as an oracle that knows the true bias.
2. **One spike in the first second could disable calibration** for the whole flight. Start-up now uses median-based outlier rejection.
3. **Rotor-drag velocity aiding.** On a multirotor, sideways specific force is rotor drag, which is proportional to body velocity: a drift-free velocity signal from the IMU alone. A multirotor detector gates it. With the final fusion settings, turning drag off makes the V1_01 objective 21% worse. Its gain (−2.45 s) was fitted once on V1_01 ground truth. A larger gain looked better on V1_01 but worse on held-out V1_02, so it was rejected as overfitting.
4. **Offline zero-phase smoothing.** Forward and backward leaky integrations are averaged, streamed through a temp file so memory stays O(1). The post-failure tail keeps the forward dead reckoning, since there's nothing after it to smooth with.
5. **The official degradation favours trusting the IMU more.** That means no output shrinkage and faster velocity fusion than my own degradation model suggested.

Where V1_01 and V1_02 disagreed (the blind-segment velocity decay: the slow flight prefers 0.25 s, the fast one 1 s), I took the middle of the region where both are within 0.4% (0.5 s), rather than the holdout's optimum.

## Verification

Contract tests cover: one pose per row, no appended poses, all-zero and NaN blind rows, accel-first and headerless input, s/ms timestamps, accelerometer in g, duplicated, stuck and garbage rows, dropouts, zip and CRLF.

```bash
python tools/test_contract.py
```

Official-degradation evaluation. EuRoC CSVs are fetched by `tools/fetch_euroc.py` (about 4 MB for V1_01). `degrade_imu.py` is the organizers' script from the Track 3 student package.

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

- Validated on two Vicon-room flights from one vehicle. Machine Hall and handheld TUM-VI sequences are untested. The drag gain is specific to the EuRoC vehicle, and the multirotor detector switches drag off for non-multirotor data.
- IMU-only position is about 18–21% better than stand-still on these flights, but still metre-level in absolute terms.
- If the evaluation data's gyro is in deg/s without a header saying so, pass `--gyro-deg`. That can't be detected reliably from the data alone.
- The smoother needs about 110 bytes of temp disk per IMU sample (20 MB for 30 minutes). If no temp directory is writable, it falls back to a causal single pass with a warning.
