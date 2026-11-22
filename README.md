# Track 03: Path Finder (submission bundle)

**XR Hackathon 2026, Track 03.** `src/main.py` turns a degraded IMU stream into a 6-DoF trajectory, including dead reckoning through the 5 seconds after the IMU fails. It is a **standalone Python script** (standard library only, single thread, offline), as the Track 3 FAQ requires.

```
team_name.zip
├── estimated_trajectory.txt   # generated on the organizer's evaluation dataset
├── metadata.json              # run statistics: runtime, format, dropouts, spikes, rotor-drag model
└── src/
    ├── main.py                # entry point (Python 3.8+, stdlib only)
    └── README.md              # method, output contract, budgets
```

The repository also contains `tools/` (validation only; numpy), a `Dockerfile`, CI, `DEMO_SCRIPT.md`, and `docs/`.

## Run

```bash
python src/main.py path/to/evaluation_dataset.zip -o estimated_trajectory.txt --metadata metadata.json
```

The input can be the `.zip`, the unzipped folder, or `imu0/data.csv` itself. No `pip install` is needed.

The evaluation harness's exact command isn't published, so every plausible call form works:

| Call | Behaviour |
|------|-----------|
| `main.py IN -o OUT` | the documented form |
| `main.py IN OUT` | positional output |
| `main.py --input IN --output OUT` | flag form |
| `main.py IN` | writes `estimated_trajectory.txt` in the current folder |
| `main.py` (no arguments) | finds `imu0/data.csv` below the current folder |
| `import main; main.estimate(IN, OUT)` | importable API |

Unknown flags are ignored with a warning, a missing output folder is created, and an output path that is a folder gets `estimated_trajectory.txt` inside it. If the offline smoother ever fails, the script falls back to its causal single pass, so a complete trajectory is always written.

### Docker

Build the image from the repository root:

```bash
docker build -t pathfinder .
```

Put `evaluation_dataset.zip` in a folder named `data`, then run from the folder that contains `data`:

```bash
docker run --rm -v "$PWD/data:/data" pathfinder evaluation_dataset.zip -o estimated_trajectory.txt --metadata metadata.json
```

All call forms in the table above work inside the container. `docker run --rm pathfinder --help` lists the options.

**Building the submission bundle** in one step:

```bash
python tools/make_bundle.py path/to/evaluation_dataset.zip CollegeName_TeamName
```

It runs `main.py`, verifies the output against the input (one pose per IMU row, timestamps in input order, 8 finite columns, unit quaternions), and writes `CollegeName_TeamName.zip` in the layout above. `metadata.json` records file names only, never local paths.

## Compliance with the official Track 3 package

| Rule (slides 14–17 and the FAQ) | How `main.py` meets it |
|---------------------------------|------------------------|
| Standalone Python script; `main.cpp` not accepted | Python 3 stdlib only, one file |
| **Exactly one pose per IMU frame; do not append poses** | One output row per input row, in input order. Blind window dead-reckoned in place. Appending is opt-in via `--extrapolate`, off by default. |
| Timestamps ns → s | Exact integer arithmetic, 9 decimals |
| Dropouts are missing rows | Timestamp gaps bridged by dead reckoning; every remaining row gets a pose |
| Estimate from the IMU file only | No other input is read |
| Offline; future samples may be used | Two-pass zero-phase smoother |
| ≤ 8 MB peak RAM, **measured at heap level** | **0.44–0.51 MB** heap peak while estimating (tracemalloc), constant in sequence length. |
| ≤ 20 ms compute per 1 s of IMU, one thread | **~1.2–1.5 ms** per IMU-second, single thread |
| SE(3)-aligned ATE and RPE (evo) | Output frame: z-up, gravity aligned, starting at the origin |

## Rotor-Drag Velocity Model (reported in `metadata.json`)

```json
"rotor_drag_model": {
    "equation": "v_drag = drag_gain × f_lateral  (m/s per m/s²)",
    "description": "Rotor-drag velocity aiding for multirotors. Thrust acts along a body-fixed axis; the accelerometer component perpendicular to thrust axis is rotor drag ≈ −k·v_body, giving a drift-free velocity measurement from IMU alone.",
    "detector": "Two-gated: (1) specific-force spread vs thrust axis; (2) rotor vibration accelerometer energy. Both gates must open for drag aiding to engage.",
    "thrust_axis_tracking": "tau_n=20.0 s slow mean of specific-force direction",
    "lateral_force_tracking": "tau_c=30.0 s offset tracking, tau_fd=0.6 s low-pass",
    "drag_warm_s": 15.0,
    "drag_enabled": true,
    "drag_gain": -2.45
}
```

On a multirotor, sideways specific force is rotor drag, which is proportional to body velocity — a drift-free velocity signal from the IMU alone. It's worth about 21% of the objective on V1_01. On handheld TUM-VI the spread-only detector was fooled, so drag is now also gated on rotor vibration (≥1.4 m/s² for multirotors, <0.4 for handheld). Gate closed = exactly the plain filter.

## Input Formats Handled

The organizers' `degrade_imu.py` documents its columns as `timestamp, accel(3), gyro(3)`, while the slide-15 input spec (EuRoC) is `timestamp, gyro(3), accel(3)`. The script detects all of this from the data, reported under `input_format` in `metadata.json`:

| Property | Detection |
|----------|-----------|
| Column order | The accelerometer triplet is the one whose median norm is about 1 g. Header cross-checked. |
| Timestamp unit | From median sample spacing (same thresholds as `degrade_imu.py`), exact ns conversion |
| Accelerometer in g | Median norm 0.5–2 means g, converted to m/s² |
| Gyro in deg/s | From header (`deg`), or `--gyro-deg` |

The script's other degradations are handled by the front-end:
- **Blind window:** all-zero rows count as invalid.
- **Stuck samples:** three or more identical rows count as frozen.
- **Duplicated rows:** a repeated timestamp gets its own, identical pose.
- **Spikes:** gated against the running noise level.

## Results with the Official Degradation Script

Four real sequences covering both datasets the jury uses: two Vicon-room flights, a Machine Hall flight in a much larger space, and a handheld TUM-VI recording with a different IMU. Each was degraded with the organizers' `degrade_imu.py` using `tools/make_official_runs.py`, giving 13 runs per sequence:

| Sequence | Platform | Speed | Stand-still spread | Role |
|----------|----------|-------|-------------------|------|
| EuRoC V1_01_easy | multirotor, Vicon room | 0.41 m/s | 1.86 m | parameter selection |
| EuRoC V1_02_medium | multirotor, Vicon room | 0.91 m/s | 1.82 m | held out for parameter fitting |
| EuRoC MH_04_difficult | multirotor, large hall | 0.97 m/s | 6.94 m | exposed the bias bug |
| TUM-VI room1 | **handheld**, different IMU | 1.07 m/s | 1.01 m | exposed the drag-gating bug |

| Sequence | Mode | Stand-still ATE | **Path Finder ATE** | Checkpoint err | Blind 5 s drift | Dropout-window err | Rotation RPE |
|----------|------|----------------|---------------------|----------------|-----------------|------------------|--------------|
| V1_01 | euroc | 1.857 | **1.525 (−17.9%)** | 1.384 | 0.311 | 1.115 | 0.51 °/s |
| | accelfirst | 1.857 | **1.522 (−18.0%)** | 1.386 | 0.309 | 1.071 | 0.51 °/s |
| | direct | 1.857 | **1.463 (−21.3%)** | 1.380 | 0.398 | 1.820 | 2.07 °/s |
| V1_02 | euroc | 1.816 | **1.484 (−18.3%)** | 1.134 | 0.671 | 1.704 | 0.28 °/s |
| | accelfirst | 1.816 | **1.465 (−19.3%)** | 1.144 | 0.680 | 1.404 | 0.28 °/s |
| | direct | 1.816 | **1.443 (−20.5%)** | 1.123 | 0.695 | 1.342 | 0.25 °/s |
| MH_04 | euroc | 6.942 | **5.445 (−21.6%)** | 4.845 | 0.448 | 6.349 | 0.25 °/s |
| | accelfirst | 6.942 | **5.401 (−22.2%)** | 4.751 | 0.442 | 6.143 | 0.26 °/s |
| | direct | 6.942 | **5.517 (−20.5%)** | 4.822 | 0.919 | 6.071 | 1.82 °/s |
| TUM-VI room1 | euroc | 1.013 | **0.808 (−20.3%)** | 0.743 | 0.839 | 0.755 | 0.81 °/s |
| | accelfirst | 1.013 | **0.805 (−20.6%)** | 0.743 | 0.846 | 0.756 | 0.82 °/s |
| | direct | 1.013 | **0.812 (−19.9%)** | 0.759 | 0.875 | 0.795 | 0.84 °/s |

**Every group beats the stand-still baseline by 17.9–22.2%** (all 52 runs; the worst single run is −14.3%). Raw double integration is 132–1,511 m off.

## Method

See `src/README.md` for the full method documentation. Summary:

1. **Robust front-end:** spike rejection, dropout handling, frozen sensor detection, gap bridging
2. **Gravity alignment & bias:** rest-window search anywhere in the file (offline pre-pass)
3. **Attitude:** quaternion integration with gravity correction (static only)
4. **Translation:** band-pass double integration with leaky velocity + position prior
5. **Rotor-drag aiding:** drift-free velocity for multirotors, two-gated detector
6. **Gap filling → dead reckoning:** interpolation ≤0.5 s, exponential decay beyond
7. **Offline two-pass zero-phase smoother:** forward + backward passes via temp file, O(1) memory

## Verification

**35 contract tests** cover: one pose per row, no appended poses, all-zero and NaN blind rows, accel-first and headerless input, s/ms timestamps, accelerometer in g, duplicated, stuck and garbage rows, dropouts, zip (including Windows-style paths), CRLF and a UTF-8 BOM, every call form, and the smoother-failure fallback:

```bash
python tools/test_contract.py
```

Official-degradation evaluation:

```bash
python tools/make_official_runs.py euroc/V1_02_medium runs_official --script degrade_imu.py --modes euroc accelfirst direct --levels 1.0 0.6 --gammas 1.7 1.0
python tools/eval_runs.py runs_official/<run> --pure
```

Heap and runtime:

```bash
python src/main.py <input> -o est.txt --metadata m.json --profile-memory
```

CI (`.github/workflows/ci.yml`) runs the contract tests on Ubuntu and Windows, checks heap and runtime budgets on a simulated run, and verifies Docker reproducibility.

## Demo

See `DEMO_SCRIPT.md` for a complete 2:30-minute demo script with timestamps, terminal commands, and slide content.

## Known Limitations

- Validated on four real sequences (three EuRoC flights, one handheld TUM-VI). Fast TUM-VI sequences (corridor, outdoors) and other Machine Hall flights are untested.
- IMU-only position is 17–21% better than stand-still, but still metre-level in absolute terms (about 5.6 m on the large Machine Hall).
- If the evaluation data's gyro is in deg/s without a header saying so, pass `--gyro-deg`.
- The smoother needs about 110 bytes of temp disk per IMU sample (20 MB for 30 minutes). Falls back to causal single pass if no temp directory is writable.
