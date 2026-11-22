# Path Finder: Trajectory Estimation Under Sensor Degradation

**XR Hackathon 2026, Track 03.** Takes a degraded IMU stream (`imu0/data.csv`, EuRoC format or anything the organizers' `degrade_imu.py` produces) and writes a 6-DoF trajectory (`estimated_trajectory.txt`, TUM format) with **exactly one pose per input row**, including dead-reckoned poses for the post-failure blind window.

Standalone Python 3.8+ standard library only — no `pip install`, no numpy, single file.

```
python main.py evaluation_dataset.zip -o estimated_trajectory.txt --metadata metadata.json
```

## Budgets

| Budget | Limit | Measured |
|--------|-------|----------|
| Peak RAM (heap: data structures, buffers, estimation) | ≤ 8 MB | **0.44–0.51 MB**, constant in sequence length (streaming; I/O in 256–512-record batches). Python compiling the script itself transiently needs ~4.6 MB more, before estimation starts. |
| Compute per 1 s of IMU | ≤ 20 ms | **~1.2–1.5 ms** (aggressive optimization: `__slots__`, separate quaternion float attributes, inlined `q_to_rot`/`q_integrate`/`_gate`, module-level math aliases, precomputed inverse time constants, inline `min()`) |
| Total runtime (83.5 s V1_02 sequence) | < 1.5 s | **~0.13–0.23 s** |
| Threads | 1 | 1 |

Measured on Windows 11 with Python 3.11 on real EuRoC flights degraded with `degrade_imu.py`, two-pass smoother on (default). Track 3 FAQ measures the 8 MB at heap level, not process RSS.

## Run

The input can be a zipped evaluation folder, a dataset folder (it searches for `imu0/data.csv`), or the CSV itself:

| Call | Behaviour |
|------|-----------|
| `main.py IN -o OUT` | Documented form |
| `main.py IN OUT` | Positional output |
| `main.py --input IN --output OUT` | Flag form |
| `main.py IN` | Writes `estimated_trajectory.txt` in cwd |
| `main.py` (no args) | Finds `imu0/data.csv` below cwd |
| `main.estimate(IN, OUT)` | Importable Python API |

Unknown flags are ignored with a warning; missing output folders are created; an output path that is a folder gets `estimated_trajectory.txt` inside it.

| Option | Meaning |
|--------|---------|
| `-o FILE` | Output trajectory (default `estimated_trajectory.txt`) |
| `--metadata FILE` | Write run statistics as JSON (runtime, format detection, dropouts, spikes, rotor-drag model, ...) |
| `--gyro-deg` | Gyro columns are in deg/s (only if header doesn't say so) |
| `--no-smooth` | Single-pass causal output (disables offline two-pass smoother) |
| `--extrapolate` | Append 5 s after last row. **Off by default**: FAQ forbids appended poses |
| `--set KEY=VAL ...` | Override a tuning parameter (see `DEFAULTS` in `main.py`) |
| `--profile-memory` | Report Python heap peak via tracemalloc |

## Output Contract

- Space-separated `timestamp tx ty tz qx qy qz qw`, one `#` header line.
- **Exactly one pose per input IMU row, in input order. Nothing is appended.**
- Timestamp is the row's time converted exactly to seconds (integer arithmetic, 9 decimals).
- All-zero, NaN, empty, out-of-range, or frozen rows still get a pose via dead reckoning.
- Duplicated timestamps get the same pose again.
- **Frame:** z-up, gravity-aligned, origin = start position, yaw = 0 at start. SE(3) alignment (`evo -a`) maps to ground truth.

## Input Format Detection (auto, reported in `metadata.json`)

| Property | Detection |
|----------|-----------|
| Column order | The accelerometer triplet = median norm ~1 g. EuRoC is gyro-first; `degrade_imu.py` documents accel-first. Header cross-checked, not blindly trusted. |
| Timestamp unit | s / ms / µs / ns from median spacing (same thresholds as `degrade_imu.py`), exact ns conversion |
| Accelerometer unit | Median norm 0.5–2 ⇒ g, converted to m/s² |
| Gyro unit | `deg` in gyro header names, or `--gyro-deg` |

Standard EuRoC input passes through bit-for-bit.

## Method

The design is a streaming complementary filter with O(1) memory per row:

1. **Robust front-end.** NaN, empty, all-zero, saturated (|ω| > 40 rad/s, |a| > 200 m/s²), and frozen (≥3 identical rows) are treated as dropouts. Impulsive spikes are gated against a running difference-noise threshold. Timestamp gaps over 2.5× nominal are bridged by blind dead reckoning. Duplicate/out-of-order stamps keep the previous pose.

2. **Gravity alignment & bias.** First second sets roll/pitch from the shortest rotation taking mean accelerometer to +z. If static, also sets gyro bias and local |g|. Median-based outlier rejection protects against a single spike voiding the static test.

   **Rest-window search:** If the file doesn't start at rest (e.g. MH_04), an offline pre-pass finds the quietest 1 s window anywhere. It counts as rest only if gyro sub-window means agree within 3× noise expectation, gravity direction moves < 0.25°, and |f| is close to g and steady.

3. **Attitude.** Exact quaternion exponential integration of bias-corrected gyro. Mahony-style gravity correction weighted by `exp(-((|a|-g)/0.15g)²)`, active only while static (`kp=0` in motion — on a multirotor the accelerometer measures thrust, not gravity).

4. **Static segment & ZUPT.** Static detection uses gyro energy and |a| deviation from g. Velocity → 0, gyro bias ← mean, accel bias ← residual while static. Defaults to initial static segment only (`static_hold_s=1e9`); re-enable with `--set static_hold_s=0.3` for handheld data with real pauses.

5. **Translation as band-pass double integration.** Remove slow body-frame specific-force mean (absorbs bias drift + residual tilt), integrate to velocity with leak `τ_v`, integrate to position with weak prior `τ_p` toward origin. Noise-adaptive shrink factor `β` is off by default (`shrink_gamma=0`).

6. **Rotor-drag velocity aiding (multirotors).** Thrust acts along a body-fixed axis; the accelerometer component perpendicular to thrust is rotor drag ≈ −k·v_body — a drift-free velocity measurement from IMU alone.

   - Thrust axis = slow mean of specific-force direction (`τ_n=20 s`)
   - Lateral force = plane perpendicular to thrust, offset tracked (`τ_c=30 s`), low-passed (`τ_fd=0.6 s`)
   - Drag velocity = `drag_gain · f_lateral` (`drag_gain=−2.45 s`, fitted on V1_01 ground truth)
   - Velocity leak pulls toward drag velocity instead of zero
   - **Two-gated detector:** specific-force spread vs thrust axis (spread_lo=0.14 rad, spread_hi=0.26 rad) AND rotor vibration accelerometer energy (vib_lo=0.8, vib_hi=1.2 m/s²). Both must open.
   - Gate closed = exactly the plain filter (handheld TUM-VI is correctly excluded).

7. **Gap filling → dead reckoning.** Gaps ≤0.5 s filled offline by interpolation between valid samples. Longer gaps dead-reckoned with exponential velocity/rate decay (`τ_vb=0.25 s`, `τ_wb=0.1 s`). Same code path for single dropped frame, 2 s signal loss, and final 5 s blind segment.

8. **Offline two-pass zero-phase smoothing (default).** Forward pass streams per-step records to a temp file (~110 bytes/sample). Pass B1 integrates velocity forward; Pass B2 reads backward integrating backward; Pass C integrates forward and averages forward/backward position. Zero-phase, no lag. Post-failure blind tail keeps forward dead reckoning anchored to smoothed position at failure. Falls back to causal single pass if no writable temp directory.

## Rotor-Drag Model (reported in `metadata.json`)

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

## Validation

**35 contract tests** cover: one pose per row, no appended poses, all-zero/NaN blind rows, accel-first/headerless input, s/ms timestamps, accel in g, duplicated/stuck/garbage rows, dropouts, zip (incl. Windows paths), CRLF, UTF-8 BOM, every call form, smoother-failure fallback:

```bash
python tools/test_contract.py
```

Official-degradation evaluation:

```bash
python tools/make_official_runs.py euroc/V1_02_medium runs_official --script degrade_imu.py --modes euroc accelfirst direct --levels 1.0 0.6 --gammas 1.7 1.0
python tools/eval_runs.py runs_official/<run> --pure
```

Heap + runtime:

```bash
python src/main.py <input> -o est.txt --metadata m.json --profile-memory
```

## Build Submission Bundle

```bash
python tools/make_bundle.py <evaluation_dataset.zip> CollegeName_TeamName
```

Runs `main.py`, verifies the output contract against the input, and writes `CollegeName_TeamName.zip`:

```
CollegeName_TeamName.zip
├── estimated_trajectory.txt
├── metadata.json
└── src/
    ├── main.py
    └── README.md
```

## Track 3 FAQ Compliance

| Rule | How We Meet It |
|------|---------------|
| Standalone Python script (no `main.cpp`) | Python 3 stdlib only, one file |
| Exactly one pose per IMU frame; no appended poses | One output row per input row, in input order. Blind window dead-reckoned in place. Appending is opt-in via `--extrapolate`, off by default. |
| Timestamps ns → s | Exact integer arithmetic, 9 decimals |
| Dropouts are missing rows | Timestamp gaps bridged by dead reckoning; every remaining row gets a pose |
| Estimate from IMU file only | No other input read |
| Offline; future samples allowed | Two-pass zero-phase smoother |
| ≤ 8 MB peak RAM (heap level) | 0.44–0.51 MB heap peak |
| ≤ 20 ms compute per 1 s IMU, one thread | ~1.2–1.5 ms per IMU-second, single thread |
| SE(3)-aligned ATE/RPE output | z-up, gravity-aligned, origin = start |