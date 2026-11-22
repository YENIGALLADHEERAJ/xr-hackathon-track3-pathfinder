# Demo Video Script — Track 3: Path Finder

**Target length:** 2 minutes 30 seconds (strict maximum: 3 minutes)  
**Format:** Terminal full-screen recording, large font (18pt+), with slide interludes  
**Repository:** `https://github.com/YENIGALLADHEERAJ/xr-hackathon-track3-pathfinder`

---

## Before You Record

1. **Prepare an officially degraded run** (do this once before recording):

   ```bash
   cd track3_pathfinder
   python tools/make_official_runs.py euroc/V1_02_medium runs_official --script euroc_data/official/degrade_imu.py --modes euroc
   ```

   This creates `runs_official/V1_02_euroc_L1.0_g1.7/` with `mav0/imu0/data.csv` (degraded) and `groundtruth.txt`.

2. **Warm up the cache** so your first take has no cold-cache delay:

   ```bash
   python src/main.py runs_official/V1_02_euroc_L1.0_g1.7 -o warmup_est.txt --metadata warmup_meta.json
   python tools/eval_runs.py runs_official/V1_02_euroc_L1.0_g1.7 --pure
   python tools/test_contract.py
   ```

   Then delete `warmup_est.txt` and `warmup_meta.json` so the recording starts clean.

3. **Terminal setup:** Record full-screen terminal with **18pt font or larger** (per Track 3 demo guidelines). Use a dark theme with high contrast. Clear the terminal before starting.

4. **Slides:** Prepare 4 simple slides (title, problem, architecture, results). You can show them as static images between terminal segments.

---

## Script

| Time | Show | Say |
|------|------|-----|
| **0:00** | **Title slide** | "Track 3, Path Finder: reconstruct a 6-DoF path from a degrading IMU, and dead-reckon through the 5 seconds after it fails." |
| **0:15** | **Slide: the problem** | "Double-integrating a noisy IMU explodes. On our test flights, plain integration is hundreds of metres off. We need robust filtering plus smoothing, not naive integration." |
| **0:30** | **Terminal:** `head -3 runs_official/V1_02_euroc_L1.0_g1.7/mav0/imu0/data.csv` then `tail -2` | "This is a real EuRoC flight degraded with the organizers' script: noise and bias ramping up, spikes, stuck and dropped samples, and all-zero rows once the IMU dies." |
| **0:50** | **Terminal:** `python src/main.py runs_official/V1_02_euroc_L1.0_g1.7 -o est.txt --metadata meta.json` | "One standalone Python script, standard library only. No numpy, no pip install. It detects the column order and units itself, and writes exactly one pose per input row." |
| **1:10** | **Terminal:** `wc -l` on input and output, then `tail -2 est.txt`, then `cat meta.json` (show `runtime_ms_per_imu_second`, `input_format`, `blind_time_s`) | "Same number of rows in and out. The blind window is dead-reckoned in place, nothing appended. About half a megabyte of heap and under 3 ms per second of data, against 8 MB and 20 ms budgets." |
| **1:35** | **Slide: architecture** (4 bullet points: robust front-end, quaternion attitude, band-pass double integration, offline two-pass smoother) | "A robust streaming filter with rotor-drag velocity aiding, then an offline two-pass smoother: forward and backward passes through a temp file, zero phase, constant memory." |
| **2:00** | **Terminal:** `python tools/eval_runs.py runs_official/V1_02_euroc_L1.0_g1.7 --pure` | "Scored like evo with SE(3) alignment, on a flight never used for tuning. A trajectory that never moves scores 1.82 m; ours is 1.48 m, 18% better. Raw integration is over 300 metres off." |
| **2:25** | **Slide: results** (ATE comparison table: stand-still vs Path Finder across V1_01, V1_02, MH_04, TUM-VI) | "Every sequence beats stand-still by 17 to 22 percent. The gains come from drag-based velocity, smoothing, attitude at the gyro's own limit, and robust handling of failures." |
| **2:45** | **Terminal:** `python tools/test_contract.py` | "35 contract tests: row count, input formats, failure rows, dropouts, every call form. All pass." |
| **2:55** | **Thank-you slide** | "Code, Docker and CI are in the repo. Thanks." |

---

## Commands to Have Ready (copy-paste)

```bash
# Show degraded input structure
head -3 runs_official/V1_02_euroc_L1.0_g1.7/mav0/imu0/data.csv
tail -2 runs_official/V1_02_euroc_L1.0_g1.7/mav0/imu0/data.csv

# Run the estimator
python src/main.py runs_official/V1_02_euroc_L1.0_g1.7 -o est.txt --metadata meta.json

# Verify row count matches input
wc -l runs_official/V1_02_euroc_L1.0_g1.7/mav0/imu0/data.csv est.txt

# Show output tail (blind segment dead-reckoned)
tail -5 est.txt

# Show key metadata stats
python -c "import json; m=json.load(open('meta.json')); print('runtime ms/IMU-s:', m['runtime_ms_per_imu_second']); print('heap:', m.get('python_peak_heap_bytes')); print('invalid rows:', m['invalid_rows']); print('blind time s:', m['blind_time_s']); print('format:', m['input_format'])"

# Evaluate against ground truth
python tools/eval_runs.py runs_official/V1_02_euroc_L1.0_g1.7 --pure

# Contract tests
python tools/test_contract.py
```

---

## Slide Content

### Slide 1 — Title
```
Track 3: Path Finder
XR Hackathon 2026

6-DoF trajectory estimation from a degrading IMU
Standalone Python — stdlib only — offline
```

### Slide 2 — The Problem
```
Plain double integration of a noisy IMU:
- Bias drift → position error growing with t²
- Gyro noise → attitude drift → gravity misalignment
- On our test flights: raw integration is 300+ metres off

We need robust filtering + smoothing, not naive integration.
```

### Slide 3 — Architecture
```
1. Robust front-end: spike rejection, dropout handling, frozen sensor detection
2. Attitude: quaternion integration of bias-corrected gyro
3. Translation: band-pass double integration with leaky velocity + position prior
4. Rotor-drag velocity aiding (multirotors): drift-free velocity from IMU alone
5. Offline two-pass zero-phase smoother (forward + backward through temp file)
6. Blind dead-reckoning: 5s post-IMU-failure horizon with exponential decay

Budgets: ≤8 MB heap, ≤20 ms per IMU-second, single thread
```

### Slide 4 — Results
```
ATE Results (metres, lower is better)

Sequence     Stand-still   Path Finder   Improvement
V1_01        1.857         1.525         −17.9%
V1_02        1.816         1.484         −18.3%
MH_04        6.942         5.445         −21.6%
TUM-VI       1.013         0.808         −20.3%

Raw integration: 300+ metres off
```

### Slide 5 — Thank You
```
Code, Docker and CI are in the repo.
https://github.com/YENIGALLADHEERAJ/xr-hackathon-track3-pathfinder

Thanks.
```

---

## Tips

- **Speak clearly and slowly.** The video is 2:30 max; don't rush.
- **Show commands before running them** so viewers can see what you're typing.
- **Highlight the key numbers** in meta.json (runtime, heap, blind time) so they're legible.
- **If a command fails**, stop, fix it, and re-record that segment. Don't leave errors in the final video.