# Demo video script (max 3 minutes)

Record the terminal full-screen at a large font. Have `euroc_data/runs`, `euroc_data/holdout`
and a built `src/pathfinder` ready (see README, "Results on real EuRoC data").

| Time | Show | Say |
|---|---|---|
| 0:00 | Title slide | "Track 3, Path Finder: reconstruct a 6-DoF path from a degrading IMU, then dead-reckon 5 seconds after it fails." |
| 0:15 | Slide: the problem | "Double-integrating a noisy IMU explodes. On our real-flight test runs, plain integration is hundreds of metres off." |
| 0:35 | Terminal: `head -3` of a degraded `imu0/data.csv`, then `grep -c nan` | "This is the input: real EuRoC IMU with 10x noise, bias drift, spikes, dropped frames, a 2-second blackout and NaN rows at the end." |
| 0:55 | Terminal: `src/pathfinder <holdout run> -o est.txt --metadata meta.json` | "One command. The C++ build streams the file and writes one pose per row. 4 MB of RAM, under half a millisecond per second of data." |
| 1:15 | `head -3 est.txt`, `cat meta.json` | "TUM format, exact nanosecond timestamps. The metadata reports runtime, peak memory and dropout handling." |
| 1:30 | Slide: architecture | "A robust streaming filter, then an offline two-pass smoother. The forward pass writes records to a temp file, and two more passes read it forwards and backwards. Zero phase, still constant memory." |
| 2:00 | Terminal: `python tools/eval_runs.py euroc_data/holdout --pure` | "Scored on a flight that was never used for tuning. Stand-still baseline 1.82 m, ours 1.65 m: 9% better on every run. Raw integration: hundreds of metres." |
| 2:25 | Slide: held-out error chart | "Honest limit: with an IMU alone, position stays metre-level. The gains come from never diverging, attitude at the gyro's own limit, and handling failures cleanly." |
| 2:45 | Terminal: `python tools/test_contract.py --cpp` then `python tools/parity.py ...` | "14 contract tests, and C++ matches the Python reference byte for byte." |
| 2:55 | Thank-you slide | "Code, Docker and CI are in the repo. Thanks." |

Tips: run each command once beforehand so the first take has no build or cache delay, and keep
the terminal font at 18pt or larger so the numbers are legible in the recording.
