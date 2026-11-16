# Demo video script (2–3 minutes, strict maximum 3)

Record the terminal full-screen with a large font. Beforehand, prepare an officially degraded run:

```bash
python tools/make_official_runs.py euroc/V1_02_medium runs_official --script degrade_imu.py --modes euroc
```

Then run each command once so the first take has no cold-cache delay.

| Time | Show | Say |
|---|---|---|
| 0:00 | Title slide | "Track 3, Path Finder: reconstruct a 6-DoF path from a degrading IMU, and dead-reckon through the 5 seconds after it fails." |
| 0:15 | Slide: the problem | "Double-integrating a noisy IMU explodes: on our test flights, plain integration is hundreds of metres off." |
| 0:30 | Terminal: `head -3` and `tail -2` of the degraded `imu0/data.csv` | "This is a real EuRoC flight degraded with the organizers' script: noise and bias ramping up, spikes, stuck and dropped samples, and all-zero rows once the IMU dies." |
| 0:50 | Terminal: `python src/main.py <run> -o est.txt --metadata meta.json` | "One standalone Python script, standard library only. It detects the column order and units itself, and writes exactly one pose per input row." |
| 1:10 | `wc -l` of input and output, `tail -2 est.txt`, `input_format` and runtime in `meta.json` | "Same number of rows in and out. The blind window is dead-reckoned in place, nothing appended. About 2 MB of heap and 2.5 ms per second of data, against 8 MB and 20 ms." |
| 1:35 | Slide: architecture | "A robust streaming filter with rotor-drag velocity aiding, then an offline two-pass smoother: forward and backward passes through a temp file, zero phase, constant memory." |
| 2:00 | Terminal: `python tools/eval_runs.py <runs> --pure` | "Scored like evo with SE(3) alignment, on a flight never used for tuning. A trajectory that never moves scores 1.82 m; ours is 1.47 m, 19% better. Raw integration is hundreds of metres off." |
| 2:25 | Slide: held-out error chart | "Honest limit: from an IMU alone, position stays metre-level. The gains come from drag-based velocity, smoothing, attitude at the gyro's own limit, and robust handling of failures." |
| 2:45 | Terminal: `python tools/test_contract.py` | "24 contract tests: row count, input formats, failure rows, dropouts." |
| 2:55 | Thank-you slide | "Code, Docker and CI are in the repo. Thanks." |

Keep the terminal font at 18pt or larger so the numbers are legible in the recording.
