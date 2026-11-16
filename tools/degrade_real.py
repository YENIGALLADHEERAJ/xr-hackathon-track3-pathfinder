#!/usr/bin/env python3
"""
Apply Track-3 style progressive degradation to a REAL EuRoC sequence.

    python degrade_real.py <euroc_seq_dir> <out_dir> [--seed 0] [--failure cut|nan]
                           [--noise-gain 10] [--bias-gain 1] [--clean]

<euroc_seq_dir> must contain mav0/imu0/data.csv and
mav0/state_groundtruth_estimate0/data.csv (as extracted by fetch_euroc.py).

Writes the same layout as simulate.py so evaluate.py / tune.py work unchanged:
    <out>/mav0/imu0/data.csv     degraded IMU (real IMU + extra noise/bias/drops/failure)
    <out>/groundtruth.txt        TUM ground truth interpolated to every IMU stamp
    <out>/meta.json              checkpoints, failure time, signal-loss window

The real sensor already carries its own noise and bias, so the added noise
ramps from 0 to (noise_gain - 1) x the EuRoC nominal level over the mission,
i.e. total noise goes from ~1x to ~noise_gain x.
"""
import argparse
import json
import os

import numpy as np


def load_csv(path):
    return np.loadtxt(path, delimiter=",", comments="#")


def slerp(q0, q1, a):
    d = np.sum(q0 * q1, 1)
    q1 = np.where(d[:, None] < 0, -q1, q1)
    d = np.abs(d)
    th = np.arccos(np.clip(d, -1, 1))
    s = np.sin(th)
    small = s < 1e-8
    w0 = np.where(small, 1 - a, np.sin((1 - a) * th) / np.where(small, 1, s))
    w1 = np.where(small, a, np.sin(a * th) / np.where(small, 1, s))
    q = w0[:, None] * q0 + w1[:, None] * q1
    return q / np.linalg.norm(q, axis=1, keepdims=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("seq")
    ap.add_argument("out")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--failure", choices=["cut", "nan"], default="cut")
    ap.add_argument("--blind", type=float, default=5.0)
    ap.add_argument("--noise-gain", type=float, default=10.0)
    ap.add_argument("--bias-gain", type=float, default=1.0)
    ap.add_argument("--clean", action="store_true", help="no degradation, no dropouts (reference run)")
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed)

    imu = load_csv(os.path.join(a.seq, "mav0", "imu0", "data.csv"))
    gt = load_csv(os.path.join(a.seq, "mav0", "state_groundtruth_estimate0", "data.csv"))
    ts = imu[:, 0].astype(np.int64)
    gts = gt[:, 0].astype(np.int64)
    # keep IMU rows inside the ground-truth span
    m = (ts >= gts[0]) & (ts <= gts[-1])
    imu, ts = imu[m], ts[m]
    w = imu[:, 1:4].copy()
    f = imu[:, 4:7].copy()
    n = len(ts)
    t = (ts - ts[0]) * 1e-9
    T = t[-1]
    u = t / T

    # ---- ground truth at every IMU stamp ----
    k = np.clip(np.searchsorted(gts, ts) - 1, 0, len(gts) - 2)
    al = ((ts - gts[k]) / (gts[k + 1] - gts[k])).astype(float)
    P = gt[k, 1:4] * (1 - al[:, None]) + gt[k + 1, 1:4] * al[:, None]
    Q = slerp(gt[k, 4:8], gt[k + 1, 4:8], al)          # (w, x, y, z)
    Q[Q[:, 0] < 0] *= -1
    gt_ok = (gts[k + 1] - gts[k]) <= 50_000_000          # skip ground truth inside mocap dropouts

    # ---- degradation ----
    keep = np.ones(n, bool)
    nanrow = np.zeros(n, bool)
    t_fail = T - a.blind
    s2 = None
    if not a.clean:
        dt = np.median(np.diff(t))
        sg0, sa0 = 1.7e-4 / np.sqrt(dt), 2.0e-3 / np.sqrt(dt)
        sg = sg0 * (a.noise_gain - 1) * u ** 2
        sa = sa0 * (a.noise_gain - 1) * u ** 2
        rwg = np.cumsum(rng.normal(0, 1, (n, 3)) * (1.9e-5 * (1 + 20 * u[:, None] ** 2)) * np.sqrt(dt), 0)
        rwa = np.cumsum(rng.normal(0, 1, (n, 3)) * (3.0e-3 * (1 + 20 * u[:, None] ** 2)) * np.sqrt(dt), 0)
        drift_g = np.outer(u ** 2, rng.normal(0, 0.01, 3))
        drift_a = np.outer(u ** 2, rng.normal(0, 0.1, 3))
        w += (rng.normal(0, 0.002, 3) + rwg + drift_g) * a.bias_gain + rng.normal(0, 1, (n, 3)) * sg[:, None]
        f += (rng.normal(0, 0.02, 3) + rwa + drift_a) * a.bias_gain + rng.normal(0, 1, (n, 3)) * sa[:, None]
        ns = int(0.0005 * n)
        idx = rng.integers(0, n, ns)
        w[idx] += rng.normal(0, 5, (ns, 3))
        f[idx] += rng.normal(0, 40, (ns, 3))
        drop = rng.random(n) < 0.01 * u
        for s in np.where(rng.random(n) < 0.0008 * u)[0]:
            drop[s:s + rng.integers(5, 40)] = True
        s2 = rng.uniform(0.6, 0.85) * t_fail
        drop |= (t >= s2) & (t < s2 + 2.0)
        drop[t < 3.0] = False
        keep &= ~drop
    blind = t > t_fail
    if a.failure == "cut":
        keep &= ~blind
    else:
        nanrow |= blind

    os.makedirs(os.path.join(a.out, "mav0", "imu0"), exist_ok=True)
    with open(os.path.join(a.out, "mav0", "imu0", "data.csv"), "w", newline="\n") as fh:
        fh.write("#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],"
                 "a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n")
        for i in np.where(keep)[0]:
            if nanrow[i]:
                fh.write("%d,nan,nan,nan,nan,nan,nan\n" % ts[i])
            else:
                fh.write("%d,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f\n" % (ts[i], *w[i], *f[i]))
    with open(os.path.join(a.out, "groundtruth.txt"), "w", newline="\n") as fh:
        fh.write("# timestamp tx ty tz qx qy qz qw\n")
        for i in range(n):
            if not gt_ok[i]:
                continue
            fh.write("%d.%09d %.6f %.6f %.6f %.9f %.9f %.9f %.9f\n" % (
                ts[i] // 10 ** 9, ts[i] % 10 ** 9, *P[i], Q[i, 1], Q[i, 2], Q[i, 3], Q[i, 0]))
    t0 = float(ts[0]) * 1e-9
    cps = [float(x) for x in np.arange(10.0, t_fail + 1e-9, 10.0)] + [float(t_fail)]
    meta = dict(source=os.path.abspath(a.seq), real=True, seed=a.seed, failure=a.failure,
                clean=a.clean, t0=t0, t_fail=t0 + t_fail, checkpoints=[t0 + c for c in cps],
                signal_loss=[t0 + s2, t0 + s2 + 2.0] if s2 is not None else [t0 + T + 1, t0 + T + 1],
                rows=int(keep.sum()), gt_rows=int(n), duration_s=float(T))
    with open(os.path.join(a.out, "meta.json"), "w") as fh:
        json.dump(meta, fh, indent=2)
    print("%s: %d IMU rows kept of %d, %.1f s" % (a.out, keep.sum(), n, T))


if __name__ == "__main__":
    main()
