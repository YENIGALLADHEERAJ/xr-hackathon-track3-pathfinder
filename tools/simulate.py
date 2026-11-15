#!/usr/bin/env python3
"""
Synthetic degraded-IMU sequence generator (stand-in for the organizer's
degradation script, which is applied to EuRoC MAV / TUM-VI).

Writes a EuRoC-style folder:
    <out>/mav0/imu0/data.csv            degraded IMU stream (input to main.py)
    <out>/groundtruth.txt               TUM-format ground truth incl. blind 5 s
    <out>/meta.json                     checkpoint times, failure time, ...

Profiles
    mav       quadrotor: thrust axis aligned with (a + g), yaw free  (EuRoC-like)
    handheld  free 3-D rotation, walking-scale motion               (TUM-VI-like)

Degradation (progressive over the mission)
    * white noise ramping up to `noise_gain` x EuRoC nominal
    * constant bias + escalating bias random walk / linear drift
    * impulsive spikes, single/burst frame dropouts, one 2 s signal loss
    * IMU death: the last `blind` seconds are either removed ("cut") or
      present as NaN rows ("nan")
"""
import argparse
import json
import os

import numpy as np

G = 9.81


def euler_zyx(yaw, pitch, roll):
    cy, sy = np.cos(yaw), np.sin(yaw)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cr, sr = np.cos(roll), np.sin(roll)
    R = np.empty(yaw.shape + (3, 3))
    R[..., 0, 0] = cy * cp
    R[..., 0, 1] = cy * sp * sr - sy * cr
    R[..., 0, 2] = cy * sp * cr + sy * sr
    R[..., 1, 0] = sy * cp
    R[..., 1, 1] = sy * sp * sr + cy * cr
    R[..., 1, 2] = sy * sp * cr - cy * sr
    R[..., 2, 0] = -sp
    R[..., 2, 1] = cp * sr
    R[..., 2, 2] = cp * cr
    return R


def rot_to_quat(R):
    """Batch rotation matrices -> (x, y, z, w)."""
    tr = R[:, 0, 0] + R[:, 1, 1] + R[:, 2, 2]
    q = np.empty((len(R), 4))
    for i in range(len(R)):
        m = R[i]
        if tr[i] > 0:
            s = np.sqrt(tr[i] + 1.0) * 2
            q[i] = [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s]
        elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
            s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
            q[i] = [0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s]
        elif m[1, 1] > m[2, 2]:
            s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
            q[i] = [(m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s]
        else:
            s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
            q[i] = [(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s, (m[1, 0] - m[0, 1]) / s]
    q[q[:, 3] < 0] *= -1
    return q


def smooth_sum(t, rng, n, fmin, fmax, amp):
    out = np.zeros_like(t)
    for _ in range(n):
        f = rng.uniform(fmin, fmax)
        out += rng.uniform(0.3, 1.0) * amp / n * np.sin(2 * np.pi * f * t + rng.uniform(0, 2 * np.pi))
    return out


def envelope(t, t_static, ramp=2.0):
    """C-infinity 0 -> 1 transition (a quadrotor's attitude depends on jerk)."""
    x = np.clip((t - t_static) / ramp, 0, 1)
    with np.errstate(divide="ignore"):
        a = np.where(x > 0, np.exp(-1.0 / np.maximum(x, 1e-12)), 0.0)
        b = np.where(x < 1, np.exp(-1.0 / np.maximum(1 - x, 1e-12)), 0.0)
    return a / (a + b)


def generate(profile, duration, seed, rate=200.0, t_static=3.0, fine=5):
    rng = np.random.default_rng(seed)
    hz = rate * fine
    t = np.arange(0, duration, 1.0 / hz)
    env = envelope(t, t_static)
    if profile == "mav":
        p = np.stack([env * smooth_sum(t, rng, 4, 0.02, 0.2, 5.0),
                      env * smooth_sum(t, rng, 4, 0.02, 0.2, 4.0),
                      env * (1.0 + smooth_sum(t, rng, 3, 0.02, 0.15, 1.0))], 1)
        p[:, 2] -= p[0, 2]
        dt = 1.0 / hz
        a = np.gradient(np.gradient(p, dt, axis=0), dt, axis=0)
        yaw = env * smooth_sum(t, rng, 3, 0.01, 0.1, 3.0)
        thrust = a + np.array([0, 0, G])
        zb = thrust / np.linalg.norm(thrust, axis=1, keepdims=True)
        xc = np.stack([np.cos(yaw), np.sin(yaw), np.zeros_like(yaw)], 1)
        yb = np.cross(zb, xc)
        yb /= np.linalg.norm(yb, axis=1, keepdims=True)
        xb = np.cross(yb, zb)
        R = np.stack([xb, yb, zb], 2)                    # columns = body axes in world
    else:  # handheld
        p = np.stack([env * smooth_sum(t, rng, 5, 0.03, 0.3, 4.0),
                      env * smooth_sum(t, rng, 5, 0.03, 0.3, 4.0),
                      env * smooth_sum(t, rng, 4, 0.1, 1.0, 0.3)], 1)
        dt = 1.0 / hz
        a = np.gradient(np.gradient(p, dt, axis=0), dt, axis=0)
        yaw = env * smooth_sum(t, rng, 4, 0.02, 0.3, 4.0)
        pitch = env * smooth_sum(t, rng, 4, 0.05, 0.6, 0.8)
        roll = env * smooth_sum(t, rng, 4, 0.05, 0.6, 0.8)
        R = euler_zyx(yaw, pitch, roll)
    # arbitrary initial attitude / IMU mounting ("unknown gravity alignment")
    tilt = rng.uniform(-0.35, 0.35, 2)
    R0 = euler_zyx(np.array(rng.uniform(-np.pi, np.pi)), np.array(tilt[0]), np.array(tilt[1]))
    R = R @ R0                                           # world <- imu (rigid mount)
    dt = 1.0 / hz
    dR = np.gradient(R, dt, axis=0)
    W = np.einsum("nji,njk->nik", R, dR)                 # R^T dR = [w]x
    w = np.stack([W[:, 2, 1] - W[:, 1, 2], W[:, 0, 2] - W[:, 2, 0], W[:, 1, 0] - W[:, 0, 1]], 1) / 2
    f = np.einsum("nji,nj->ni", R, a + np.array([0, 0, G]))
    sl = slice(0, None, fine)
    return t[sl], p[sl], R[sl], w[sl], f[sl]


def degrade(t, w, f, rng, noise_gain=10.0, bias_gain=1.0):
    n = len(t)
    T = t[-1]
    u = t / T
    sg0, sa0 = 1.7e-4 * np.sqrt(200), 2.0e-3 * np.sqrt(200)
    sg = sg0 * (1 + (noise_gain - 1) * u ** 2)
    sa = sa0 * (1 + (noise_gain - 1) * u ** 2)
    dt = t[1] - t[0]
    bg = rng.normal(0, 0.004, 3) * bias_gain
    ba = rng.normal(0, 0.04, 3) * bias_gain
    rwg = np.cumsum(rng.normal(0, 1, (n, 3)) * (1.9e-5 * (1 + 20 * u[:, None] ** 2)) * np.sqrt(dt), 0) * bias_gain
    rwa = np.cumsum(rng.normal(0, 1, (n, 3)) * (3.0e-3 * (1 + 20 * u[:, None] ** 2)) * np.sqrt(dt), 0) * bias_gain
    drift_g = np.outer(u ** 2, rng.normal(0, 0.01, 3)) * bias_gain
    drift_a = np.outer(u ** 2, rng.normal(0, 0.1, 3)) * bias_gain
    wd = w + bg + rwg + drift_g + rng.normal(0, 1, (n, 3)) * sg[:, None]
    fd = f + ba + rwa + drift_a + rng.normal(0, 1, (n, 3)) * sa[:, None]
    # impulsive spikes
    ns = int(0.0005 * n)
    idx = rng.integers(0, n, ns)
    wd[idx] += rng.normal(0, 5, (ns, 3))
    fd[idx] += rng.normal(0, 40, (ns, 3))
    return wd, fd


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--profile", choices=["mav", "handheld"], default="mav")
    ap.add_argument("--duration", type=float, default=120.0, help="mission length incl. blind part [s]")
    ap.add_argument("--blind", type=float, default=5.0)
    ap.add_argument("--failure", choices=["cut", "nan"], default="cut")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--noise-gain", type=float, default=10.0)
    ap.add_argument("--bias-gain", type=float, default=1.0)
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed + 1000)
    t, p, R, w, f = generate(a.profile, a.duration, a.seed)
    wd, fd = degrade(t, w, f, rng, a.noise_gain, a.bias_gain)
    t0_ns = 1403636579758555392
    ts = t0_ns + np.round(t * 1e9).astype(np.int64)
    n = len(t)
    t_fail = t[-1] - a.blind
    keep = np.ones(n, bool)
    nanrow = np.zeros(n, bool)
    # frame dropouts, probability rising with mission progress
    u = t / t[-1]
    drop = rng.random(n) < 0.01 * u
    burst_starts = np.where(rng.random(n) < 0.0008 * u)[0]
    for s in burst_starts:
        drop[s:s + rng.integers(5, 40)] = True
    # one 2 s signal loss in the last third (before failure)
    s2 = rng.uniform(0.6, 0.85) * t_fail
    loss = (t >= s2) & (t < s2 + 2.0)
    drop |= loss
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
                fh.write("%d,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f\n" % (ts[i], *wd[i], *fd[i]))
    q = rot_to_quat(R)
    with open(os.path.join(a.out, "groundtruth.txt"), "w", newline="\n") as fh:
        fh.write("# timestamp tx ty tz qx qy qz qw\n")
        for i in range(n):
            fh.write("%d.%09d %.6f %.6f %.6f %.9f %.9f %.9f %.9f\n" % (
                ts[i] // 10 ** 9, ts[i] % 10 ** 9, *p[i], *q[i]))
    cps = [float(x) for x in np.arange(10.0, t_fail + 1e-9, 10.0)] + [float(t_fail)]
    meta = dict(profile=a.profile, seed=a.seed, failure=a.failure,
                t0=float(ts[0]) * 1e-9, t_fail=float(ts[0]) * 1e-9 + t_fail,
                checkpoints=[float(ts[0]) * 1e-9 + c for c in cps],
                signal_loss=[float(ts[0]) * 1e-9 + s2, float(ts[0]) * 1e-9 + s2 + 2.0],
                rows=int(keep.sum()), gt_rows=int(n))
    with open(os.path.join(a.out, "meta.json"), "w") as fh:
        json.dump(meta, fh, indent=2)


if __name__ == "__main__":
    main()
