#!/usr/bin/env python3
"""
Build evaluation runs from a real EuRoC sequence with the ORGANIZERS'
degradation script (degrade_imu.py from the Track 3 student package).

    python make_official_runs.py <euroc_seq_dir> <out_root> --script path/to/degrade_imu.py
                                 [--modes euroc accelfirst direct] [--levels 1.0 0.6]
                                 [--gammas 1.7 1.0] [--gap2s]

degrade_imu.py documents its input as `timestamp, accel(3), gyro(3)`, while
EuRoC (and the Track 3 input spec) is `timestamp, gyro(3), accel(3)`.  The
estimator must be right whichever way the organizers fed it, so three modes:

    euroc       reorder to accel-first, degrade, reorder back to EuRoC order
                (the intended degradation, delivered in the slide-15 format)
    accelfirst  degrade accel-first and leave it accel-first
                (tests column-order auto-detection)
    direct      run the script straight on the EuRoC file: it then treats the
                gyro columns as accelerometer and applies accel-level noise,
                bias and 3-9 rad/s spikes to the gyro (worst case)

The IMU is first trimmed to the ground-truth span so that the 5 s failure
window (all-zero rows written by the script) always has ground truth.
--gap2s additionally removes a 2 s chunk of rows (the FAQ's "2-second
dropouts"), placed at 60-85% of the pre-failure span.

Each run directory gets mav0/imu0/data.csv, groundtruth.txt (TUM, every
original IMU stamp) and meta.json for tools/evaluate.py.
"""
import argparse
import json
import os
import subprocess
import sys

import numpy as np

HDR_EUROC = ("#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],"
             "a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]")
HDR_ACCEL_FIRST = ("#timestamp [ns],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2],"
                   "w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1]")


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


def write_csv(path, header, ts, cols):
    with open(path, "w", newline="\n") as f:
        f.write(header + "\n")
        for i in range(len(ts)):
            f.write("%d,%s\n" % (ts[i], ",".join(repr(float(v)) for v in cols[i])))


def read_csv(path):
    rows = []
    with open(path) as f:
        for line in f:
            if not line.strip() or not line[0].isdigit() and line[0] != "-":
                continue
            p = line.strip().split(",")
            rows.append((int(p[0]), [float(x) for x in p[1:7]]))
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("seq")
    ap.add_argument("out")
    ap.add_argument("--script", required=True)
    ap.add_argument("--modes", nargs="*", default=["euroc", "accelfirst", "direct"])
    ap.add_argument("--levels", nargs="*", type=float, default=[1.0])
    ap.add_argument("--gammas", nargs="*", type=float, default=[1.7])
    ap.add_argument("--gap2s", action="store_true")
    ap.add_argument("--tag", default=None)
    a = ap.parse_args()

    imu = np.loadtxt(os.path.join(a.seq, "mav0", "imu0", "data.csv"), delimiter=",", comments="#")
    gt = np.loadtxt(os.path.join(a.seq, "mav0", "state_groundtruth_estimate0", "data.csv"),
                    delimiter=",", comments="#")
    ts = imu[:, 0].astype(np.int64)
    gts = gt[:, 0].astype(np.int64)
    m = (ts >= gts[0]) & (ts <= gts[-1])
    ts, w, f = ts[m], imu[m, 1:4], imu[m, 4:7]
    k = np.clip(np.searchsorted(gts, ts) - 1, 0, len(gts) - 2)
    al = ((ts - gts[k]) / (gts[k + 1] - gts[k])).astype(float)
    P = gt[k, 1:4] * (1 - al[:, None]) + gt[k + 1, 1:4] * al[:, None]
    Q = slerp(gt[k, 4:8], gt[k + 1, 4:8], al)
    Q[Q[:, 0] < 0] *= -1
    t0 = ts[0] * 1e-9
    T = (ts[-1] - ts[0]) * 1e-9
    tag = a.tag or os.path.basename(os.path.normpath(a.seq)).split("_")[0] + os.path.basename(
        os.path.normpath(a.seq)).split("_")[1]

    for mode in a.modes:
        for lvl in a.levels:
            for gam in a.gammas:
                name = "%s_%s_L%g_g%g%s" % (tag, mode, lvl, gam, "_gap2s" if a.gap2s else "")
                d = os.path.join(a.out, name)
                os.makedirs(os.path.join(d, "mav0", "imu0"), exist_ok=True)
                clean = os.path.join(d, "_clean.csv")
                deg = os.path.join(d, "_degraded.csv")
                if mode == "direct":
                    write_csv(clean, HDR_EUROC, ts, np.c_[w, f])
                else:
                    write_csv(clean, HDR_ACCEL_FIRST, ts, np.c_[f, w])
                r = subprocess.run([sys.executable, a.script, clean, deg, "--max-level", str(lvl),
                                    "--gamma", str(gam), "--meta", os.path.join(d, "degrade_stats.json")],
                                   capture_output=True, text=True)
                if r.returncode:
                    raise SystemExit(r.stderr)
                rows = read_csv(deg)
                if a.gap2s:
                    t_rel = np.array([(t - ts[0]) * 1e-9 for t, _ in rows])
                    rng = np.random.default_rng(int(lvl * 100 + gam * 10))
                    s2 = rng.uniform(0.6, 0.85) * (T - 5.0)
                    rows = [r_ for r_, tr in zip(rows, t_rel) if not (s2 <= tr < s2 + 2.0)]
                header = HDR_ACCEL_FIRST if mode == "accelfirst" else HDR_EUROC
                with open(os.path.join(d, "mav0", "imu0", "data.csv"), "w", newline="\n") as fo:
                    fo.write(header + "\n")
                    for t, v in rows:
                        if mode == "euroc":           # back to EuRoC order: gyro, accel
                            v = v[3:6] + v[0:3]
                        fo.write("%d,%s\n" % (t, ",".join("%.6f" % x for x in v)))
                os.remove(clean)
                os.remove(deg)
                with open(os.path.join(d, "groundtruth.txt"), "w", newline="\n") as fh:
                    fh.write("# timestamp tx ty tz qx qy qz qw\n")
                    for i in range(len(ts)):
                        fh.write("%d.%09d %.6f %.6f %.6f %.9f %.9f %.9f %.9f\n" % (
                            ts[i] // 10 ** 9, ts[i] % 10 ** 9, *P[i], Q[i, 1], Q[i, 2], Q[i, 3], Q[i, 0]))
                # dropout window = the largest timestamp gap before failure (+3 s after)
                tk = np.array([t for t, _ in rows], dtype=np.int64)
                gaps = np.diff(tk) * 1e-9
                pre = (tk[1:] - ts[0]) * 1e-9 < T - 5.0
                j = int(np.argmax(np.where(pre, gaps, 0)))
                g0, g1 = tk[j] * 1e-9, tk[j + 1] * 1e-9
                t_fail = t0 + T - 5.0
                cps = [float(x) for x in np.arange(10.0, T - 5.0 + 1e-9, 10.0)] + [T - 5.0]
                meta = dict(source=os.path.abspath(a.seq), real=True, mode=mode, max_level=lvl, gamma=gam,
                            gap2s=a.gap2s, t0=t0, t_fail=t_fail, checkpoints=[t0 + c for c in cps],
                            signal_loss=[g0, g1], largest_gap_s=float(gaps[j]),
                            rows=len(rows), gt_rows=int(len(ts)), duration_s=float(T))
                with open(os.path.join(d, "meta.json"), "w") as fh:
                    json.dump(meta, fh, indent=2)
                print("%-40s rows %6d (clean %6d)  largest gap %.3f s" % (name, len(rows), len(ts), gaps[j]))


if __name__ == "__main__":
    main()
