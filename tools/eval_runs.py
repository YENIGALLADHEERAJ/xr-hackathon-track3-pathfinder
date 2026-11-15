#!/usr/bin/env python3
"""
Score a parameter set on a folder of runs (made by simulate.py / degrade_real.py)
against the stand-still and raw-strapdown baselines.

    python eval_runs.py <runs_dir> [--set key=val ...] [--label NAME]
"""
import argparse
import glob
import json
import os
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, "..", "src"), HERE]
import main as M          # noqa: E402
import evaluate as E      # noqa: E402

PURE = dict(kp=0.0, ki=0.0, tau_ba=1e9, tau_aw=1e9, tau_v=1e9, tau_p=1e9, shrink_gamma=0.0)


def zero(d):
    _, p, _ = E.load_tum(os.path.join(d, "groundtruth.txt"))
    meta = json.load(open(os.path.join(d, "meta.json")))
    t, _, _ = E.load_tum(os.path.join(d, "groundtruth.txt"))
    p = p[t <= meta["t_fail"]]
    return float(np.sqrt(np.mean(np.sum((p - p.mean(0)) ** 2, 1))))


def score_dir(d, prm, tmp):
    out = os.path.join(tmp, "est.txt")
    M.run(d, out, prm)
    meta = json.load(open(os.path.join(d, "meta.json")))
    return E.evaluate(os.path.join(d, "groundtruth.txt"), out, meta)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs")
    ap.add_argument("--set", nargs="*", default=[])
    ap.add_argument("--pure", action="store_true", help="also score raw strapdown")
    a = ap.parse_args()
    prm = dict(M.DEFAULTS)
    for kv in a.set:
        k, v = kv.split("=", 1)
        prm[k] = float(v)
    tmp = tempfile.mkdtemp()
    hdr = "%-22s %6s %8s %7s %7s %7s %8s %7s" % ("run", "zero", "pure", "ATE", "ckpt", "blind5", "dropout", "rot°/s")
    print(hdr)
    rows = []
    for d in sorted(glob.glob(os.path.join(a.runs, "*"))):
        if not os.path.exists(os.path.join(d, "meta.json")):
            continue
        r = score_dir(d, prm, tmp)
        pu = score_dir(d, dict(prm, **PURE), tmp)["ate_rmse_m"] if a.pure else float("nan")
        z = zero(d)
        rows.append((z, r["ate_rmse_m"]))
        print("%-22s %6.2f %8.1f %7.2f %7.2f %7.2f %8.2f %7.2f" % (
            os.path.basename(d), z, pu, r["ate_rmse_m"], r["checkpoint_err_mean_m"],
            r.get("blind5_rel_err_mean_m", float("nan")), r["dropout_err_mean_m"], r["rpe_rot_1s_deg"]))
    z, e = np.array(rows).T
    print("mean: zero %.3f  ATE %.3f  (%+.1f%% vs stand-still)" % (z.mean(), e.mean(), 100 * (e.mean() / z.mean() - 1)))


if __name__ == "__main__":
    main()
