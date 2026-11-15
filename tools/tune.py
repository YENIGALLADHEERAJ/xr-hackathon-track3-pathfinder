#!/usr/bin/env python3
"""
Coordinate-descent tuning of main.py parameters on synthetic degraded runs,
with a held-out validation set.

    python tune.py <workdir> [--rounds 2]
"""
import argparse
import itertools
import json
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))
sys.path.insert(0, HERE)
import main as M          # noqa: E402
import evaluate as E      # noqa: E402

GRID = dict(
    drag_gain=[-1.5, -2.45, -3.5, -5.0],
    tau_v=[1.0, 2.0, 3.0, 5.0, 8.0],
    tau_fd=[0.1, 0.3, 0.6, 1.0],
    tau_p=[5.0, 10.0, 20.0, 40.0],
    shrink_gamma=[0.0, 1.0, 2.0, 4.0],
    tau_c=[10.0, 30.0, 60.0],
    tau_aw=[3.0, 5.0, 10.0],
    tau_ba=[10.0, 30.0],
    tau_vb=[1.0, 2.0, 4.0],
)


def make_sets(work):
    specs = []
    for split, seeds in (("train", [1, 2, 3]), ("val", [11, 12, 13])):
        for prof in ("mav", "handheld"):
            for s in seeds:
                fail = "cut" if s % 2 else "nan"
                dur = 120.0 if s % 3 else 180.0
                name = "%s_%s_%d" % (split, prof, s)
                d = os.path.join(work, name)
                if not os.path.exists(os.path.join(d, "meta.json")):
                    subprocess.check_call([sys.executable, os.path.join(HERE, "simulate.py"), d,
                                           "--profile", prof, "--seed", str(s), "--failure", fail,
                                           "--duration", str(dur)])
                specs.append((split, d))
    return specs


def score(res):
    def ok(k, fallback):
        v = res.get(k, float("nan"))
        return fallback if v != v else v          # NaN-safe (clean runs have no loss window)
    ate = res["ate_rmse_m"]
    return (0.4 * ate + 0.3 * ok("checkpoint_err_mean_m", ate)
            + 0.2 * ok("blind5_rel_err_mean_m", ate) + 0.1 * ok("dropout_err_mean_m", ate))


def run_cfg(prm, sets, work):
    out = os.path.join(work, "_est.txt")
    rs = []
    for d in sets:
        M.run(d, out, prm)
        meta = json.load(open(os.path.join(d, "meta.json")))
        r = E.evaluate(os.path.join(d, "groundtruth.txt"), out, meta)
        r["J"] = score(r)
        rs.append(r)
    return rs


def summary(rs):
    keys = ["J", "ate_rmse_m", "checkpoint_err_mean_m", "blind5_rel_err_mean_m",
            "dropout_err_mean_m", "rpe_trans_1s_m", "rpe_rot_1s_deg"]
    return {k: float(np.nanmean([r[k] for r in rs])) for k in keys}


def zero_baseline(sets):
    """ATE of an estimator that never moves (lower bar any method must beat)."""
    v = []
    for d in sets:
        _, p, _ = E.load_tum(os.path.join(d, "groundtruth.txt"))
        v.append(float(np.sqrt(np.mean(np.sum((p - p.mean(0)) ** 2, 1)))))
    return float(np.mean(v))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("work")
    ap.add_argument("--rounds", type=int, default=2)
    ap.add_argument("--extra-train", nargs="*", default=[], help="additional run dirs (e.g. real EuRoC)")
    ap.add_argument("--extra-val", nargs="*", default=[])
    ap.add_argument("--fix", nargs="*", default=[], help="key=val held fixed during tuning")
    ap.add_argument("--start", nargs="*", default=[], help="key=val starting point (tunable)")
    ap.add_argument("--no-synthetic-train", action="store_true")
    a = ap.parse_args()
    os.makedirs(a.work, exist_ok=True)
    specs = make_sets(a.work)
    train = ([] if a.no_synthetic_train else [d for s, d in specs if s == "train"]) + a.extra_train
    val = [d for s, d in specs if s == "val"] + a.extra_val
    fixed = {kv.split("=")[0]: float(kv.split("=")[1]) for kv in a.fix}
    print("zero-motion baseline ATE  train %.3f  val %.3f" % (zero_baseline(train), zero_baseline(val)))
    best = dict(M.DEFAULTS)
    best.update({kv.split("=")[0]: float(kv.split("=")[1]) for kv in a.start})
    best.update(fixed)
    bestJ = summary(run_cfg(best, train, a.work))["J"]
    print("defaults        train J=%.3f" % bestJ)
    for rnd in range(a.rounds):
        for k, vals in GRID.items():
            if k in fixed:
                continue
            for v in vals:
                if v == best[k]:
                    continue
                cand = dict(best)
                cand[k] = v
                J = summary(run_cfg(cand, train, a.work))["J"]
                if J < bestJ - 1e-4:
                    best, bestJ = cand, J
                    print("  round %d  %s=%g  -> J=%.3f" % (rnd, k, v, J), flush=True)
    print("best params:", json.dumps({k: best[k] for k in GRID}))
    print("tuned  train:", json.dumps(summary(run_cfg(best, train, a.work))))
    print("tuned  val  :", json.dumps(summary(run_cfg(best, val, a.work))))
    for d in val:
        r = run_cfg(best, [d], a.work)[0]
        print("   %-28s ATE %.3f  zero %.3f  rot %.2f deg/s" % (os.path.basename(d), r["ate_rmse_m"],
              zero_baseline([d]), r["rpe_rot_1s_deg"]))
    print("default val :", json.dumps(summary(run_cfg(dict(M.DEFAULTS), val, a.work))))
    json.dump(best, open(os.path.join(a.work, "best_params.json"), "w"), indent=2)


if __name__ == "__main__":
    main()
