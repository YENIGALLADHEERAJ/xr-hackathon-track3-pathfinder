#!/usr/bin/env python3
"""
Trajectory evaluation mirroring the organizer's evo-based ranking.

    python evaluate.py groundtruth.txt estimated_trajectory.txt [--meta meta.json]

Metrics
    ATE RMSE  - after SE(3) Umeyama alignment (evo_ape tum gt est -a)
    RPE       - translational / rotational error over 1 s windows (evo_rpe --delta 1 --delta_unit s)
    Checkpts  - aligned position error at mission checkpoints        (tier T1)
    Blind 5 s - error after IMU failure, alignment from pre-failure  (tier T2)
    Dropout   - error over the 2 s signal loss + 3 s after           (tier T3)
"""
import argparse
import json

import numpy as np


def load_tum(path):
    d = np.loadtxt(path, comments="#")
    return d[:, 0], d[:, 1:4], d[:, 4:8]


def associate(t_gt, t_est, tol=1e-3):
    idx = np.searchsorted(t_gt, t_est)
    idx = np.clip(idx, 1, len(t_gt) - 1)
    left = t_gt[idx - 1]
    right = t_gt[idx]
    idx = np.where(np.abs(t_est - left) < np.abs(t_est - right), idx - 1, idx)
    ok = np.abs(t_gt[idx] - t_est) < tol
    return idx[ok], np.where(ok)[0]


def umeyama(src, dst):
    """R, t minimising |dst - (R src + t)|."""
    ms, md = src.mean(0), dst.mean(0)
    C = (dst - md).T @ (src - ms) / len(src)
    U, _, Vt = np.linalg.svd(C)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    return R, md - R @ ms


def quat_to_rot(q):
    x, y, z, w = q.T
    R = np.empty((len(q), 3, 3))
    R[:, 0, 0] = 1 - 2 * (y * y + z * z); R[:, 0, 1] = 2 * (x * y - z * w); R[:, 0, 2] = 2 * (x * z + y * w)
    R[:, 1, 0] = 2 * (x * y + z * w); R[:, 1, 1] = 1 - 2 * (x * x + z * z); R[:, 1, 2] = 2 * (y * z - x * w)
    R[:, 2, 0] = 2 * (x * z - y * w); R[:, 2, 1] = 2 * (y * z + x * w); R[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return R


def rot_angle(R):
    c = (np.trace(R, axis1=-2, axis2=-1) - 1) / 2
    return np.degrees(np.arccos(np.clip(c, -1, 1)))


def rpe(t, P, Rm, Pg, Rg, delta=1.0):
    j = np.searchsorted(t, t + delta)
    ok = j < len(t)
    i, j = np.where(ok)[0], j[ok]
    ok = np.abs(t[j] - t[i] - delta) < 0.05
    i, j = i[ok], j[ok]
    if len(i) == 0:
        return float("nan"), float("nan")
    de = np.einsum("nji,nj->ni", Rm[i], P[j] - P[i])
    dg = np.einsum("nji,nj->ni", Rg[i], Pg[j] - Pg[i])
    Re = np.einsum("nji,njk->nik", Rm[i], Rm[j])
    Rgg = np.einsum("nji,njk->nik", Rg[i], Rg[j])
    E = np.einsum("nji,njk->nik", Rgg, Re)
    trans = np.sqrt(np.mean(np.sum((dg - de) ** 2, 1)))
    rot = np.sqrt(np.mean(rot_angle(E) ** 2))
    return float(trans), float(rot)


def evaluate(gt_path, est_path, meta=None):
    tg, pg, qg = load_tum(gt_path)
    te, pe, qe = load_tum(est_path)
    gi, ei = associate(tg, te)
    t, Pg, Pe = tg[gi], pg[gi], pe[ei]
    Rg, Re = quat_to_rot(qg[gi]), quat_to_rot(qe[ei])
    res = dict(est_rows=int(len(te)), matched=int(len(gi)), gt_rows=int(len(tg)))
    t_fail = meta["t_fail"] if meta else t[-1] + 1
    pre = t <= t_fail
    R, tr = umeyama(Pe[pre], Pg[pre])
    Pa = Pe @ R.T + tr
    Ra = np.einsum("ij,njk->nik", R, Re)
    err = np.linalg.norm(Pa - Pg, axis=1)
    res["ate_rmse_m"] = float(np.sqrt(np.mean(err[pre] ** 2)))
    R2, t2 = umeyama(Pe, Pg)
    res["ate_rmse_all_m"] = float(np.sqrt(np.mean(np.sum((Pe @ R2.T + t2 - Pg) ** 2, 1))))
    res["rpe_trans_1s_m"], res["rpe_rot_1s_deg"] = rpe(t[pre], Pa[pre], Ra[pre], Pg[pre], Rg[pre])
    res["gt_path_length_m"] = float(np.sum(np.linalg.norm(np.diff(Pg[pre], axis=0), axis=1)))
    if meta:
        cps = np.array(meta["checkpoints"])
        k = np.clip(np.searchsorted(t, cps), 0, len(t) - 1)
        res["checkpoint_err_mean_m"] = float(err[k].mean())
        blind = t > t_fail
        if blind.any():
            # blind error measured relative to the pose at failure (drift of the dead reckoning)
            kf = np.searchsorted(t, t_fail) - 1
            rel_e = Pa[blind] - Pa[kf]
            rel_g = Pg[blind] - Pg[kf]
            e = np.linalg.norm(rel_e - rel_g, axis=1)
            res["blind_rows_scored"] = int(blind.sum())
            res["blind5_abs_err_mean_m"] = float(err[blind].mean())
            res["blind5_rel_err_mean_m"] = float(e.mean())
            res["blind5_rel_err_end_m"] = float(e[-1])
            res["blind5_gt_displacement_m"] = float(np.linalg.norm(rel_g[-1]))
        else:
            res["blind_rows_scored"] = 0
        s0, s1 = meta["signal_loss"]
        w = (t >= s0) & (t <= s1 + 3.0)
        res["dropout_err_mean_m"] = float(err[w].mean()) if w.any() else float("nan")
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("gt")
    ap.add_argument("est")
    ap.add_argument("--meta")
    a = ap.parse_args()
    meta = json.load(open(a.meta)) if a.meta else None
    for k, v in evaluate(a.gt, a.est, meta).items():
        print("%-28s %s" % (k, ("%.4f" % v) if isinstance(v, float) else v))


if __name__ == "__main__":
    main()
