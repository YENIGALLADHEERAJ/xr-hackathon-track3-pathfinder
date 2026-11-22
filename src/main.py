#!/usr/bin/env python3
"""
Path Finder -- XR Hackathon 2026, Track 03
Trajectory estimation under sensor degradation (IMU only).

    degraded EuRoC IMU CSV  ->  6-DoF trajectory in TUM format
    (one pose per IMU frame, plus 5 s of post-failure dead reckoning)

Usage
-----
    python main.py <imu0/data.csv | dataset_dir | dataset.zip> [-o estimated_trajectory.txt]

Design
------
* Single CPU thread, streaming, O(1) memory: rows are read, filtered and written
  one at a time.  Pure Python standard library (no numpy) so the resident
  footprint is the interpreter itself.
* Robust front-end: NaN / empty / all-zero / out-of-range / frozen rows are
  treated as dropouts; impulsive spikes are gated against a noise-adaptive
  threshold; timestamp gaps are bridged by blind propagation.
* Attitude: exact quaternion integration of the bias-corrected gyro.  A
  Mahony-style gravity correction (gain weighted by how close |accel| is to
  1 g) is applied while the platform is static; in motion it is off by default
  because a multirotor's accelerometer measures thrust, not gravity.  Gravity
  alignment is estimated from the data (no assumption on IMU mounting).
* Translation: band-pass double integration -- slow body/world acceleration
  mean removal (absorbs bias drift and tilt error), leaky velocity, a position
  prior toward the mission origin, ZUPT when static, and a noise-adaptive
  shrinkage that trusts integrated motion less as the sensor degrades.  IMU-only
  position is unobservable at low frequency; this keeps the trajectory bounded
  while retaining the short-term motion the IMU does measure well.
* Dead reckoning: during dropouts and after the IMU dies the last filtered
  velocity and angular rate are extrapolated with exponential decay.

Output frame: z-up, gravity aligned, origin = start position, yaw = 0 at start.
(The ground-truth frame is unknown to an IMU-only system; evo's SE(3)
alignment `-a` maps between them.)
"""

import io
import math
import os
import struct
import sys
import time
from array import array

# zipfile, decimal, json, platform and tempfile are imported only where they are
# needed: together with argparse they would add ~3 MB to the heap at start-up.

# --- module-level math aliases (avoid attribute lookup per call) ---
_sqrt = math.sqrt
_sin = math.sin
_cos = math.cos
_exp = math.exp

# --------------------------------------------------------------------------
# Tunable parameters. Selected on real EuRoC V1_01 degraded with the organizers'
# degrade_imu.py (13 runs, 3 input modes), confirmed on held-out V1_02. kp/ki = 0: on a
# multirotor the accelerometer points along thrust, not gravity, so tilt
# correction toward it hurts.
# --------------------------------------------------------------------------
DEFAULTS = dict(
    init_s=1.0,          # seconds used for initial gravity / bias alignment
    kp=0.0,              # gravity-correction gain (rad/s per unit error)
    kp_static=2.0,       # gain while static
    ki=0.0,              # gyro-bias integral gain
    tau_ba=30.0,         # body-frame accel-bias / mean tracking time constant [s]
    tau_aw=5.0,          # world-frame acceleration mean removal [s]
    tau_v=1.0,           # velocity leak / drag-fusion time constant [s]
    tau_p=5.0,           # position prior (leak to origin) time constant [s]
    tau_vb=0.25,         # velocity decay during blind dead reckoning [s]
    tau_wb=0.1,          # angular-rate decay during blind dead reckoning [s]
    gap_factor=2.5,      # dt > gap_factor * nominal -> dropout
    spike_k=8.0,         # spike gate: k * diff-noise + floor
    static_hold_s=1e9,   # stillness needed for in-flight ZUPT [s]. 1e9 = only the
                         # initial static segment: on real EuRoC, rotor vibration made
                         # slow flight look static and corrupted the gyro bias
    horizon_s=5.0,       # post-failure dead-reckoning horizon [s]
    shrink_gamma=0.0,    # position shrinkage exponent vs. noise growth (0 = off)
    out_gain=1.2,        # output position gain: the leaky integrators shrink slow motion
                         # (best ATE/RPE across all four validation sequences, see README)
    interp_max=0.5,      # gaps / invalid runs up to this long [s] are filled by interpolating
                         # between the samples on either side (offline); longer ones dead-reckon
    # --- rotor-drag velocity aiding (multirotors), gated by a multirotor detector ---
    drag=1.0,            # 1 = enabled (detector keeps it off on non-multirotor data)
    drag_gain=-2.45,     # velocity per unit lateral specific force [s] = -1/k (EuRoC Firefly)
    tau_n=20.0,          # thrust-axis tracking [s]
    tau_c=30.0,          # lateral specific-force offset tracking [s]
    tau_fd=0.6,          # low-pass of lateral specific force [s]
    tau_spread=5.0,      # multirotor detector averaging [s]
    spread_lo=0.14,      # rms angle f vs thrust axis [rad] below which drag is fully on (8 deg)
    spread_hi=0.26,      # ... above which it is off (15 deg)
    drag_warm=15.0,      # seconds of motion before drag can engage
    vib_lo=0.8,          # accel vibration [m/s^2] below which drag is off (handheld)
    vib_hi=1.2,          # ... above which it is fully on (rotors spinning)
    rest_search=1.0,     # 1 = if the start is not static, take the gyro bias from the
                         # quietest 1 s window anywhere in the file (offline pre-pass)
    rest_gyro_std=0.05,  # max mean per-axis gyro std [rad/s] for a window to count as rest
    rest_ratio=3.0,      # max sub-window-mean spread / white-noise expectation (rest test)
    rest_tilt_deg=0.25,  # max gravity-direction change between sub-windows (rest test) [deg]
    # --- offline two-pass smoothing (the file is complete before output is due) ---
    smooth=1.0,          # 1 = forward-backward (zero-phase) velocity/position smoothing
    tau_vs=0.75,         # smoother velocity leak / drag-fusion time constant [s]
    tau_ps=15.0,         # smoother position prior time constant [s]
    tau_vbs=2.0,         # smoother velocity decay across measurement gaps [s]
)

GYRO_MAX = 40.0    # rad/s   beyond this a sample is considered garbage
ACC_MAX = 200.0    # m/s^2


# --------------------------------------------------------------------------
# small vector / quaternion helpers  (quaternion = (w, x, y, z), body->world)
# --------------------------------------------------------------------------
def q_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def q_norm(q):
    n = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


def q_integrate(q, wx, wy, wz, dt):
    """Right-multiply by the exact exponential of body rate * dt."""
    ang = math.sqrt(wx * wx + wy * wy + wz * wz) * dt
    if ang < 1e-12:
        return q
    h = 0.5 * ang
    s = math.sin(h) / (ang / dt)
    return q_norm(q_mul(q, (math.cos(h), wx * s, wy * s, wz * s)))


def q_to_rot(q):
    w, x, y, z = q
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return ((1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)),
            (2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)),
            (2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)))


def q_from_two_vectors(u, v):
    """Shortest-arc rotation taking unit vector u onto unit vector v."""
    d = u[0] * v[0] + u[1] * v[1] + u[2] * v[2]
    cx = u[1] * v[2] - u[2] * v[1]
    cy = u[2] * v[0] - u[0] * v[2]
    cz = u[0] * v[1] - u[1] * v[0]
    if d < -0.999999:                       # anti-parallel: rotate pi about x or y
        ax = (1.0, 0.0, 0.0) if abs(u[0]) < 0.9 else (0.0, 1.0, 0.0)
        cx = u[1] * ax[2] - u[2] * ax[1]
        cy = u[2] * ax[0] - u[0] * ax[2]
        cz = u[0] * ax[1] - u[1] * ax[0]
        return q_norm((0.0, cx, cy, cz))
    return q_norm((1.0 + d, cx, cy, cz))


# --------------------------------------------------------------------------
# Estimator
# --------------------------------------------------------------------------
class PathFinder:
    __slots__ = (
        "p", "_qw", "_qx", "_qy", "_qz",
        "pos", "v", "vs", "ws", "bg", "ba", "mw", "g", "t_prev", "dt_nom",
        "last_raw", "stale", "ref", "dn", "rej",
        "nw", "na", "na_min", "t_run", "beta",
        "ew", "ea", "static_t",
        "_gap_factor", "_spike_k", "_tau_v", "_tau_p", "_tau_vb", "_tau_wb",
        "_tau_ba", "_tau_aw", "_kp_static", "_kp", "_ki", "_interp_max",
        "_tau_n", "_tau_c", "_tau_fd", "_tau_spread",
        "_spread_lo", "_spread_hi", "_vib_lo", "_vib_hi",
        "_drag_warm", "_drag_gain", "_drag",
        "_tau_vs", "_tau_ps", "_tau_vbs", "_horizon_s",
        "_out_gain", "_shrink_gamma", "_gravity", "_g15",
        "_inv_tau_v", "_inv_tau_p", "_inv_tau_vb", "_inv_tau_wb",
        "_inv_tau_ba", "_inv_tau_aw", "_inv_tau_n", "_inv_tau_c",
        "_inv_tau_fd", "_inv_tau_spread",
        "_inv_tau_vs", "_inv_tau_ps", "_inv_tau_vbs",
        "tb", "v_hold", "w_hold", "fbar", "eref", "lc", "fd",
        "spread", "gate", "t_drag", "drag_gate_time",
        "n_invalid", "n_spikes", "n_gaps", "blind_time",
        "sink", "last_good", "last_good_ts",
        "pending", "long_gap", "p_last_raw", "p_stale",
        "init_static",
    )

    def __init__(self, prm):
        self.p = prm
        self._qw, self._qx, self._qy, self._qz = 1.0, 0.0, 0.0, 0.0
        self.pos = [0.0, 0.0, 0.0]
        self.v = [0.0, 0.0, 0.0]
        self.vs = [0.0, 0.0, 0.0]          # smoothed velocity (for dead reckoning)
        self.ws = [0.0, 0.0, 0.0]          # smoothed corrected body rate
        self.bg = [0.0, 0.0, 0.0]          # gyro bias
        self.ba = [0.0, 0.0, 0.0]          # accel bias / slow mean (body)
        self.mw = [0.0, 0.0, 0.0]          # world accel slow mean
        self.g = 9.81
        self.t_prev = None
        self.dt_nom = 0.005
        # front-end state
        self.last_raw = None
        self.stale = 0
        self.ref = None                    # last accepted (gated) measurement
        self.dn = [0.0] * 6                # EMA of |diff| per channel
        self.rej = [0] * 6                 # consecutive spike rejections per channel
        self.nw = 0.0                      # gyro diff-noise power (sum of 3 axes)
        self.na = 0.0                      # accel diff-noise power
        self.na_min = 1e9                  # best (lowest) accel noise seen
        self.t_run = 0.0
        self.beta = 1.0                    # output position shrinkage
        self.ew = 1.0                      # gyro energy (static detector)
        self.ea = 1.0                      # accel-magnitude energy
        self.static_t = 0.0
        # --- cached scalar parameters (avoid dict lookups in hot paths) ---
        self._gap_factor = p["gap_factor"]
        self._spike_k = p["spike_k"]
        self._tau_v = p["tau_v"]
        self._tau_p = p["tau_p"]
        self._tau_vb = p["tau_vb"]
        self._tau_wb = p["tau_wb"]
        self._tau_ba = p["tau_ba"]
        self._tau_aw = p["tau_aw"]
        self._kp_static = p["kp_static"]
        self._kp = p["kp"]
        self._ki = p["ki"]
        self._interp_max = p["interp_max"]
        self._tau_n = p["tau_n"]
        self._tau_c = p["tau_c"]
        self._tau_fd = p["tau_fd"]
        self._tau_spread = p["tau_spread"]
        self._spread_lo = p["spread_lo"]
        self._spread_hi = p["spread_hi"]
        self._vib_lo = p["vib_lo"]
        self._vib_hi = p["vib_hi"]
        self._drag_warm = p["drag_warm"]
        self._drag_gain = p["drag_gain"]
        self._drag = p["drag"]
        self._tau_vs = p["tau_vs"]
        self._tau_ps = p["tau_ps"]
        self._tau_vbs = p["tau_vbs"]
        self._horizon_s = p["horizon_s"]
        self._out_gain = p["out_gain"]
        self._shrink_gamma = p["shrink_gamma"]
        self._gravity = 9.81
        self._g15 = 0.15 * 9.81          # 0.15 * g, precomputed for trust calc
        # --- precomputed inverse time constants (avoid division per sample) ---
        self._inv_tau_v = 1.0 / p["tau_v"]
        self._inv_tau_p = 1.0 / p["tau_p"]
        self._inv_tau_vb = 1.0 / p["tau_vb"]
        self._inv_tau_wb = 1.0 / p["tau_wb"]
        self._inv_tau_ba = 1.0 / p["tau_ba"]
        self._inv_tau_aw = 1.0 / p["tau_aw"]
        self._inv_tau_n = 1.0 / p["tau_n"]
        self._inv_tau_c = 1.0 / p["tau_c"]
        self._inv_tau_fd = 1.0 / p["tau_fd"]
        self._inv_tau_spread = 1.0 / p["tau_spread"]
        self._inv_tau_vs = 1.0 / p["tau_vs"]
        self._inv_tau_ps = 1.0 / p["tau_ps"]
        self._inv_tau_vbs = 1.0 / p["tau_vbs"]

        # dead-reckoning state
        self.tb = 0.0                      # time spent blind
        self.v_hold = [0.0, 0.0, 0.0]
        self.w_hold = [0.0, 0.0, 0.0]
        # rotor-drag aiding state (IMU frame unless noted)
        self.fbar = [0.0, 0.0, 9.81]       # slow mean specific force -> thrust axis
        self.eref = 0                      # IMU axis used to build the drag-plane basis
        self.lc = [0.0, 0.0]               # lateral force offsets
        self.fd = [0.0, 0.0]               # low-passed lateral force
        self.spread = 0.0                  # mean squared angle of f vs thrust axis
        self.gate = 0.0
        self.t_drag = 0.0
        self.drag_gate_time = 0.0
        # stats
        self.n_invalid = 0
        self.n_spikes = 0
        self.n_gaps = 0
        self.blind_time = 0.0
        self.sink = None                   # step recorder for the offline smoother
        self.last_good = None              # last accepted measurement (for gap interpolation)
        self.last_good_ts = None
        self.pending = []                  # invalid rows waiting for the next valid sample
        self.long_gap = False
        self.p_last_raw = None             # stale tracking for the pending buffer
        self.p_stale = 0

    # ---------------- initial alignment ----------------
    def initialise(self, samples, dt_nom):
        """samples: list of valid 6-tuples from the first init_s seconds."""
        self.dt_nom = dt_nom
        if not samples:
            return
        # robust to impulsive spikes: drop rows far from the per-channel median
        # (one gyro spike in the first second would otherwise void the static test
        # and leave the gyro bias uncalibrated for the whole flight)
        n = len(samples)
        med = [sorted(s[i] for s in samples)[n // 2] for i in range(6)]
        good = [s for s in samples
                if all(abs(s[i] - med[i]) <= (0.5 if i < 3 else 3.0) for i in range(6))]
        if len(good) >= max(10, n // 2):
            samples = good
        n = len(samples)
        mf = [sum(s[3 + i] for s in samples) / n for i in range(3)]
        mw = [sum(s[i] for s in samples) / n for i in range(3)]
        vw = sum((s[i] - mw[i]) ** 2 for s in samples for i in range(3)) / n
        fn = [math.sqrt(s[3] ** 2 + s[4] ** 2 + s[5] ** 2) for s in samples]
        mfn = sum(fn) / n
        vfn = sum((x - mfn) ** 2 for x in fn) / n
        norm = math.sqrt(mf[0] ** 2 + mf[1] ** 2 + mf[2] ** 2)
        if norm < 1e-6:
            return
        static = vw < 0.05 ** 2 and vfn < 0.3 ** 2 and abs(norm - 9.81) < 1.0
        if static:
            self.bg = mw[:]
            self.g = norm
            self.static_t = self.p["static_hold_s"]
        q = q_from_two_vectors((mf[0] / norm, mf[1] / norm, mf[2] / norm),
                                (0.0, 0.0, 1.0))
        self._qw, self._qx, self._qy, self._qz = q
        self.init_static = static
        # thrust axis starts at the resting specific-force direction; the drag-plane
        # basis uses the IMU axis least aligned with it (fixed for continuity)
        self.fbar = mf[:]
        ab = [abs(mf[0]), abs(mf[1]), abs(mf[2])]
        self.eref = ab.index(min(ab))

    # ---------------- per-sample update ----------------
    def update(self, ts, m, synthetic=False):
        """ts: int nanoseconds; m: 6-tuple (wx,wy,wz,ax,ay,az) or None.
        synthetic=True: m is already gated / interpolated (skip the front-end)."""
        if self.t_prev is None:
            self.t_prev = ts
            if m is not None:
                self.ref = list(m)
                if not synthetic:
                    self.last_good, self.last_good_ts = list(m), ts
            return
        dt = (ts - self.t_prev) * 1e-9
        if dt <= 0.0:                       # duplicate / out of order: hold pose
            return
        t_old = self.t_prev
        self.t_prev = ts
        dtn = self.dt_nom
        if dt > self._gap_factor * dtn:  # timestamp gap -> dropped frames
            self.n_gaps += 1
            gap = dt - dtn
            k = int(gap / dtn + 0.5)
            if k > 20000:
                k = 20000
            lg = self.last_good
            if (self._interp_max > 0.0 and m is not None and lg is not None and k > 0
                    and (ts - self.last_good_ts) * 1e-9 <= self._interp_max):
                mg = m if synthetic else self._gate_inline(m, dtn)
                if mg is not None:          # fill the gap by interpolating the measurements
                    span = float(ts - self.last_good_ts)
                    sub = gap / k
                    for j in range(1, k + 1):
                        f = (t_old + j * sub * 1e9 - self.last_good_ts) / span
                        mj = [lg[i] + f * (mg[i] - lg[i]) for i in range(6)]
                        self.tb = 0.0
                        self._propagate(sub, mj)
                    self.tb = 0.0
                    self._propagate(dtn, mg)
                    if not synthetic:
                        self.last_good, self.last_good_ts = list(mg), ts
                    return
                for _ in range(k):          # gated out: dead-reckon the gap and this row
                    self._blind(sub)
                self.n_invalid += 1
                self._blind(dtn)
                return
            if k > 0:
                sub = gap / k
                for _ in range(k):
                    self._blind(sub)
            dt = dtn
        if not synthetic:
            m = self._gate_inline(m, dt)
        if m is None:
            self.n_invalid += 1
            self._blind(dt)
        else:
            self.tb = 0.0
            self._propagate(dt, m)
            if not synthetic:
                self.last_good, self.last_good_ts = list(m), ts

    def _gate_inline(self, m, dt):
        """Inlined spike gate: replaces _gate() call to avoid method overhead."""
        if m is None:
            return None
        if m == self.last_raw:
            self.stale += 1
        else:
            self.stale = 0
            self.last_raw = m
        if self.stale >= 3:                 # frozen sensor
            return None
        if (m[3] == 0.0 and m[4] == 0.0 and m[5] == 0.0):
            return None
        for i in range(3):
            if abs(m[i]) > GYRO_MAX or abs(m[3 + i]) > ACC_MAX:
                return None
        ref = self.ref
        if ref is None:
            self.ref = list(m)
            return m
        if self.tb > 0.0:                   # after a dropout the old reference is stale
            self.ref = list(m)
            self.rej = [0] * 6
            return m
        k = self._spike_k
        a = dt / 0.5
        if a > 1.0:
            a = 1.0
        out = list(m)
        dn = self.dn
        rej = self.rej
        for i in range(6):
            d = m[i] - ref[i]
            floor = 0.5 if i < 3 else 3.0
            # impulsive spike -> hold reference; a persistent level change
            # (3 consecutive rejections) is real and is accepted
            if abs(d) > k * dn[i] + floor and rej[i] < 2:
                out[i] = ref[i]
                rej[i] += 1
                self.n_spikes += 1
            else:
                if rej[i] == 0:
                    dn[i] += a * (abs(d) - dn[i])
                rej[i] = 0
                ref[i] = m[i]
        return out

    # ---------------- offline gap filling for invalid rows ----------------
    def _prevalid(self, m):
        """Cheap copy of the front-end's rejection rules (no state shared with it)."""
        if m is None or (m[3] == 0.0 and m[4] == 0.0 and m[5] == 0.0):
            return False
        for i in range(3):
            if abs(m[i]) > GYRO_MAX or abs(m[3 + i]) > ACC_MAX:
                return False
        if m == self.p_last_raw:
            self.p_stale += 1
        else:
            self.p_stale = 0
            self.p_last_raw = m
        return self.p_stale < 3

    def push(self, ts, m, emit):
        """Process one input row and call emit(ts) for every row whose pose is
        final, in input order.  With interp_max > 0, invalid rows (NaN, zero,
        stuck) are held until the next valid sample and then filled by
        interpolation; runs longer than interp_max (e.g. the 5 s failure) are
        dead-reckoned as before."""
        imax = self._interp_max
        if imax <= 0.0:
            self.update(ts, m)
            emit(ts)
            return
        valid = self._prevalid(m)
        if not valid:
            if self.last_good is None or self.long_gap:
                self.update(ts, None)
                emit(ts)
                return
            self.pending.append(ts)
            if (ts - self.last_good_ts) * 1e-9 > imax:      # too long: dead-reckon
                for tp in self.pending:
                    self.update(tp, None)
                    emit(tp)
                self.pending = []
                self.long_gap = True
            return
        self.long_gap = False
        if self.pending:
            mg = self._gate(m, self.dt_nom)
            lg, t0 = self.last_good, self.last_good_ts
            if mg is None or lg is None or ts <= t0:
                for tp in self.pending:
                    self.update(tp, None)
                    emit(tp)
            else:
                span = float(ts - t0)
                for tp in self.pending:
                    f = (tp - t0) / span
                    self.update(tp, [lg[i] + f * (mg[i] - lg[i]) for i in range(6)], synthetic=True)
                    emit(tp)
            self.pending = []
            if mg is not None:
                self.update(ts, mg, synthetic=True)
                self.last_good, self.last_good_ts = list(mg), ts
            else:
                self.update(ts, None)
            emit(ts)
            return
        self.update(ts, m)
        emit(ts)

    def flush_pending(self, emit):
        for tp in self.pending:
            self.update(tp, None)
            emit(tp)
        self.pending = []

    # ---------------- front-end ----------------
    def _gate(self, m, dt):
        if m is None:
            return None
        if m == self.last_raw:
            self.stale += 1
        else:
            self.stale = 0
            self.last_raw = m
        if self.stale >= 3:                 # frozen sensor
            return None
        if (m[3] == 0.0 and m[4] == 0.0 and m[5] == 0.0):
            return None
        for i in range(3):
            if abs(m[i]) > GYRO_MAX or abs(m[3 + i]) > ACC_MAX:
                return None
        ref = self.ref
        if ref is None:
            self.ref = list(m)
            return m
        if self.tb > 0.0:                   # after a dropout the old reference is stale
            self.ref = list(m)
            self.rej = [0] * 6
            return m
        k = self._spike_k
        a = min(1.0, dt / 0.5)
        out = list(m)
        dn = self.dn
        rej = self.rej
        for i in range(6):
            d = m[i] - ref[i]
            floor = 0.5 if i < 3 else 3.0
            # impulsive spike -> hold reference; a persistent level change
            # (3 consecutive rejections) is real and is accepted
            if abs(d) > k * dn[i] + floor and rej[i] < 2:
                out[i] = ref[i]
                rej[i] += 1
                self.n_spikes += 1
            else:
                if rej[i] == 0:
                    dn[i] += a * (abs(d) - dn[i])
                rej[i] = 0
                ref[i] = m[i]
        return out

    # ---------------- strapdown propagation with corrections ----------------
    def _propagate(self, dt, m):
        # --- hoist all state to locals (avoid self. attribute lookups) ---
        bg = self.bg; ba = self.ba; v = self.v; pos = self.pos
        ws = self.ws; vs = self.vs; dn = self.dn; rej = self.rej
        fbar = self.fbar; lc = self.lc; fd = self.fd; mw = self.mw
        g = self.g; g15 = self._g15
        sqrt = _sqrt; sin = _sin; cos = _cos; exp = _exp

        wx, wy, wz = m[0] - bg[0], m[1] - bg[1], m[2] - bg[2]
        fx, fy, fz = m[3], m[4], m[5]

        # --- noise & static detection ---
        a_n = min(1.0, dt / 2.0)
        a_s = min(1.0, dt / 0.2)
        self.nw += a_n * ((dn[0] * dn[0] + dn[1] * dn[1] + dn[2] * dn[2]) - self.nw)
        self.na += a_n * ((dn[3] * dn[3] + dn[4] * dn[4] + dn[5] * dn[5]) - self.na)
        self.t_run += dt
        if self.t_run > 3.0 and self.na < self.na_min:
            self.na_min = self.na
        fn = sqrt(fx * fx + fy * fy + fz * fz)
        self.ew += a_s * ((wx * wx + wy * wy + wz * wz) - self.ew)
        self.ea += a_s * ((fn - g) * (fn - g) - self.ea)
        still = (self.ew < 4.0 * self.nw + 0.0004 and
                 self.ea < 1.5 * self.na + 0.0225)
        self.static_t = self.static_t + dt if still else 0.0
        static = self.static_t >= 1e9

        # --- attitude: gravity-aided complementary filter ---
        # Inline q_to_rot(self.q): compute rotation matrix directly (no tuple overhead)
        qw, qx, qy, qz = self._qw, self._qx, self._qy, self._qz
        xx, yy, zz = qx * qx, qy * qy, qz * qz
        xy, xz, yz = qx * qy, qx * qz, qy * qz
        wxq, wyq, wzq = qw * qx, qw * qy, qw * qz
        r20 = 2.0 * (xz - wyq)
        r21 = 2.0 * (yz + wxq)
        r22 = 1.0 - 2.0 * (xx + yy)
        vx, vy, vz = r20, r21, r22           # estimated "up" in body

        if fn > 1e-6:
            ux, uy, uz = fx / fn, fy / fn, fz / fn
            r = (fn - g) / g15
            trust = exp(-r * r)
            ex = uy * vz - uz * vy
            ey = uz * vx - ux * vz
            ez = ux * vy - uy * vx
            kp = (self._kp_static if static else self._kp) * trust
            ki = self._ki * trust * dt
            wx += kp * ex
            wy += kp * ey
            wz += kp * ez
            bg[0] -= ki * ex
            bg[1] -= ki * ey
            bg[2] -= ki * ez

        if static:                                       # gyro bias at rest
            b = min(1.0, dt / 1.0)
            bg[0] += b * (m[0] - bg[0])
            bg[1] += b * (m[1] - bg[1])
            bg[2] += b * (m[2] - bg[2])

        # --- inline q_integrate(self.q, wx, wy, wz, dt) ---
        ang = sqrt(wx * wx + wy * wy + wz * wz) * dt
        if ang >= 1e-12:
            h = 0.5 * ang
            s = sin(h) / (ang / dt)
            c = cos(h)
            sx, sy, sz = wx * s, wy * s, wz * s
            # q_mul(q, (c, sx, sy, sz)) then q_norm
            nw = qw * c - qx * sx - qy * sy - qz * sz
            nx = qw * sx + qx * c + qy * sz - qz * sy
            ny = qw * sy - qx * sz + qy * c + qz * sx
            nz = qw * sz + qx * sy - qy * sx + qz * c
            qnorm = sqrt(nw * nw + nx * nx + ny * ny + nz * nz)
            qw, qx, qy, qz = nw / qnorm, nx / qnorm, ny / qnorm, nz / qnorm
            self._qw, self._qx, self._qy, self._qz = qw, qx, qy, qz

        # --- smoothed angular rate ---
        b = dt / 0.1
        if b > 1.0:
            b = 1.0
        ws[0] += b * (wx - ws[0])
        ws[1] += b * (wy - ws[1])
        ws[2] += b * (wz - ws[2])

        # --- accelerometer bias / mean tracking (body frame) ---
        # Recompute full rotation matrix from updated quaternion
        xx, yy, zz = qx * qx, qy * qy, qz * qz
        xy, xz, yz = qx * qy, qx * qz, qy * qz
        wxq, wyq, wzq = qw * qx, qw * qy, qw * qz
        r00 = 1.0 - 2.0 * (yy + zz)
        r01 = 2.0 * (xy - wzq)
        r02 = 2.0 * (xz + wyq)
        r10 = 2.0 * (xy + wzq)
        r11 = 1.0 - 2.0 * (xx + zz)
        r12 = 2.0 * (yz - wxq)
        r20 = 2.0 * (xz - wyq)
        r21 = 2.0 * (yz + wxq)
        r22 = 1.0 - 2.0 * (xx + yy)
        vx, vy, vz = r20, r21, r22

        tba = 0.5 if static else self._tau_ba
        b = min(1.0, dt / tba)
        ba[0] += b * (fx - g * vx - ba[0])
        ba[1] += b * (fy - g * vy - ba[1])
        ba[2] += b * (fz - g * vz - ba[2])
        bx, by, bz = fx - ba[0], fy - ba[1], fz - ba[2]
        ax = r00 * bx + r01 * by + r02 * bz
        ay = r10 * bx + r11 * by + r12 * bz
        az = r20 * bx + r21 * by + r22 * bz - g

        mw0, mw1, mw2 = mw
        b = dt * self._inv_tau_aw
        if b > 1.0:
            b = 1.0
        mw0 += b * (ax - mw0)
        mw1 += b * (ay - mw1)
        mw2 += b * (az - mw2)
        mw[0], mw[1], mw[2] = mw0, mw1, mw2
        ax -= mw0
        ay -= mw1
        az -= mw2

        # --- rotor-drag velocity aiding ---
        drag_on = False
        vtx = vty = vtz = 0.0
        if self._drag > 0.0 and not static:
            fb0, fb1, fb2 = fbar
            b = dt * self._inv_tau_n
            if b > 1.0:
                b = 1.0
            fb0 += b * (fx - fb0)
            fb1 += b * (fy - fb1)
            fb2 += b * (fz - fb2)
            fbar[0], fbar[1], fbar[2] = fb0, fb1, fb2
            nn = sqrt(fb0 * fb0 + fb1 * fb1 + fb2 * fb2)
            if nn > 1e-6 and fn > 1e-6:
                nx, ny, nz = fb0 / nn, fb1 / nn, fb2 / nn
                cs = (fx * nx + fy * ny + fz * nz) / fn
                b = dt * self._inv_tau_spread
                if b > 1.0:
                    b = 1.0
                self.spread += b * (2.0 * (1.0 - cs) - self.spread)
                e = self.eref
                ex_ = 1.0 if e == 0 else 0.0
                ey_ = 1.0 if e == 1 else 0.0
                ez_ = 1.0 if e == 2 else 0.0
                c1x = ny * ez_ - nz * ey_
                c1y = nz * ex_ - nx * ez_
                c1z = nx * ey_ - ny * ex_
                cn = sqrt(c1x * c1x + c1y * c1y + c1z * c1z)
                c1x, c1y, c1z = c1x / cn, c1y / cn, c1z / cn
                c2x = ny * c1z - nz * c1y
                c2y = nz * c1x - nx * c1z
                c2z = nx * c1y - ny * c1x
                l1 = fx * c1x + fy * c1y + fz * c1z
                l2 = fx * c2x + fy * c2y + fz * c2z
                lc0, lc1 = lc
                fd0, fd1 = fd
                b = dt * self._inv_tau_c
                if b > 1.0:
                    b = 1.0
                lc0 += b * (l1 - lc0)
                lc1 += b * (l2 - lc1)
                lc[0], lc[1] = lc0, lc1
                b = dt * self._inv_tau_fd
                if b > 1.0:
                    b = 1.0
                fd0 += b * ((l1 - lc0) - fd0)
                fd1 += b * ((l2 - lc1) - fd1)
                fd[0], fd[1] = fd0, fd1
                self.t_drag += dt
                ang = sqrt(self.spread) if self.spread > 0.0 else 0.0
                lo, hi = self._spread_lo, self._spread_hi
                gk = (hi - ang) / (hi - lo)
                gate = 0.0 if gk < 0.0 else (1.0 if gk > 1.0 else gk)
                vib = sqrt(self.na) if self.na > 0.0 else 0.0
                gv = (vib - self._vib_lo) / (self._vib_hi - self._vib_lo)
                gate *= 0.0 if gv < 0.0 else (1.0 if gv > 1.0 else gv)
                if self.t_drag < self._drag_warm:
                    gate = 0.0
                self.gate = gate
                if gate > 0.0:
                    kg = gate * self._drag_gain
                    s1 = kg * fd0
                    s2 = kg * fd1
                    tbx = s1 * c1x + s2 * c2x
                    tby = s1 * c1y + s2 * c2y
                    tbz = s1 * c1z + s2 * c2z
                    vtx = r00 * tbx + r01 * tby + r02 * tbz
                    vty = r10 * tbx + r11 * tby + r12 * tbz
                    vtz = r20 * tbx + r21 * tby + r22 * tbz
                    drag_on = True
                    self.drag_gate_time += dt * gate

        if self.sink is not None:
            self.sink(0, 1 if drag_on else 0, 1 if static else 0, dt, ax, ay, az, vtx, vty, vtz)

        # --- translation ---
        ox, oy, oz = v
        if static:
            dec = exp(-dt / 0.05)
            v[0] = v[0] * dec
            v[1] = v[1] * dec
            v[2] = v[2] * dec
        elif drag_on:
            dec = 1.0 - dt * self._inv_tau_v
            kv = dt * self._inv_tau_v
            v[0] = (v[0] + ax * dt) * dec + vtx * kv
            v[1] = (v[1] + ay * dt) * dec + vty * kv
            v[2] = (v[2] + az * dt) * dec + vtz * kv
        else:
            dec = 1.0 - dt * self._inv_tau_v
            v[0] = (v[0] + ax * dt) * dec
            v[1] = (v[1] + ay * dt) * dec
            v[2] = (v[2] + az * dt) * dec
        lp = 1.0 - dt * self._inv_tau_p
        h = 0.5 * dt
        pos[0] = (pos[0] + h * (ox + v[0])) * lp
        pos[1] = (pos[1] + h * (oy + v[1])) * lp
        pos[2] = (pos[2] + h * (oz + v[2])) * lp
        b = dt / 0.2
        if b > 1.0:
            b = 1.0
        vs[0] += b * (v[0] - vs[0])
        vs[1] += b * (v[1] - vs[1])
        vs[2] += b * (v[2] - vs[2])

    # ---------------- dead reckoning without measurements ----------------
    def _blind(self, dt):
        if self.sink is not None:
            self.sink(1, 0, 0, dt, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        if self.tb == 0.0:
            self.v_hold = self.vs[:]
            self.w_hold = self.ws[:]
        self.tb += dt
        self.blind_time += dt
        dw = exp(-self.tb * self._inv_tau_wb)
        wh = self.w_hold
        # --- inline q_integrate(self.q, wh[0]*dw, wh[1]*dw, wh[2]*dw, dt) ---
        qw, qx, qy, qz = self._qw, self._qx, self._qy, self._qz
        wx, wy, wz = wh[0] * dw, wh[1] * dw, wh[2] * dw
        ang = sqrt(wx * wx + wy * wy + wz * wz) * dt
        if ang >= 1e-12:
            h = 0.5 * ang
            s = sin(h) / (ang / dt)
            c = cos(h)
            sx, sy, sz = wx * s, wy * s, wz * s
            nw = qw * c - qx * sx - qy * sy - qz * sz
            nx = qw * sx + qx * c + qy * sz - qz * sy
            ny = qw * sy - qx * sz + qy * c + qz * sx
            nz = qw * sz + qx * sy - qy * sx + qz * c
            qnorm = sqrt(nw * nw + nx * nx + ny * ny + nz * nz)
            qw, qx, qy, qz = nw / qnorm, nx / qnorm, ny / qnorm, nz / qnorm
            self._qw, self._qx, self._qy, self._qz = qw, qx, qy, qz
        dv = exp(-self.tb * self._inv_tau_vb)
        v, pos, vh = self.v, self.pos, self.v_hold
        for i in range(3):
            nv = vh[i] * dv
            pos[i] += 0.5 * dt * (v[i] + nv)
            v[i] = nv

    def pose(self):
        w, x, y, z = self._qw, self._qx, self._qy, self._qz
        if w < 0.0:
            w, x, y, z = -w, -x, -y, -z
        # noise-adaptive shrinkage: as the measured accelerometer noise rises
        # above the best level seen, trust the integrated position less
        gam = self._shrink_gamma
        if gam > 0.0 and self.na > 0.0 and self.na_min < 1e8:
            ratio = self.na_min / self.na
            b = ratio ** (0.5 * gam)
            if b > 1.0:
                b = 1.0
            self.beta += 0.01 * (b - self.beta)
        pos = self.pos
        bt = self.beta * self._out_gain
        return (pos[0] * bt, pos[1] * bt, pos[2] * bt), (x, y, z, w)


# --------------------------------------------------------------------------
# I/O
# --------------------------------------------------------------------------
def open_imu_csv(path):
    """Return a text stream for imu0/data.csv given a csv, directory or zip.
    Read as utf-8-sig so a byte-order mark never turns the first row into a header."""
    if os.path.isdir(path):
        cand = other = None
        for root, _, files in os.walk(path):
            if os.path.basename(root) == "imu0" and "data.csv" in files:
                cand = os.path.join(root, "data.csv")
                break
            if cand is None and "data.csv" in files:
                cand = os.path.join(root, "data.csv")
            if other is None:
                for f in sorted(files):
                    if f.lower().endswith(".csv"):
                        other = os.path.join(root, f)
                        break
        cand = cand or other
        if cand is None:
            raise FileNotFoundError("no imu0/data.csv under " + path)
        return open(cand, "r", newline="", encoding="utf-8-sig", errors="replace")
    if _is_zip(path):
        import zipfile
        zf = zipfile.ZipFile(path)
        allnames = zf.namelist()
        norm = [n.replace("\\", "/") for n in allnames]          # zips made on Windows
        idx = [i for i, n in enumerate(norm) if n.endswith("imu0/data.csv")]
        if not idx:
            idx = [i for i, n in enumerate(norm) if n.endswith("data.csv")]
        if not idx:
            idx = [i for i, n in enumerate(norm) if n.lower().endswith(".csv")]
        if not idx:
            raise FileNotFoundError("no imu0/data.csv inside " + path)
        return io.TextIOWrapper(zf.open(allnames[idx[0]]), encoding="utf-8-sig", errors="replace",
                                newline="")
    return open(path, "r", newline="", encoding="utf-8-sig", errors="replace")


def _is_zip(path):
    try:
        with open(path, "rb") as f:
            return f.read(4) in (b"PK\x03\x04", b"PK\x05\x06")
    except OSError:
        return False


def find_default_input(folder="."):
    """Input to use when none is given: imu0/data.csv below the folder, else a
    lone data.csv / .zip / .csv in it."""
    for root, _, files in os.walk(folder):
        if os.path.basename(root) == "imu0" and "data.csv" in files:
            return os.path.join(root, "data.csv")
    here = sorted(os.listdir(folder))
    for pick in (lambda f: f == "data.csv", lambda f: f.lower().endswith(".zip"),
                 lambda f: f.lower().endswith(".csv")):
        hits = [f for f in here if pick(f)]
        if len(hits) == 1:
            return os.path.join(folder, hits[0])
    return None


G0 = 9.80665
_isfinite = math.isfinite
SAMPLE_ROWS = 400                    # rows buffered to detect the input format
IO_CHUNK = 512                       # records per read/write batch in the smoother passes
LINE_BUF = 256                       # output lines / forward records buffered before a write


def _split_lines(stream):
    """Yield (kind, payload): ('hdr', text) for header/comment lines, ('row', parts)
    for rows whose first field is numeric."""
    for line in stream:
        s = line.strip()
        if not s:
            continue
        if s[0] == "#":
            yield "hdr", s
            continue
        parts = s.split(",")
        t = parts[0].strip()
        try:
            float(t)
        except ValueError:
            yield "hdr", s                         # header without '#', or garbage
            continue
        yield "row", parts


def _median(v):
    v = sorted(v)
    return v[len(v) // 2] if v else 0.0


class InputFormat:
    """Detected from the data (header names are only cross-checked):
    * timestamp unit from the median sample spacing (s / ms / us / ns, the same
      thresholds as the organizers' degrade_imu.py), converted exactly to ns
    * column order: the accelerometer triplet is the one whose median norm is
      ~1 g; EuRoC is gyro-first, degrade_imu.py documents accel-first
    * accelerometer in g (median norm 0.5-2) is converted to m/s^2
    * gyro in deg/s only when the header says so or --gyro-deg is given"""

    def __init__(self, header_lines, sample, gyro_deg=False):
        ts = []
        for parts in sample:
            try:
                ts.append(float(parts[0]))
            except ValueError:
                pass
        dts = [ts[i + 1] - ts[i] for i in range(len(ts) - 1) if ts[i + 1] > ts[i]]
        dt = _median(dts) if dts else 5e6
        if dt < 1.0:
            self.ts_unit, self.ts_exp = "s", 9
        elif dt < 1e3:
            self.ts_unit, self.ts_exp = "ms", 6
        elif dt < 1e6:
            self.ts_unit, self.ts_exp = "us", 3
        else:
            self.ts_unit, self.ts_exp = "ns", 0
        n1, n2 = [], []
        for parts in sample:
            if len(parts) < 7:
                continue
            try:
                v = [float(x) for x in parts[1:7]]
            except ValueError:
                continue
            if not all(math.isfinite(x) for x in v):
                continue
            a1 = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
            a2 = math.sqrt(v[3] * v[3] + v[4] * v[4] + v[5] * v[5])
            if a1 == 0.0 and a2 == 0.0:
                continue
            n1.append(a1)
            n2.append(a2)
        m1, m2 = _median(n1), _median(n2)
        self.accel_first = m1 > m2                  # default (tie / no data): EuRoC gyro-first
        acc_norm = m1 if self.accel_first else m2
        self.accel_g = 0.5 < acc_norm < 2.0
        self.acc_scale = G0 if self.accel_g else 1.0
        hdr = " ".join(header_lines).lower()
        self.header_order = None
        names = [h.strip().lower() for h in (header_lines[-1].split(",") if header_lines else [])]
        if len(names) >= 7:
            def kind(nm):
                if "w_" in nm or "gyr" in nm or nm.startswith("g") or "omega" in nm:
                    return "g"
                if "a_" in nm or "acc" in nm or nm.startswith("a"):
                    return "a"
                return "?"
            k1, k2 = kind(names[1]), kind(names[4])
            if (k1, k2) == ("g", "a"):
                self.header_order = "gyro_first"
            elif (k1, k2) == ("a", "g"):
                self.header_order = "accel_first"
        self.gyro_deg = gyro_deg or ("deg" in hdr and ("w_" in hdr or "gyr" in hdr))
        self.gyr_scale = math.pi / 180.0 if self.gyro_deg else 1.0
        self.identity = (self.ts_exp == 0 and not self.accel_first and not self.accel_g
                         and not self.gyro_deg)

    def describe(self):
        order = "accel_first" if self.accel_first else "gyro_first"
        return dict(timestamp_unit=self.ts_unit, column_order=order,
                    accel_unit="g" if self.accel_g else "m/s^2",
                    gyro_unit="deg/s" if self.gyro_deg else "rad/s",
                    header_order=self.header_order,
                    header_agrees=None if self.header_order is None else self.header_order == order)

    def ts_ns(self, tok):
        tok = tok.strip()
        if self.ts_exp == 0 and tok.isdigit():
            return int(tok)
        import decimal
        d = decimal.Decimal(tok).scaleb(self.ts_exp)
        return int(d.to_integral_value(rounding=decimal.ROUND_HALF_EVEN))

    def row(self, parts):
        """-> (ts_ns, meas|None) with meas = (wx, wy, wz, ax, ay, az) in rad/s, m/s^2."""
        try:
            ts = self.ts_ns(parts[0])
        except (ValueError, ArithmeticError):        # decimal.InvalidOperation is one
            return None
        meas = None
        if len(parts) >= 7:
            try:
                v = tuple(map(float, parts[1:7]))
                if all(map(_isfinite, v)):
                    meas = v
            except ValueError:
                meas = None
        if meas is not None and not self.identity:
            if self.accel_first:
                meas = meas[3:6] + meas[0:3]
            if self.gyro_deg:
                k = self.gyr_scale
                meas = (meas[0] * k, meas[1] * k, meas[2] * k) + meas[3:6]
            if self.accel_g:
                k = self.acc_scale
                meas = meas[0:3] + (meas[3] * k, meas[4] * k, meas[5] * k)
        return ts, meas


class _TempDir:
    """Private temp folder (system temp, else next to the output); removed by cleanup()."""

    def __init__(self, near):
        bases = [os.environ.get(k) for k in ("TMPDIR", "TEMP", "TMP")]
        bases += ["/tmp", os.path.dirname(os.path.abspath(near))]
        for b in bases:
            if not b or not os.path.isdir(b):
                continue
            name = os.path.join(b, "pathfinder_%d_%d" % (os.getpid(), time.time_ns()))
            try:
                os.mkdir(name)
            except OSError:
                continue
            self.name = name
            return
        raise OSError("no writable temporary folder")

    def cleanup(self):
        name, self.name = getattr(self, "name", None), None
        if not name:
            return
        for f in os.listdir(name):
            try:
                os.remove(os.path.join(name, f))
            except OSError:
                pass
        try:
            os.rmdir(name)
        except OSError:
            pass

    def __del__(self):
        self.cleanup()


def find_rest(inp, gyro_deg, dt_nom_ns, prm, n=200, step=50):
    """Offline pre-pass: the quietest window of n consecutive valid samples
    (lowest mean per-axis gyro std, |f| close to g and steady).  Returns
    (gyro_mean, |f| mean, gyro_std) or None if no window is at rest.
    O(n) memory (a ring of the last n samples)."""
    _, rows = open_rows(open_imu_csv(inp), gyro_deg)
    ring = [None] * n
    cnt = 0
    last_ts = None
    since = 0
    best = None
    gmax = 2.5 * dt_nom_ns
    for ts, m in rows:
        bad = m is None or (m[3] == 0.0 and m[4] == 0.0 and m[5] == 0.0)
        if bad or (last_ts is not None and (ts - last_ts > gmax or ts <= last_ts)):
            cnt = 0                                   # windows never span invalid rows or gaps
            since = 0
            last_ts = None if bad else ts
            if bad:
                continue
        last_ts = ts
        ring[cnt % n] = m
        cnt += 1
        since += 1
        if cnt < n or since < step:
            continue
        since = 0
        sw = [0.0] * 3
        sw2 = [0.0] * 3
        sf = sf2 = 0.0
        for v in ring:
            for i in range(3):
                sw[i] += v[i]
                sw2[i] += v[i] * v[i]
            fn = math.sqrt(v[3] * v[3] + v[4] * v[4] + v[5] * v[5])
            sf += fn
            sf2 += fn * fn
        mean = [x / n for x in sw]
        sd = [math.sqrt(max(sw2[i] / n - mean[i] * mean[i], 0.0)) for i in range(3)]
        std = sum(sd) / 3.0
        fm = sf / n
        fs = math.sqrt(max(sf2 / n - fm * fm, 0.0))
        if abs(fm - 9.81) > 0.5 or fs > 0.3:
            continue
        # at rest the gyro is white noise around the bias: the means of 4 sub-windows
        # agree to ~std/sqrt(n/4).  Hand tremor or slow rotation makes them wander
        # (TUM-VI handheld: same std as a noisy rest window, but a wrong "bias").
        q = n // 4
        ratio = 0.0
        for i in range(3):
            subs = []
            for k in range(4):
                s_ = 0.0
                for j in range(k * q, (k + 1) * q):
                    s_ += ring[(cnt + j) % n][i]          # oldest-to-newest order
                subs.append(s_ / q)
            mu = sum(subs) / 4.0
            spread_ = math.sqrt(sum((x - mu) ** 2 for x in subs) / 3.0)
            ratio += spread_ / (sd[i] / math.sqrt(q) + 1e-9) / 3.0
        if ratio > prm["rest_ratio"]:
            continue
        # ... and the gravity direction does not move: sub-window mean specific
        # forces must agree in direction (a hand-held device tilts by tenths of a
        # degree per second; sensor noise averages to ~0.01 deg over 50 samples)
        dirs = []
        for k in range(4):
            a = [0.0, 0.0, 0.0]
            for j in range(k * q, (k + 1) * q):
                v = ring[(cnt + j) % n]
                a[0] += v[3]
                a[1] += v[4]
                a[2] += v[5]
            na_ = math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])
            dirs.append((a[0] / na_, a[1] / na_, a[2] / na_))
        cmin = min(sum(dirs[k1][i] * dirs[k2][i] for i in range(3))
                   for k1 in range(4) for k2 in range(k1 + 1, 4))
        if math.degrees(math.acos(max(-1.0, min(1.0, cmin)))) > prm["rest_tilt_deg"]:
            continue
        if best is None or std < best[2]:
            best = (mean, fm, std)
    if best is None or best[2] > prm["rest_gyro_std"]:
        return None
    return best


def open_rows(stream, gyro_deg=False):
    """Detect the input format from the first SAMPLE_ROWS rows, then yield
    (ts_ns, meas|None) for every data row (O(1) memory)."""
    it = _split_lines(stream)
    header, sample = [], []
    for kind, payload in it:
        if kind == "hdr":
            if not sample:
                header.append(payload)
            continue
        sample.append(payload)
        if len(sample) >= SAMPLE_ROWS:
            break
    fmt = InputFormat(header, sample, gyro_deg)

    def gen():
        sample.reverse()                    # pop() from the end = original order,
        while sample:                       # and the detection sample is freed as it goes
            r = fmt.row(sample.pop())
            if r is not None:
                yield r
        for kind, payload in it:
            if kind == "row":
                r = fmt.row(payload)
                if r is not None:
                    yield r
    return fmt, gen()


def fmt_ts(ts):
    sign = "-" if ts < 0 else ""
    ts = abs(ts)
    return "%s%d.%09d" % (sign, ts // 1000000000, ts % 1000000000)


# --------------------------------------------------------------------------
# Offline two-pass smoother
# --------------------------------------------------------------------------
# The forward pass streams one fixed-size record per filter step and per output
# row into a temp file.  Three further streaming passes over that file (two of
# them reading it backwards) compute forward and backward leaky velocity and
# position integrations and average them: a zero-phase version of the same
# filter, so real motion is no longer delayed and smeared by the causal leaks.
# Memory stays O(1); the temp file is ~107 bytes per IMU sample.
REC = struct.Struct("<bbbq12d")      # kind, drag, static, ts, 12 doubles
V3 = struct.Struct("<3d")
V6 = struct.Struct("<6d")
STEP, BLIND, ROW = 0, 1, 2


def _read_fwd(f, st, chunk=IO_CHUNK):
    f.seek(0)
    n = st.size
    while True:
        data = f.read(n * chunk)
        if not data:
            return
        for i in range(0, len(data) - n + 1, n):
            yield st.unpack_from(data, i)


def _read_bwd(f, st, chunk=IO_CHUNK):
    f.seek(0, 2)
    end = f.tell()
    n = st.size
    end -= end % n
    while end > 0:
        start = max(0, end - n * chunk)
        f.seek(start)
        data = f.read(end - start)
        for i in range(len(data) - n, -1, -n):
            yield st.unpack_from(data, i)
        end = start


def _vstep(v, r, sgn, prm):
    """One step of the smoother's leaky velocity integration (sgn=-1: backward)."""
    dt = r[4]
    if r[2]:                                   # static: zero-velocity
        d = math.exp(-dt / 0.05)
        return [v[0] * d, v[1] * d, v[2] * d]
    if r[0] == BLIND:                          # no measurement: decay
        d = math.exp(-dt / prm["tau_vbs"])
        return [v[0] * d, v[1] * d, v[2] * d]
    dec = 1.0 - dt / prm["tau_vs"]
    if r[1]:                                   # drag aiding: leak toward drag velocity
        kv = dt / prm["tau_vs"]
        return [(v[0] + sgn * r[5] * dt) * dec + r[8] * kv,
                (v[1] + sgn * r[6] * dt) * dec + r[9] * kv,
                (v[2] + sgn * r[7] * dt) * dec + r[10] * kv]
    return [(v[0] + sgn * r[5] * dt) * dec,
            (v[1] + sgn * r[6] * dt) * dec,
            (v[2] + sgn * r[7] * dt) * dec]


def _smooth(t1_path, tmpdir, out, prm, k_last):
    """Passes B1, B2, C.  k_last = index of the last step with a real measurement;
    after it (the post-failure blind tail) the forward dead reckoning is kept,
    anchored to the smoothed position at failure."""
    tps = prm["tau_ps"]
    gain = prm["out_gain"]
    t2_path = os.path.join(tmpdir, "vf.bin")
    t3_path = os.path.join(tmpdir, "vb.bin")
    with open(t1_path, "rb") as f1:
        # B1: forward velocity
        with open(t2_path, "wb") as f2:
            v = [0.0, 0.0, 0.0]
            buf = []
            for r in _read_fwd(f1, REC):
                if r[0] != ROW:
                    v = _vstep(v, r, 1.0, prm)
                    buf.append(V3.pack(v[0], v[1], v[2]))
                    if len(buf) >= IO_CHUNK:
                        f2.write(b"".join(buf))
                        buf.clear()
            f2.write(b"".join(buf))
        # B2: backward velocity, averaged velocity, backward position
        with open(t2_path, "rb") as f2, open(t3_path, "wb") as f3:
            itf = _read_bwd(f2, V3)
            vb = [0.0, 0.0, 0.0]
            pb = [0.0, 0.0, 0.0]
            buf = []
            for r in _read_bwd(f1, REC):
                if r[0] == ROW:
                    continue
                vf = next(itf)
                vb = _vstep(vb, r, -1.0, prm)
                dt = r[4]
                lp = 1.0 - dt / tps
                vx = 0.5 * (vf[0] + vb[0])
                vy = 0.5 * (vf[1] + vb[1])
                vz = 0.5 * (vf[2] + vb[2])
                pb = [(pb[0] - vx * dt) * lp, (pb[1] - vy * dt) * lp, (pb[2] - vz * dt) * lp]
                buf.append(V6.pack(vx, vy, vz, pb[0], pb[1], pb[2]))
                if len(buf) >= IO_CHUNK:
                    f3.write(b"".join(buf))
                    buf.clear()
            f3.write(b"".join(buf))
        # C: forward position, average, write poses
        with open(t3_path, "rb") as f3:
            # Record k of vb.bin holds the backward position at the START of step k,
            # i.e. at the end of step k-1.  Read one record ahead so the forward
            # position at the end of step k is averaged with the backward position
            # at that same instant, and rows before the first step (pf = 0) get the
            # smoothed start position instead of the origin.
            itb = _read_bwd(f3, V6)
            nxt = next(itb, None)
            pf = [0.0, 0.0, 0.0]
            ps = [0.5 * nxt[3], 0.5 * nxt[4], 0.5 * nxt[5]] if nxt else [0.0, 0.0, 0.0]
            ps_fail = [0.0, 0.0, 0.0]
            pos_ref = [0.0, 0.0, 0.0]
            count = 0
            lines = []
            for r in _read_fwd(f1, REC):
                if r[0] != ROW:
                    vx, vy, vz = nxt[0], nxt[1], nxt[2]
                    nxt = next(itb, None)
                    bx, by, bz = (nxt[3], nxt[4], nxt[5]) if nxt else (0.0, 0.0, 0.0)
                    dt = r[4]
                    lp = 1.0 - dt / tps
                    pf = [(pf[0] + vx * dt) * lp, (pf[1] + vy * dt) * lp, (pf[2] + vz * dt) * lp]
                    ps = [0.5 * (pf[0] + bx), 0.5 * (pf[1] + by), 0.5 * (pf[2] + bz)]
                    if count == k_last:
                        ps_fail = ps
                    count += 1
                    continue
                beta = r[8]
                if count - 1 <= k_last:            # smoothed part
                    px, py, pz = ps
                    pos_ref = [r[9], r[10], r[11]]
                else:                              # post-failure tail: forward dead reckoning
                    px = ps_fail[0] + r[9] - pos_ref[0]
                    py = ps_fail[1] + r[10] - pos_ref[1]
                    pz = ps_fail[2] + r[11] - pos_ref[2]
                lines.append("%s %.6f %.6f %.6f %.9f %.9f %.9f %.9f\n" % (
                    fmt_ts(r[3]), px * beta * gain, py * beta * gain, pz * beta * gain, r[4], r[5], r[6], r[7]))
                if len(lines) >= LINE_BUF:
                    out.write("".join(lines))
                    lines.clear()
            out.write("".join(lines))


def run(inp, out_path, prm, extrapolate=False, gyro_deg=False):
    """extrapolate=False (default, as the Track 3 FAQ requires): exactly one pose
    per input row; the blind segment's rows are present with invalid values and
    are dead-reckoned in place."""
    t0 = time.perf_counter()
    est = PathFinder(prm)
    stream = open_imu_csv(inp)
    fmt, rows = open_rows(stream, gyro_deg)

    # ---- buffer the first init_s seconds for alignment ----
    head = []
    t_first = None
    for ts, m in rows:
        if t_first is None:
            t_first = ts
        head.append((ts, m))
        if ts - t_first >= prm["init_s"] * 1e9 and len(head) >= 20:
            break
    if not head:
        raise SystemExit("no IMU rows found in " + inp)
    diffs = sorted(head[i + 1][0] - head[i][0] for i in range(len(head) - 1)
                   if head[i + 1][0] > head[i][0])
    dt_nom_ns = diffs[len(diffs) // 2] if diffs else 5000000
    est.initialise([m for _, m in head if m is not None], dt_nom_ns * 1e-9)
    rest = None
    if prm["rest_search"] > 0.0 and not getattr(est, "init_static", False):
        rest = find_rest(inp, gyro_deg, dt_nom_ns, prm)
        if rest is not None:                          # gyro bias (and local g) from a rest window
            est.bg = list(rest[0])
            est.g = rest[1]

    out = open(out_path, "w", newline="\n")
    out.write("# timestamp tx ty tz qx qy qz qw\n")
    buf = []
    n_out = 0
    last_ts = None
    trailing_invalid_ns = 0
    run_start = None
    smooth = prm["smooth"] > 0.0
    tmpdir = t1 = None
    if smooth:
        try:
            tmpdir = _TempDir(out_path)
            t1 = open(os.path.join(tmpdir.name, "steps.bin"), "wb")
        except OSError as ex:          # no usable temp space: causal single pass
            sys.stderr.write("[pathfinder] warning: %s; using causal (single-pass) mode\n" % ex)
            smooth = False
    nsteps = [0, -1]                      # [steps so far, index of last measured step]

    if smooth:
        def sink(kind, drag, static, dt, ax, ay, az, vx, vy, vz):
            if kind == STEP:
                nsteps[1] = nsteps[0]
            nsteps[0] += 1
            # doubles: r[4]=dt, r[5:8]=world accel, r[8:11]=drag target velocity
            buf.append(REC.pack(kind, drag, static, 0, dt, ax, ay, az, vx, vy, vz,
                                0.0, 0.0, 0.0, 0.0, 0.0))
        est.sink = sink

    def emit(ts):
        pos, q = est.pose()
        if smooth:
            p = est.pos
            # doubles: r[4:8]=quaternion (x,y,z,w), r[8]=beta, r[9:12]=raw forward position
            buf.append(REC.pack(ROW, 0, 0, ts, q[0], q[1], q[2], q[3], est.beta,
                                p[0], p[1], p[2], 0.0, 0.0, 0.0, 0.0))
        else:
            buf.append("%s %.6f %.6f %.6f %.9f %.9f %.9f %.9f\n" % (
                fmt_ts(ts), pos[0], pos[1], pos[2], q[0], q[1], q[2], q[3]))

    def flush():
        if smooth:
            t1.write(b"".join(buf))
        else:
            out.write("".join(buf))
        buf.clear()

    def emit_row(ts):
        nonlocal n_out
        emit(ts)
        n_out += 1
        if len(buf) >= LINE_BUF:
            flush()

    def consume(ts, m):
        nonlocal last_ts, trailing_invalid_ns, run_start
        est.push(ts, m, emit_row)
        if m is None or (m[3] == 0.0 and m[4] == 0.0 and m[5] == 0.0):   # invalid (NaN / zero rows)
            if run_start is None:
                run_start = last_ts if last_ts is not None else ts
            trailing_invalid_ns = ts - run_start
        else:
            run_start = None
            trailing_invalid_ns = 0
        last_ts = ts

    for ts, m in head:
        consume(ts, m)
    head = None
    for ts, m in rows:
        consume(ts, m)
    est.flush_pending(emit_row)

    # ---- post-failure dead reckoning beyond the last frame ----
    n_extra = 0
    if extrapolate and last_ts is not None and \
            trailing_invalid_ns < 0.8 * prm["horizon_s"] * 1e9:
        n_extra = int(round(prm["horizon_s"] * 1e9 / dt_nom_ns))
        for k in range(1, n_extra + 1):
            ts = last_ts + k * dt_nom_ns
            est.update(ts, None)
            emit(ts)
            if len(buf) >= LINE_BUF:
                flush()
    flush()
    if smooth:
        t1.close()
        _smooth(os.path.join(tmpdir.name, "steps.bin"), tmpdir.name, out, prm, nsteps[1])
        tmpdir.cleanup()
    out.close()
    stream.close()
    elapsed = time.perf_counter() - t0
    span = (last_ts - t_first) * 1e-9 if last_ts is not None else 0.0
    return dict(
        input=os.path.basename(os.path.normpath(inp)),      # names only: this file is submitted
        output=os.path.basename(out_path),
        imu_rows=n_out,
        extrapolated_rows=n_extra,
        nominal_rate_hz=round(1e9 / dt_nom_ns, 3),
        imu_span_s=round(span, 3),
        static_initialisation=bool(getattr(est, "init_static", False)),
        invalid_rows=est.n_invalid,
        timestamp_gaps=est.n_gaps,
        spikes_rejected=est.n_spikes,
        blind_time_s=round(est.blind_time, 3),
        drag_active_s=round(est.drag_gate_time, 3),
        drag_final_gate=round(est.gate, 3),
        drag_enabled=bool(prm["drag"] > 0.0),
        drag_gain=prm["drag_gain"],
        rotor_drag_model={
            "equation": "v_drag = drag_gain * f_lateral  (m/s per m/s^2)",
            "description": "Rotor-drag velocity aiding for multirotors. Thrust acts along a body-fixed axis; the accelerometer component perpendicular to thrust axis is rotor drag ~ -k * v_body, giving a drift-free velocity measurement from IMU alone.",
            "detector": "Two-gated: (1) specific-force spread vs thrust axis (spread_lo=0.14 rad, spread_hi=0.26 rad); (2) rotor vibration accelerometer energy (vib_lo=0.8, vib_hi=1.2 m/s^2). Both gates must open for drag aiding to engage.",
            "thrust_axis_tracking": "tau_n=20.0 s slow mean of specific-force direction",
            "lateral_force_tracking": "tau_c=30.0 s offset tracking, tau_fd=0.6 s low-pass",
            "drag_warm_s": prm["drag_warm"],
        },
        smoothing="two-pass zero-phase" if smooth else "causal (single pass)",
        input_format=fmt.describe(),
        gyro_bias_source=("initial static segment" if getattr(est, "init_static", False) else
                          ("rest window (gyro std %.4f rad/s)" % rest[2] if rest is not None else "none found")),
        runtime_s=round(elapsed, 4),
        runtime_ms_per_imu_second=round(1000.0 * elapsed / span, 3) if span > 0 else None,
        threads=1,
    )


def estimate(input_path, output_path="estimated_trajectory.txt", **overrides):
    """Importable entry point: degraded IMU file/folder/zip -> TUM trajectory file.
    Returns the run statistics.  Falls back to the causal single pass if the
    offline smoother fails for any reason, so a trajectory is always written."""
    prm = dict(DEFAULTS)
    prm.update(overrides)
    return _run_safe(input_path, output_path, prm, False, False)


def _run_safe(inp, out_path, prm, extrapolate, gyro_deg):
    try:
        return run(inp, out_path, prm, extrapolate=extrapolate, gyro_deg=gyro_deg)
    except (FileNotFoundError, SystemExit):
        raise
    except Exception as ex:                       # never leave the evaluator without a file
        if prm["smooth"] <= 0.0 and prm["rest_search"] <= 0.0:
            raise
        sys.stderr.write("[pathfinder] warning: %s: %s; retrying as a causal single pass\n"
                         % (type(ex).__name__, ex))
        safe = dict(prm, smooth=0.0, rest_search=0.0)
        info = run(inp, out_path, safe, extrapolate=extrapolate, gyro_deg=gyro_deg)
        info["fallback"] = "%s: %s" % (type(ex).__name__, ex)
        return info


USAGE = """usage: main.py [IN] [OUT] [-i IN] [-o OUT] [--metadata FILE] [--gyro-deg] [--no-smooth]
               [--extrapolate] [--profile-memory] [--set KEY=VAL ...]

Path Finder: degraded-IMU 6-DoF trajectory estimator (XR Hackathon 2026, Track 03).
IN is imu0/data.csv, a EuRoC-style sequence folder or a .zip; OUT defaults to
estimated_trajectory.txt.  With no IN, imu0/data.csv is searched below the
current folder.  Unknown options are ignored with a warning.

  -i, --input IN       input (same as the positional IN)
  -o, --output OUT     output trajectory (same as the positional OUT)
  --metadata FILE      write run statistics as JSON
  --gyro-deg           gyro columns are in deg/s
  --no-smooth          single-pass causal output (no offline smoother)
  --extrapolate        append 5 s of poses after the last row (off: the FAQ
                       requires exactly one pose per input row)
  --profile-memory     report the Python heap peak (tracemalloc; slower)
  --set KEY=VAL ...    override tuning parameters (see DEFAULTS)
"""

_FLAGS = {"--extrapolate", "--no-extrapolate", "--gyro-deg", "--no-smooth", "--profile-memory"}
_VALUED = {"-i": "input", "--input": "input", "-o": "output", "--output": "output",
           "--metadata": "metadata"}


def parse_args(argv):
    """Returns (options dict, positionals, unknown).  Accepts '--opt value' and
    '--opt=value'; an unknown '--opt value' pair is skipped as a whole when the
    value does not look like a path."""
    opt = {"set": []}
    pos, unknown = [], []
    i = 0
    while i < len(argv):
        a = argv[i]
        if a in ("-h", "--help"):
            sys.stdout.write(USAGE)
            raise SystemExit(0)
        key, val = (a.split("=", 1) + [None])[:2] if a.startswith("--") and "=" in a else (a, None)
        if key in _VALUED:
            if val is None:
                i += 1
                if i >= len(argv):
                    raise SystemExit("option %s needs a value" % key)
                val = argv[i]
            opt[_VALUED[key]] = val
        elif key in _FLAGS:
            opt[key[2:].replace("-", "_")] = True
        elif key == "--set":
            while i + 1 < len(argv) and "=" in argv[i + 1] and not argv[i + 1].startswith("-"):
                i += 1
                opt["set"].append(argv[i])
        elif a.startswith("-") and len(a) > 1:
            unknown.append(a)
            nxt = argv[i + 1] if i + 1 < len(argv) else None
            if val is None and nxt is not None and not nxt.startswith("-") and \
                    not any(c in nxt for c in "/\\.") and len(pos) >= 1:
                unknown.append(nxt)            # its value, e.g. "--threads 1"
                i += 1
        else:
            pos.append(a)
        i += 1
    return opt, pos, unknown


def _compute_info():
    import platform
    return dict(python=platform.python_version(), implementation=platform.python_implementation(),
                system=platform.system(), machine=platform.machine(), cpu_count=os.cpu_count())


def main(argv=None):
    opt, pos, unknown = parse_args(sys.argv[1:] if argv is None else list(argv))
    if unknown:                                   # an evaluation harness may pass its own flags
        sys.stderr.write("[pathfinder] warning: ignoring unrecognised arguments: %s\n" % " ".join(unknown))
    inp = opt.get("input") or (pos[0] if pos else None)
    if inp is None:
        inp = find_default_input(".")
        if inp is None:
            sys.stderr.write(USAGE)
            raise SystemExit("error: no input given and no imu0/data.csv found below the current folder")
        sys.stderr.write("[pathfinder] no input given; using %s\n" % inp)
    out_path = opt.get("output") or (pos[1] if len(pos) > 1 else None) or "estimated_trajectory.txt"
    if os.path.isdir(out_path):
        out_path = os.path.join(out_path, "estimated_trajectory.txt")
    parent = os.path.dirname(os.path.abspath(out_path))
    if not os.path.isdir(parent):
        os.makedirs(parent, exist_ok=True)
    prm = dict(DEFAULTS)
    for kv in opt["set"]:
        k, v = kv.split("=", 1)
        if k not in prm:
            raise SystemExit("unknown parameter " + k)
        prm[k] = float(v)
    if opt.get("no_smooth"):
        prm["smooth"] = 0.0
    if opt.get("profile_memory"):
        import tracemalloc
        tracemalloc.start()
    info = _run_safe(inp, out_path, prm, bool(opt.get("extrapolate")) and not opt.get("no_extrapolate"),
                     bool(opt.get("gyro_deg")))
    if opt.get("profile_memory"):
        info["python_peak_heap_bytes"] = tracemalloc.get_traced_memory()[1]
        tracemalloc.stop()
    info["parameters"] = prm
    sys.stderr.write(
        "[pathfinder] %d poses (+%d extrapolated) -> %s | %.1f ms per IMU-second | "
        "invalid=%d gaps=%d spikes=%d\n" % (
            info["imu_rows"], info["extrapolated_rows"], out_path,
            info["runtime_ms_per_imu_second"] or 0.0,
            info["invalid_rows"], info["timestamp_gaps"], info["spikes_rejected"]))
    if opt.get("metadata"):
        import json
        info["compute"] = _compute_info()
        mparent = os.path.dirname(os.path.abspath(opt["metadata"]))
        if not os.path.isdir(mparent):
            os.makedirs(mparent, exist_ok=True)
        with open(opt["metadata"], "w") as f:
            json.dump(info, f, indent=2)
    return 0


if __name__ == "__main__":
    sys.exit(main())
