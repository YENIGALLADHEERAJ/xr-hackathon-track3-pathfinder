#!/usr/bin/env python3
"""Output-contract tests for src/main.py (stdlib only).  python test_contract.py

Covers the Track 3 rules (slides 15-17 and the official FAQ): exactly one pose
per input row, no appended poses, exact ns -> s timestamps, blind segment given
as invalid rows, dropouts as missing rows, plus the input variations the
organizers' degrade_imu.py can produce (accel-first columns, s/ms timestamps,
accel in g, no header, duplicated and stuck rows)."""
import math
import os
import subprocess
import sys
import tempfile
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
MAIN = os.path.join(HERE, "..", "src", "main.py")
CMD = [sys.executable, MAIN]
HDR = ("#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],"
       "a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]")
HDR_AF = ("#timestamp [ns],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2],"
          "w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1]")
T0 = 1403636579758555392
DT = 5000000


def sample(k):
    s = 0.3 * math.sin(k * 0.01)
    return (0.01 * s, -0.02 * s, 0.05 * s), (0.2 * s, 0.1, 9.81 + 0.05 * s)


def rows(n, bad=None, zero=None, order="gw", ts_fmt=None, acc_scale=1.0):
    out = []
    for k in range(n):
        ts = T0 + k * DT
        tstr = ts_fmt(ts) if ts_fmt else "%d" % ts
        if bad and bad(k):
            out.append(tstr + ",nan,nan,nan,nan,nan,nan")
            continue
        w, a = sample(k)
        if zero and zero(k):
            w, a = (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
        a = tuple(x * acc_scale for x in a)
        v = w + a if order == "gw" else a + w
        out.append(tstr + "," + ",".join("%.6f" % x for x in v))
    return out


def run(lines, *args, header=HDR, eol="\n", as_zip=False, deflate=False, meta=False):
    d = tempfile.mkdtemp()
    csv = os.path.join(d, "mav0", "imu0", "data.csv")
    os.makedirs(os.path.dirname(csv))
    with open(csv, "w", newline="") as f:
        f.write(eol.join(([header] if header else []) + lines) + eol)
    inp = d
    if as_zip:
        inp = os.path.join(d, "eval.zip")
        with zipfile.ZipFile(inp, "w", zipfile.ZIP_DEFLATED if deflate else zipfile.ZIP_STORED) as z:
            z.write(csv, "eval/mav0/imu0/data.csv")
    out = os.path.join(d, "est.txt")
    extra = ["--metadata", os.path.join(d, "m.json")] if meta else []
    subprocess.run(CMD + [inp, "-o", out] + list(args) + extra, check=True, capture_output=True)
    est = [l.split() for l in open(out) if not l.startswith("#")]
    if meta:
        import json
        return est, json.load(open(os.path.join(d, "m.json")))
    return est


def check(name, cond):
    print("%-62s %s" % (name, "PASS" if cond else "FAIL"))
    return cond


def pos(e):
    return [tuple(float(x) for x in r[1:8]) for r in e]


def main():
    ok = True
    n = 2000
    base = rows(n)
    e = run(base)
    ok &= check("exactly one pose per input row (no appended poses)", len(e) == n)
    ok &= check("8 columns per row", all(len(r) == 8 for r in e))
    ok &= check("exact ns -> s timestamps", e[0][0] == "1403636579.758555392" and
                e[1][0] == "1403636579.763555392")
    ok &= check("unit quaternions", all(abs(sum(float(x) ** 2 for x in r[4:]) - 1) < 1e-6 for r in e))
    ok &= check("timestamps in input order",
                all(r[0] == "%d.%09d" % divmod(T0 + k * DT, 10 ** 9) for k, r in enumerate(e)))
    e = run(rows(n + 1000, zero=lambda k: k >= n))
    ok &= check("blind 5 s as all-zero rows (degrade_imu.py): one pose each", len(e) == n + 1000)
    ok &= check("blind segment is dead-reckoned (finite, moving on)",
                all(math.isfinite(float(x)) for r in e[n:] for x in r))
    e = run(rows(n + 1000, bad=lambda k: k >= n))
    ok &= check("blind 5 s as NaN rows: one pose each", len(e) == n + 1000)
    e = run(base, "--extrapolate")
    ok &= check("--extrapolate (opt-in) appends 5 s at the nominal rate",
                len(e) == n + 1000 and e[n][0] == "%d.%09d" % divmod(T0 + n * DT, 10 ** 9))
    e_ref = run(base)
    e_af, m_af = run(rows(n, order="wg"), header=HDR_AF, meta=True)
    ok &= check("accel-first columns detected and handled identically",
                m_af["input_format"]["column_order"] == "accel_first" and pos(e_af) == pos(e_ref))
    e_nh, m_nh = run(rows(n, order="wg"), header=None, meta=True)
    ok &= check("no header, accel-first: detected from the data",
                m_nh["input_format"]["column_order"] == "accel_first" and pos(e_nh) == pos(e_ref))
    e_s, m_s = run(rows(n, ts_fmt=lambda t: "%d.%09d" % divmod(t, 10 ** 9)), meta=True)
    ok &= check("timestamps in seconds: unit detected, values exact",
                m_s["input_format"]["timestamp_unit"] == "s" and [r[0] for r in e_s] == [r[0] for r in e_ref]
                and pos(e_s) == pos(e_ref))
    e_ms, m_ms = run(rows(n, ts_fmt=lambda t: "%d.%06d" % divmod(t, 10 ** 6)), meta=True)
    ok &= check("timestamps in milliseconds: unit detected, values exact",
                m_ms["input_format"]["timestamp_unit"] == "ms" and [r[0] for r in e_ms] == [r[0] for r in e_ref])
    e_g, m_g = run(rows(n, acc_scale=1 / 9.80665), meta=True)
    ok &= check("accelerometer in g: detected and converted",
                m_g["input_format"]["accel_unit"] == "g" and len(e_g) == n and
                max(abs(a - b) for p, q in zip(pos(e_g), pos(e_ref)) for a, b in zip(p, q)) < 1e-3)
    dup = base[:300] + [base[300]] + base[300:]
    e = run(dup)
    ok &= check("duplicated row (same timestamp) keeps its own pose", len(e) == n + 1 and e[300] == e[301])
    stuck = base[:]
    for k in range(501, 509):
        stuck[k] = "%d" % (T0 + k * DT) + "," + base[500].split(",", 1)[1]
    e = run(stuck)
    ok &= check("stuck (frozen) samples keep a pose each", len(e) == n)
    e = run(base, as_zip=True)
    ok &= check("zip input (stored)", len(e) == n)
    big = rows(40000)
    ez = run(big, as_zip=True, deflate=True)
    ep = run(big)
    ok &= check("zip input (deflate) identical to plain csv", ez == ep and len(ez) == 40000)
    e = run(base, eol="\r\n")
    ok &= check("CRLF input", len(e) == n)
    bad = base[:]
    bad[100] = "%d,,,,,," % (T0 + 100 * DT)
    bad[200] = "%d,0,0,0,0,0,0" % (T0 + 200 * DT)
    bad[300] = "%d,1e9,0,0,0,0,9.8" % (T0 + 300 * DT)
    bad[400] = "%d,0.1,0.2" % (T0 + 400 * DT)
    bad[500] = "garbage line"
    e = run(bad)
    ok &= check("empty/zero/saturated/short rows keep a pose; garbage skipped", len(e) == n - 1)
    ok &= check("no NaN/inf in output", all(math.isfinite(float(x)) for r in e for x in r))
    gap = base[:600] + base[1000:]
    e = run(gap)
    ok &= check("2 s dropout (missing rows): one pose per remaining row", len(e) == n - 400)
    ooo = base[:]
    ooo[700], ooo[701] = ooo[701], ooo[700]
    e = run(ooo)
    ok &= check("out-of-order row keeps a pose", len(e) == n)
    e = run(base, "--no-smooth")
    ok &= check("--no-smooth (causal) also one pose per row", len(e) == n)
    print("ALL PASS" if ok else "SOME TESTS FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
