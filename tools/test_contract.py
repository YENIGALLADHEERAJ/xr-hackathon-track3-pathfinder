#!/usr/bin/env python3
"""Output-contract tests (stdlib only).

    python test_contract.py          # tests src/main.py
    python test_contract.py --cpp    # tests the C++ build src/pathfinder[.exe]
"""
import math
import os
import subprocess
import sys
import tempfile
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
MAIN = os.path.join(HERE, "..", "src", "main.py")
EXE = os.path.join(HERE, "..", "src", "pathfinder.exe" if os.name == "nt" else "pathfinder")
CMD = [EXE] if "--cpp" in sys.argv else [sys.executable, MAIN]
HDR = ("#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],"
       "a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]")
T0 = 1403636579758555392
DT = 5000000


def rows(n, start=0, bad=None):
    out = []
    for k in range(start, start + n):
        if bad and bad(k):
            out.append("%d,nan,nan,nan,nan,nan,nan" % (T0 + k * DT))
        else:
            s = 0.3 * math.sin(k * 0.01)
            out.append("%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f" % (T0 + k * DT, 0.01 * s, -0.02 * s, 0.05 * s,
                                                            0.2 * s, 0.1, 9.81 + 0.05 * s))
    return out


def run(lines, *args, eol="\n", as_zip=False, deflate=False):
    d = tempfile.mkdtemp()
    csv = os.path.join(d, "mav0", "imu0", "data.csv")
    os.makedirs(os.path.dirname(csv))
    with open(csv, "w", newline="") as f:
        f.write(eol.join([HDR] + lines) + eol)
    inp = d
    if as_zip:
        inp = os.path.join(d, "eval.zip")
        with zipfile.ZipFile(inp, "w", zipfile.ZIP_DEFLATED if deflate else zipfile.ZIP_STORED) as z:
            z.write(csv, "eval/mav0/imu0/data.csv")
    out = os.path.join(d, "est.txt")
    subprocess.run(CMD + [inp, "-o", out] + list(args), check=True, capture_output=True)
    est = [l.split() for l in open(out) if not l.startswith("#")]
    return est


def check(name, cond):
    print("%-58s %s" % (name, "PASS" if cond else "FAIL"))
    return cond


def main():
    ok = True
    print("implementation:", " ".join(os.path.basename(c) for c in CMD))
    n = 2000
    base = rows(n)
    e = run(base)
    ok &= check("one pose per row + 5 s extrapolated (1000 rows @200 Hz)", len(e) == n + 1000)
    ok &= check("8 columns per row", all(len(r) == 8 for r in e))
    ok &= check("exact ns -> s timestamps", e[0][0] == "1403636579.758555392" and
                e[1][0] == "1403636579.763555392")
    ok &= check("extrapolated stamps continue at nominal rate",
                e[n][0] == "%d.%09d" % divmod(T0 + n * DT, 10 ** 9))
    ok &= check("unit quaternions", all(abs(sum(float(x) ** 2 for x in r[4:]) - 1) < 1e-6 for r in e))
    e = run(base, "--no-extrapolate")
    ok &= check("--no-extrapolate: exactly one pose per row", len(e) == n)
    e = run(rows(n + 1000, bad=lambda k: k >= n))
    ok &= check("trailing 5 s NaN rows: dead-reckoned in place, no append", len(e) == n + 1000)
    e = run(base, as_zip=True)
    ok &= check("zip input (stored)", len(e) == n + 1000)
    big = rows(40000)
    ez = run(big, as_zip=True, deflate=True)
    ep = run(big)
    ok &= check("zip input (deflate) identical to plain csv", ez == ep and len(ez) == 41000)
    e = run(base, eol="\r\n")
    ok &= check("CRLF input", len(e) == n + 1000)
    bad = base[:]
    bad[100] = "%d,,,,,," % (T0 + 100 * DT)
    bad[200] = "%d,0,0,0,0,0,0" % (T0 + 200 * DT)
    bad[300] = "%d,1e9,0,0,0,0,9.8" % (T0 + 300 * DT)
    bad[400] = "%d,0.1,0.2" % (T0 + 400 * DT)
    bad[500] = "garbage line"
    e = run(bad, "--no-extrapolate")
    ok &= check("empty/zero/saturated/short rows keep a pose; garbage skipped", len(e) == n - 1)
    ok &= check("no NaN/inf in output", all(math.isfinite(float(x)) for r in e for x in r))
    gap = base[:600] + base[1000:]
    e = run(gap, "--no-extrapolate")
    ok &= check("2 s timestamp gap: one pose per remaining row", len(e) == n - 400)
    ooo = base[:]
    ooo[700], ooo[701] = ooo[701], ooo[700]
    e = run(ooo, "--no-extrapolate")
    ok &= check("out-of-order row keeps a pose", len(e) == n)
    print("ALL PASS" if ok else "SOME TESTS FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
