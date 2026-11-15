#!/usr/bin/env python3
"""
Check that the C++ build (src/pathfinder[.exe]) reproduces src/main.py.

    python parity.py <sequence_dir_or_csv> [...]

Reports identical-line ratio, exact timestamp agreement and max numeric
difference per sequence.  Exit code 1 on any timestamp/row mismatch or a
numeric difference above 1e-4.
"""
import os
import subprocess
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "..", "src")
EXE = os.path.join(SRC, "pathfinder.exe" if os.name == "nt" else "pathfinder")


def main():
    tmp = tempfile.mkdtemp()
    py_out, cpp_out = os.path.join(tmp, "py.txt"), os.path.join(tmp, "cpp.txt")
    ok = True
    for inp in sys.argv[1:]:
        subprocess.run([sys.executable, os.path.join(SRC, "main.py"), inp, "-o", py_out],
                       check=True, capture_output=True)
        subprocess.run([EXE, inp, "-o", cpp_out], check=True, capture_output=True)
        a = open(py_out).read().splitlines()
        b = open(cpp_out).read().splitlines()
        same = sum(x == y for x, y in zip(a, b))
        ts_ok = len(a) == len(b) and all(x.split(" ")[0] == y.split(" ")[0] for x, y in zip(a, b))
        A, B = np.loadtxt(py_out), np.loadtxt(cpp_out)
        d = float(np.abs(A[:, 1:] - B[:, 1:]).max()) if A.shape == B.shape else float("inf")
        good = ts_ok and d < 1e-4
        ok &= good
        print("%-24s rows %d/%d  identical lines %6.2f%%  timestamps %s  max|diff| %.1e  %s" % (
            os.path.basename(os.path.normpath(inp)), len(a), len(b), 100.0 * same / max(len(a), 1),
            "exact" if ts_ok else "MISMATCH", d, "OK" if good else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
