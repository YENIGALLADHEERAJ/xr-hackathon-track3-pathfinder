#!/usr/bin/env python3
"""
Build the Track 3 submission bundle in one step (stdlib only).

    python tools/make_bundle.py <evaluation dataset: zip | folder | csv> <CollegeName_TeamName> [--out DIR]

Runs src/main.py on the dataset, checks the output contract against the input
(exactly one pose per IMU row, timestamps in input order, 8 finite columns,
unit quaternions), and writes <CollegeName_TeamName>.zip laid out as slide 17
requires:

    <team>.zip
    ├── estimated_trajectory.txt
    ├── metadata.json
    └── src/
        ├── main.py
        └── README.md
"""
import argparse
import math
import os
import shutil
import subprocess
import sys
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "..", "src")
sys.path.insert(0, SRC)
import main as M  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dataset")
    ap.add_argument("team", help="registered name, e.g. CollegeName_TeamName")
    ap.add_argument("--out", default=".", help="folder to write the bundle into")
    ap.add_argument("--gyro-deg", action="store_true")
    a = ap.parse_args()
    stage = os.path.join(a.out, a.team)
    if os.path.exists(stage):
        shutil.rmtree(stage)
    os.makedirs(os.path.join(stage, "src"))
    traj = os.path.join(stage, "estimated_trajectory.txt")
    meta = os.path.join(stage, "metadata.json")
    cmd = [sys.executable, os.path.join(SRC, "main.py"), a.dataset, "-o", traj, "--metadata", meta]
    if a.gyro_deg:
        cmd.append("--gyro-deg")
    subprocess.run(cmd, check=True)

    # ---- verify the contract against the input itself ----
    fmt, rows = M.open_rows(M.open_imu_csv(a.dataset), a.gyro_deg)
    want = [M.fmt_ts(ts) for ts, _ in rows]
    got = [l.split() for l in open(traj) if not l.startswith("#")]
    problems = []
    if len(got) != len(want):
        problems.append("row count: %d poses for %d IMU rows" % (len(got), len(want)))
    if [g[0] for g in got] != want:
        problems.append("timestamps differ from the input rows")
    if any(len(g) != 8 for g in got):
        problems.append("a row does not have 8 columns")
    else:
        vals = [[float(x) for x in g[1:]] for g in got]
        if not all(math.isfinite(x) for v in vals for x in v):
            problems.append("non-finite value in the output")
        if any(abs(sum(x * x for x in v[3:]) - 1.0) > 1e-6 for v in vals):
            problems.append("a quaternion is not unit length")
    if problems:
        raise SystemExit("CONTRACT CHECK FAILED:\n  " + "\n  ".join(problems))

    shutil.copyfile(os.path.join(SRC, "main.py"), os.path.join(stage, "src", "main.py"))
    shutil.copyfile(os.path.join(SRC, "README.md"), os.path.join(stage, "src", "README.md"))
    zpath = os.path.join(a.out, a.team + ".zip")
    if os.path.exists(zpath):
        os.remove(zpath)
    with zipfile.ZipFile(zpath, "w", zipfile.ZIP_DEFLATED) as z:
        for rel in ("estimated_trajectory.txt", "metadata.json", "src/main.py", "src/README.md"):
            z.write(os.path.join(stage, rel), rel)
    print("contract OK: %d poses for %d IMU rows (%s, %s)" % (
        len(got), len(want), fmt.describe()["column_order"], fmt.describe()["timestamp_unit"]))
    print("bundle: %s (%.1f MB)" % (os.path.abspath(zpath), os.path.getsize(zpath) / 2 ** 20))
    with zipfile.ZipFile(zpath) as z:
        for n in z.namelist():
            print("   ", n)


if __name__ == "__main__":
    main()
