#!/usr/bin/env python3
"""
Resource benchmark for main.py against the Track-3 budgets
(<= 8 MB peak RAM, <= 20 ms compute per 1 s of IMU, single thread).

    python benchmark.py <imu input> [--repeat 3]

Reports
    * runtime per IMU-second (best of N, measured inside main.run)
    * Python heap peak (tracemalloc)        - memory the algorithm allocates
    * whole-process peak RSS (OS)           - includes the interpreter itself
    * a baseline peak RSS of `python -c pass` for comparison
"""
import argparse
import json
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
MAIN = os.path.join(HERE, "..", "src", "main.py")

PEAK_RSS_SNIPPET = r"""
import sys, runpy, os
def peak():
    if os.name == 'nt':
        import ctypes, ctypes.wintypes as w
        class PMC(ctypes.Structure):
            _fields_ = [('cb', w.DWORD), ('PageFaultCount', w.DWORD),
                        ('PeakWorkingSetSize', ctypes.c_size_t), ('WorkingSetSize', ctypes.c_size_t),
                        ('QuotaPeakPagedPoolUsage', ctypes.c_size_t), ('QuotaPagedPoolUsage', ctypes.c_size_t),
                        ('QuotaPeakNonPagedPoolUsage', ctypes.c_size_t), ('QuotaNonPagedPoolUsage', ctypes.c_size_t),
                        ('PagefileUsage', ctypes.c_size_t), ('PeakPagefileUsage', ctypes.c_size_t)]
        c = PMC(); c.cb = ctypes.sizeof(PMC)
        k32 = ctypes.WinDLL('kernel32')
        k32.GetCurrentProcess.restype = w.HANDLE
        f = k32.K32GetProcessMemoryInfo
        f.argtypes = [w.HANDLE, ctypes.POINTER(PMC), w.DWORD]
        f.restype = w.BOOL
        f(k32.GetCurrentProcess(), ctypes.byref(c), c.cb)
        return c.PeakWorkingSetSize
    import resource
    r = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    return r if sys.platform == 'darwin' else r * 1024
script = sys.argv[1]
if script != '-':
    sys.argv = sys.argv[1:]
    try:
        runpy.run_path(script, run_name='__main__')
    except SystemExit:
        pass
sys.stderr.write('PEAK_RSS %d\n' % peak())
"""


def peak_rss(args):
    r = subprocess.run([sys.executable, "-c", PEAK_RSS_SNIPPET] + args,
                       capture_output=True, text=True)
    for line in r.stderr.splitlines():
        if line.startswith("PEAK_RSS"):
            return int(line.split()[1])
    raise RuntimeError(r.stderr)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input")
    ap.add_argument("--repeat", type=int, default=3)
    a = ap.parse_args()
    tmp = tempfile.mkdtemp()
    out = os.path.join(tmp, "est.txt")
    meta = os.path.join(tmp, "meta.json")
    best = None
    for _ in range(a.repeat):
        subprocess.run([sys.executable, MAIN, a.input, "-o", out, "--metadata", meta],
                       check=True, capture_output=True)
        m = json.load(open(meta))
        if best is None or m["runtime_ms_per_imu_second"] < best["runtime_ms_per_imu_second"]:
            best = m
    subprocess.run([sys.executable, MAIN, a.input, "-o", out, "--metadata", meta, "--profile-memory"],
                   check=True, capture_output=True)
    heap = json.load(open(meta))["python_peak_heap_bytes"]
    rss = peak_rss([MAIN, a.input, "-o", out])
    base = peak_rss(["-"])
    res = dict(
        imu_rows=best["imu_rows"],
        imu_span_s=best["imu_span_s"],
        runtime_s=best["runtime_s"],
        runtime_ms_per_imu_second=best["runtime_ms_per_imu_second"],
        budget_ms_per_imu_second=20.0,
        python_heap_peak_MB=round(heap / 2 ** 20, 3),
        process_peak_rss_MB=round(rss / 2 ** 20, 2),
        bare_interpreter_peak_rss_MB=round(base / 2 ** 20, 2),
        algorithm_rss_over_interpreter_MB=round((rss - base) / 2 ** 20, 2),
        budget_peak_ram_MB=8.0,
        threads=1,
    )
    print(json.dumps(res, indent=2))


if __name__ == "__main__":
    main()
