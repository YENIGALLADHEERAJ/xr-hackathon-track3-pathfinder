#!/usr/bin/env python3
"""
Fetch only the IMU + ground-truth CSVs of EuRoC sequences out of a large remote
zip, using HTTP range requests (no images are downloaded).

    python fetch_euroc.py <zip_url> <out_dir> [--list] [--match V1_01]

Handles ZIP64 archives and archives that contain per-sequence inner zips
(read recursively through range requests when the inner zip is stored, or by
inflating the member stream otherwise).
"""
import argparse
import io
import os
import struct
import urllib.request
import zlib

WANT = ("imu0/data.csv", "state_groundtruth_estimate0/data.csv", "vicon0/data.csv", "leica0/data.csv")


class Remote:
    UA = {"User-Agent": "curl/8.5.0"}

    # All requests go through curl with retries: on flaky links it is far more
    # robust than urllib.  The URL is re-resolved on every request because the
    # CDN redirect targets are signed per request.
    CURL = ["curl", "-sL", "-A", UA["User-Agent"], "--retry", "8", "--retry-all-errors",
            "--retry-delay", "5", "--connect-timeout", "60", "--speed-limit", "1000", "--speed-time", "120"]

    def __init__(self, url):
        import subprocess
        self.url = url
        self.size = None
        for attempt in range(8):
            r = subprocess.run(self.CURL + ["-I", url], capture_output=True, text=True)
            lens = [l.split(":", 1)[1].strip() for l in r.stdout.splitlines()
                    if l.lower().startswith("content-length:")]
            if lens:
                self.size = int(lens[-1])
                break
            print("  HEAD retry %d" % (attempt + 1), flush=True)
        if self.size is None:
            raise IOError("could not get remote size for " + url)
        self.fetched = 0

    def read(self, off, n):
        import subprocess
        data = b""
        for attempt in range(8):
            r = subprocess.run(self.CURL + ["-r", "%d-%d" % (off, off + n - 1), self.url],
                               capture_output=True)
            data = r.stdout
            if len(data) == n:
                break
            print("  range read retry %d (%d/%d bytes)" % (attempt + 1, len(data), n), flush=True)
        if len(data) != n:
            raise IOError("short range read %d/%d (server may not support ranges)" % (len(data), n))
        self.fetched += n
        return data


class Sub:
    """A byte range inside another readable (for stored inner zips)."""
    def __init__(self, base, off, size):
        self.base, self.off, self.size = base, off, size

    def read(self, off, n):
        return self.base.read(self.off + off, n)


def central_directory(src):
    tail_n = min(src.size, 65536 + 22)
    tail = src.read(src.size - tail_n, tail_n)
    i = tail.rfind(b"PK\x05\x06")
    if i < 0:
        raise IOError("no end-of-central-directory record")
    _, _, _, _, n_ent, cd_size, cd_off, _ = struct.unpack("<IHHHHIIH", tail[i:i + 22])
    if cd_off == 0xFFFFFFFF or n_ent == 0xFFFF:
        j = tail.rfind(b"PK\x06\x07")
        loc_off = struct.unpack("<Q", tail[j + 8:j + 16])[0]
        z64 = src.read(loc_off, 56)
        n_ent, cd_size, cd_off = struct.unpack("<QQQ", z64[32:56])
    cd = src.read(cd_off, cd_size)
    out, p = [], 0
    for _ in range(n_ent):
        (sig, _, _, _, method, _, _, _, csize, usize, nl, xl, cl, _, _, _, loff) = \
            struct.unpack("<IHHHHHHIIIHHHHHII", cd[p:p + 46])
        name = cd[p + 46:p + 46 + nl].decode("utf-8", "replace")
        extra = cd[p + 46 + nl:p + 46 + nl + xl]
        if 0xFFFFFFFF in (csize, usize, loff):
            q = 0
            while q + 4 <= len(extra):
                hid, hl = struct.unpack("<HH", extra[q:q + 4])
                if hid == 1:
                    vals = list(struct.unpack("<%dQ" % (hl // 8), extra[q + 4:q + 4 + hl]))
                    if usize == 0xFFFFFFFF: usize = vals.pop(0)
                    if csize == 0xFFFFFFFF: csize = vals.pop(0)
                    if loff == 0xFFFFFFFF: loff = vals.pop(0)
                q += 4 + hl
        out.append(dict(name=name, method=method, csize=csize, usize=usize, loff=loff))
        p += 46 + nl + xl + cl
    return out


def data_offset(src, e):
    lh = src.read(e["loff"], 30)
    nl, xl = struct.unpack("<HH", lh[26:30])
    return e["loff"] + 30 + nl + xl


def extract(src, e):
    raw = src.read(data_offset(src, e), e["csize"])
    if e["method"] == 0:
        return raw
    if e["method"] == 8:
        return zlib.decompress(raw, -15)
    raise IOError("unsupported method %d" % e["method"])


def fetch_compressed_inner(src, e, full, out_dir):
    """Stream one deflated inner zip (single ranged GET), inflate it to a temp
    file, extract only the wanted CSVs, then delete the temp file."""
    import tempfile
    import time
    import zipfile
    import subprocess
    from concurrent.futures import ThreadPoolExecutor
    off = data_offset(src, e)
    n = e["csize"]
    CH = 16 << 20
    parts_dir = os.path.join(out_dir, "_parts_" + os.path.basename(full))
    os.makedirs(parts_dir, exist_ok=True)
    jobs = [(i, off + s, min(n - s, CH)) for i, s in enumerate(range(0, n, CH))]
    t0 = time.time()

    def get(job):                       # resumable: finished chunks are kept on disk
        i, a, ln = job
        p = os.path.join(parts_dir, "%05d" % i)
        for attempt in range(8):
            if os.path.exists(p) and os.path.getsize(p) == ln:
                return ln
            subprocess.run(["curl", "-sL", "--max-time", "600", "--speed-limit", "20000",
                            "--speed-time", "60", "-A", Remote.UA["User-Agent"],
                            "-r", "%d-%d" % (a, a + ln - 1), "-o", p, src.url])
            time.sleep(min(2 ** attempt, 30))
        raise IOError("chunk %d failed after retries" % i)

    done = 0
    with ThreadPoolExecutor(4) as ex:
        for ln in ex.map(get, jobs):
            done += ln
            el = time.time() - t0
            print("  %s: %.0f / %.0f MB  (%.1f MB/s)" % (full, done / 2 ** 20, n / 2 ** 20,
                                                        done / 2 ** 20 / max(el, 1e-6)), flush=True)
    dec = zlib.decompressobj(-15)
    tmp = tempfile.NamedTemporaryFile(suffix=".zip", delete=False, dir=out_dir)
    got = 0
    try:
        for i, _, ln in jobs:
            p = os.path.join(parts_dir, "%05d" % i)
            with open(p, "rb") as fh:
                data = fh.read()
            got += len(data)
            tmp.write(dec.decompress(data))
        tmp.write(dec.flush())
        tmp.close()
        src.fetched += got
        if got != n:
            raise IOError("download incomplete: %d of %d bytes" % (got, n))
        base = os.path.join(out_dir, os.path.splitext(os.path.basename(full))[0])
        with zipfile.ZipFile(tmp.name) as z:
            for name in z.namelist():
                if any(name.endswith(w) for w in WANT):
                    dst = os.path.join(base, name)
                    os.makedirs(os.path.dirname(dst), exist_ok=True)
                    with open(dst, "wb") as f:
                        f.write(z.read(name))
                    print("  extracted %s  %.2f MB" % (dst, os.path.getsize(dst) / 2 ** 20), flush=True)
        import shutil
        shutil.rmtree(parts_dir, ignore_errors=True)
    finally:
        tmp.close()
        os.remove(tmp.name)


def walk(src, prefix, match, out_dir, list_only, depth=0):
    for e in central_directory(src):
        full = prefix + e["name"]
        if e["name"].endswith(".zip") and depth < 2:
            if match and match not in full:
                continue
            print("%s[inner zip] %s  %.1f MB (method %d)" % ("  " * depth, full, e["csize"] / 2 ** 20, e["method"]))
            if e["method"] == 0:
                walk(Sub(src, data_offset(src, e), e["csize"]), full + "!/", match, out_dir, list_only, depth + 1)
            elif list_only or not match:
                print("  compressed inner zip: needs full %.0f MB download (use --match to pick one)"
                      % (e["csize"] / 2 ** 20))
            else:
                fetch_compressed_inner(src, e, full, out_dir)
            continue
        if not any(e["name"].endswith(w) for w in WANT):
            continue
        if match and match not in full:
            continue
        print("%s%s  %.2f MB" % ("  " * depth, full, e["usize"] / 2 ** 20))
        if list_only:
            continue
        rel = full.replace("!/", "/").replace(".zip/", "/")
        dst = os.path.join(out_dir, rel)
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        with open(dst, "wb") as f:
            f.write(extract(src, e))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("url")
    ap.add_argument("out")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--match", default=None)
    a = ap.parse_args()
    src = Remote(a.url)
    print("remote size %.1f MB" % (src.size / 2 ** 20))
    walk(src, "", a.match, a.out, a.list)
    print("downloaded %.2f MB via range requests" % (src.fetched / 2 ** 20))


if __name__ == "__main__":
    main()
