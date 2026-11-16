#!/usr/bin/env python3
"""
Fetch only the IMU and motion-capture CSVs of a TUM-VI sequence (EuRoC-format
export, an UNCOMPRESSED .tar) by HTTP range requests, without downloading the
images.

    python fetch_tumvi.py <tar_url> <out_dir> [--chunk-mb 24]

Tar has no index, but members are stored back to back with 512-byte headers;
the camera images come first, so the CSVs sit near the end.  The tail of the
archive is read in chunks (walking backwards until both CSVs are found),
headers are recognised by the 'ustar' magic plus a valid checksum, and the
members are extracted.  mocap0/data.csv is also written as
state_groundtruth_estimate0/data.csv so the EuRoC tools can read it.
"""
import argparse
import os
import shutil
import subprocess

WANT = ("imu0/data.csv", "mocap0/data.csv")


def head_size(url):
    r = subprocess.run(["curl", "-sIL", "-A", "curl/8.5.0", "--retry", "5", url], capture_output=True, text=True)
    lens = [l.split(":", 1)[1].strip() for l in r.stdout.splitlines() if l.lower().startswith("content-length:")]
    return int(lens[-1])


def get(url, a, b):
    for _ in range(6):
        r = subprocess.run(["curl", "-sL", "-A", "curl/8.5.0", "--retry", "5", "--retry-all-errors",
                            "-r", "%d-%d" % (a, b - 1), url], capture_output=True)
        if len(r.stdout) == b - a:
            return r.stdout
    raise IOError("range %d-%d failed" % (a, b))


def headers(buf, base):
    """Yield (offset, name, size) for valid tar headers in buf (buf starts at a 512-aligned offset)."""
    for i in range(0, len(buf) - 511, 512):
        blk = buf[i:i + 512]
        if blk[257:262] != b"ustar":
            continue
        try:
            chk = int(blk[148:156].split(b"\0")[0].strip() or b"0", 8)
        except ValueError:
            continue
        if chk != sum(blk[:148]) + 8 * 32 + sum(blk[156:]):
            continue
        name = blk[0:100].split(b"\0")[0].decode("utf-8", "replace")
        prefix = blk[345:500].split(b"\0")[0].decode("utf-8", "replace")
        if prefix:
            name = prefix + "/" + name
        size = int(blk[124:136].split(b"\0")[0].strip() or b"0", 8)
        yield base + i, name, size


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("url")
    ap.add_argument("out")
    ap.add_argument("--chunk-mb", type=float, default=24)
    ap.add_argument("--max-mb", type=float, default=100,
                    help="stop scanning after this much (the CSVs can sit mid-archive: "
                         "TUM-VI room1 needed ~845 MB because cam1 images follow them)")
    a = ap.parse_args()
    total = head_size(a.url)
    chunk = int(a.chunk_mb * 2 ** 20) // 512 * 512
    found = {}
    end = total
    fetched = 0
    while end > 0 and len(found) < len(WANT) and fetched < a.max_mb * 2 ** 20:
        start = max(0, (end - chunk) // 512 * 512)
        buf = get(a.url, start, end)
        fetched += len(buf)
        for off, name, size in headers(buf, start):
            for w in WANT:
                if name.endswith(w) and w not in found:
                    found[w] = (off, size)
        end = start
        print("scanned %.0f MB of tail, found %s" % (fetched / 2 ** 20, sorted(found)), flush=True)
    if len(found) < len(WANT):
        raise SystemExit("could not find %s" % [w for w in WANT if w not in found])
    for w, (off, size) in found.items():
        data = get(a.url, off + 512, off + 512 + size)
        fetched += size
        dst = os.path.join(a.out, "mav0", w)
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        with open(dst, "wb") as f:
            f.write(data)
        print("extracted %s  %.2f MB" % (dst, size / 2 ** 20))
    gt = os.path.join(a.out, "mav0", "state_groundtruth_estimate0", "data.csv")
    os.makedirs(os.path.dirname(gt), exist_ok=True)
    shutil.copyfile(os.path.join(a.out, "mav0", "mocap0", "data.csv"), gt)
    print("total downloaded %.1f MB of %.0f MB archive" % (fetched / 2 ** 20, total / 2 ** 20))


if __name__ == "__main__":
    main()
