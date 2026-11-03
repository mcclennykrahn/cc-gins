"""Download ONLY the OXTS folder of KITTI raw drives via HTTP range requests.

KITTI raw *_extract.zip files (20-30 GB, mostly camera/lidar) store the oxts/ members uncompressed
at the start of the archive, so we read the central directory and fetch one contiguous byte range
(~10-35 MB per drive). No full zip is ever written to disk. After download the ~10^4-10^5 tiny
text files are packed into <drive>_<kind>.npz and deleted (keeps disk use ~5 MB per drive).

Usage: python scripts/fetch_kitti_oxts.py [--kind extract] [drive ...]
"""
import argparse
import shutil
import struct
import urllib.request
from pathlib import Path

from remotezip import RemoteZip

from ccgins.data.kitti import consolidate

BASE = "https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data"
DEFAULT_DRIVES = [
    "2011_09_30_drive_0028", "2011_09_30_drive_0018", "2011_09_30_drive_0027",
    "2011_09_30_drive_0033", "2011_09_30_drive_0034", "2011_10_03_drive_0027",
    "2011_10_03_drive_0034", "2011_10_03_drive_0042",
]


def fetch(drive, kind, out_root: Path):
    url = f"{BASE}/{drive}/{drive}_{kind}.zip"
    out = out_root / f"{drive}_{kind}"
    npz = out_root / f"{drive}_{kind}.npz"
    if npz.exists():
        print(f"{drive}: already present")
        return 0
    with RemoteZip(url) as z:
        inf = [i for i in z.infolist() if "/oxts/" in i.filename and not i.filename.endswith("/")]
    lo = min(i.header_offset for i in inf)
    hi = max(i.header_offset + 30 + len(i.filename.encode()) + 1024 + i.compress_size for i in inf)
    req = urllib.request.Request(url, headers={"Range": f"bytes={lo}-{hi}"})
    blob = urllib.request.urlopen(req, timeout=300).read()
    for i in inf:
        assert i.compress_type == 0, "expected stored members"
        o = i.header_offset - lo
        sig, = struct.unpack("<I", blob[o:o + 4])
        assert sig == 0x04034B50
        nlen, xlen = struct.unpack("<HH", blob[o + 26:o + 30])
        start = o + 30 + nlen + xlen
        data = blob[start:start + i.compress_size]
        rel = i.filename.split("/oxts/", 1)[1]
        dst = out / "oxts" / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        dst.write_bytes(data)
    consolidate(out, npz)
    shutil.rmtree(out)
    print(f"{drive}: {len(inf)} files, downloaded {len(blob) / 1e6:.1f} MB -> {npz.name}")
    return len(blob)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("drives", nargs="*", default=DEFAULT_DRIVES)
    ap.add_argument("--kind", default="extract")
    ap.add_argument("--out", default="data/kitti")
    a = ap.parse_args()
    tot = 0
    for d in a.drives:
        try:
            tot += fetch(d, a.kind, Path(a.out))
        except Exception as e:  # noqa: BLE001
            print(f"{d}: FAILED {e!r}")
    print(f"total downloaded: {tot / 1e6:.1f} MB")
