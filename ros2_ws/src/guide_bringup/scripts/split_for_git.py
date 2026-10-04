"""Cut big files into pieces GitHub accepts (under 100 MB each), and join them again.

    py ros2_ws\\src\\guide_bringup\\scripts\\split_for_git.py walk_left.avi walk_right.avi --out walks\\stereo_record
    python3 ros2_ws/src/guide_bringup/scripts/split_for_git.py --join walks/stereo_record

Splitting writes <name>.part000, .part001, ... and <name>.sha256 next to them;
--join puts every file back together and checks it. Standard library only.
"""

from __future__ import annotations

import argparse
import hashlib
import sys
from pathlib import Path

PIECE = 90 * 1024 * 1024


def split(path: Path, out: Path) -> None:
    out.mkdir(parents=True, exist_ok=True)
    digest, n = hashlib.sha256(), 0
    with open(path, "rb") as f:
        while chunk := f.read(PIECE):
            digest.update(chunk)
            (out / f"{path.name}.part{n:03d}").write_bytes(chunk)
            n += 1
    (out / f"{path.name}.sha256").write_text(f"{digest.hexdigest()}  {path.name}  {n} pieces\n")
    print(f"{path.name}: {n} pieces")


def join(folder: Path) -> int:
    bad = 0
    for sums in sorted(folder.glob("*.sha256")):
        want, name = sums.read_text().split()[:2]
        parts = sorted(folder.glob(f"{name}.part*"))
        digest = hashlib.sha256()
        with open(folder / name, "wb") as f:
            for p in parts:
                data = p.read_bytes()
                digest.update(data)
                f.write(data)
        ok = digest.hexdigest() == want
        bad += not ok
        print(f"{name}: {len(parts)} pieces, {'ok' if ok else 'CHECKSUM MISMATCH: a piece is missing or damaged'}")
    return 1 if bad else 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("files", nargs="*", type=Path)
    ap.add_argument("--out", type=Path, default=Path("."))
    ap.add_argument("--join", type=Path, metavar="FOLDER")
    args = ap.parse_args(argv)
    if args.join:
        return join(args.join)
    for f in args.files:
        split(f, args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
