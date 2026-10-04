"""Shrink a walk's MJPG .avi files to H.264 .mp4 (about 10x smaller) for a slow upload.

    pip install imageio-ffmpeg
    py walks\\stereo_record\\shrink_walk.py aq3000_walk1 aq3000_walk2 aq_asb_walk3 --out walkdata

Every frame is kept, in order (-vsync passthrough), so <walk>_times.csv still lines up
frame for frame. CRF 26 keeps the corners visual odometry tracks (8x smaller); raise --crf
(30) for smaller files, lower (20) for sharper. imageio-ffmpeg brings its own ffmpeg, nothing else to install.
"""

import argparse
import subprocess
import sys
from pathlib import Path


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("walks", nargs="+", help="walk names: <name>_left.avi and <name>_right.avi")
    ap.add_argument("--out", type=Path, default=Path("walkdata"))
    ap.add_argument("--crf", type=int, default=26)
    a = ap.parse_args(argv)
    try:
        import imageio_ffmpeg

        ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()
    except ImportError:
        print("pip install imageio-ffmpeg"); return 1
    a.out.mkdir(parents=True, exist_ok=True)
    for walk in a.walks:
        for side in ("left", "right"):
            src = Path(f"{walk}_{side}.avi")
            if not src.exists():
                print(f"no {src}"); return 1
            dst = a.out / f"{Path(walk).name}_{side}.mp4"
            print(f"{src} -> {dst} ...", flush=True)
            subprocess.run([ffmpeg, "-y", "-loglevel", "error", "-i", str(src), "-vsync", "passthrough",
                            "-c:v", "libx264", "-preset", "slow", "-crf", str(a.crf), "-pix_fmt", "yuv420p",
                            "-an", str(dst)], check=True)
            print(f"  {src.stat().st_size / 1e6:.0f} MB -> {dst.stat().st_size / 1e6:.0f} MB")
    print(f"done: git add {a.out}  (each file must be under 100 MB; if not, run again with --crf 30)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
