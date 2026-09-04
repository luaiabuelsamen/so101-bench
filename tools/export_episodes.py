"""Export a recorded dataset's episodes as individually viewable H.264 clips.

lerobot writes one AV1 file per chunk holding every episode end to end. AV1 does
not play in Safari on most Macs, and a single blob is awkward to review, so this
cuts the chunk at the episode boundaries recorded in the parquet and re-encodes
to H.264, then writes an index page.

    python scripts/export_episodes.py --root data/real/pickplace_real_v0
    python scripts/export_episodes.py --root ... --serve 8001
"""

import argparse
import glob
import subprocess
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq


def episode_spans(root: Path, fps: float):
    """(episode, start_s, duration_s) from the recorded frame counts."""
    files = sorted(glob.glob(str(root / "data" / "**" / "*.parquet"), recursive=True))
    if not files:
        raise SystemExit(f"no parquet under {root}/data")
    table = pa.concat_tables([pq.read_table(f) for f in files])
    idx = np.array(table["episode_index"].to_pylist())
    spans, cursor = [], 0
    for ep in sorted(set(int(e) for e in idx)):
        n = int((idx == ep).sum())
        spans.append((ep, cursor / fps, n / fps, n))
        cursor += n
    return spans


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True)
    ap.add_argument("--out", default=None, help="default: <root>/clips")
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--serve", type=int, default=0, help="serve the clips on this port")
    args = ap.parse_args()

    root = Path(args.root).resolve()
    out = Path(args.out) if args.out else root / "clips"
    out.mkdir(parents=True, exist_ok=True)

    sources = sorted(glob.glob(str(root / "videos" / "**" / "*.mp4"), recursive=True))
    if not sources:
        raise SystemExit(f"no source video under {root}/videos")
    source = sources[0]
    if len(sources) > 1:
        print(f"note: {len(sources)} chunk files; using {source}")

    spans = episode_spans(root, args.fps)
    print(f"{len(spans)} episodes from {Path(source).name}")

    written = []
    for ep, start, dur, frames in spans:
        dest = out / f"episode_{ep:03d}.mp4"
        cmd = [
            "ffmpeg", "-y", "-loglevel", "error",
            "-i", source, "-ss", f"{start:.3f}", "-t", f"{dur:.3f}",
            "-c:v", "libx264", "-preset", "veryfast", "-crf", "23",
            "-pix_fmt", "yuv420p", "-movflags", "+faststart",
            str(dest),
        ]
        subprocess.run(cmd, check=True)
        mb = dest.stat().st_size / 1e6
        print(f"  episode {ep}: {frames:4d} frames  {dur:5.1f}s  -> {dest.name} ({mb:.1f} MB)")
        written.append((ep, dest.name, dur, frames))

    cards = "\n".join(
        f'<figure><figcaption>episode {ep} &mdash; {d:.1f}s, {n} frames</figcaption>'
        f'<video src="{name}" controls preload="metadata"></video></figure>'
        for ep, name, d, n in written
    )
    (out / "index.html").write_text(
        "<!doctype html><meta charset=utf-8><title>episodes</title>"
        "<style>body{background:#111;color:#ddd;font:14px system-ui;margin:24px}"
        "figure{margin:0 0 28px}video{width:100%;max-width:640px;border:1px solid #333}"
        "figcaption{opacity:.7;margin-bottom:6px}</style>"
        f"<h2>{root.name}</h2>{cards}"
    )
    print(f"\nwrote {out}/index.html")

    if args.serve:
        import socket

        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()

        class H(SimpleHTTPRequestHandler):
            def __init__(self, *a, **k):
                super().__init__(*a, directory=str(out), **k)

            def log_message(self, *a):
                pass

        print(f"serving http://{ip}:{args.serve}/  (Ctrl-C to stop)", flush=True)
        ThreadingHTTPServer(("0.0.0.0", args.serve), H).serve_forever()


if __name__ == "__main__":
    main()
