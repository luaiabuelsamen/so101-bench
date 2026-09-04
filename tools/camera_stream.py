"""Serve the bench camera as MJPEG so it can be watched from a browser.

The bench is driven over SSH, where an X-forwarded cv2 window is slow enough to
be misleading about framing. This streams over HTTP instead.

Holds the camera open for as long as it runs, so stop it before teleoperating or
recording -- the device allows one reader.

    python scripts/camera_stream.py [--device /dev/video0] [--port 8000]
"""

import argparse
import socket
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

BOUNDARY = "frameboundary"

PAGE = b"""<!doctype html><meta charset=utf-8><title>bench camera</title>
<style>
 body{margin:0;background:#111;color:#ddd;font:14px system-ui;text-align:center}
 img{max-width:100%;height:auto;image-rendering:pixelated;border:1px solid #333}
 p{opacity:.65}
</style>
<h3>bench camera</h3>
<img src="/stream">
<p>live &mdash; stop the server before teleop or record</p>
"""


class Camera:
    """One reader, many viewers: the newest frame is shared under a lock."""

    def __init__(self, device, width, height):
        self.cap = cv2.VideoCapture(device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if not self.cap.isOpened():
            raise SystemExit(f"could not open {device} (is it held by teleop/record?)")
        self.lock = threading.Lock()
        self.jpeg = None
        self.stop = threading.Event()
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while not self.stop.is_set():
            ok, frame = self.cap.read()
            if not ok:
                continue
            ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                with self.lock:
                    self.jpeg = buf.tobytes()

    def latest(self):
        with self.lock:
            return self.jpeg


class Handler(BaseHTTPRequestHandler):
    camera = None

    def log_message(self, *a):  # keep the console quiet
        pass

    def do_GET(self):
        if self.path != "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(PAGE)
            return

        self.send_response(200)
        self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={BOUNDARY}")
        self.end_headers()
        try:
            while True:
                frame = self.camera.latest()
                if frame is None:
                    continue
                self.wfile.write(
                    f"--{BOUNDARY}\r\nContent-Type: image/jpeg\r\n"
                    f"Content-Length: {len(frame)}\r\n\r\n".encode()
                )
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            pass  # viewer closed the tab


def lan_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except Exception:
        return "127.0.0.1"
    finally:
        s.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default="/dev/video0")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    args = ap.parse_args()

    Handler.camera = Camera(args.device, args.width, args.height)
    server = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    print(f"streaming {args.device} -> http://{lan_ip()}:{args.port}/", flush=True)
    print("Ctrl-C to stop (frees the camera).", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nstopping, camera released")


if __name__ == "__main__":
    main()
