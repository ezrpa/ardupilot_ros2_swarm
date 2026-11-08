#!/usr/bin/env python3
"""Record a Gazebo camera sensor topic to video, free of real-time-factor lag.

Why this exists
---------------
Screen-capturing the Gazebo window records WALL-CLOCK time, so whenever the
simulation runs below real time the video comes out in slow motion and stutters
whenever the physics loop hitches. Neither artefact is in the simulation; both
are artefacts of the capture method.

A Gazebo camera sensor instead publishes exactly one image per sensor update,
and sensor updates are paced by SIMULATION time (`<update_rate>` is in sim
seconds). So N frames always represent N/update_rate seconds of simulated time,
no matter how long the machine took to produce them. Feeding those frames to an
encoder at a fixed frame rate therefore yields perfectly smooth video whose
playback speed is a known multiple of simulated real time:

    playback_speed = output_fps / sensor_update_rate

Subscribing is also what makes the sensor render at all: gz-sim skips updating
camera sensors that have no subscribers, which is why a sensor configured with
`<save>` alone writes nothing.

Usage
-----
    gz_cine_recorder.py --topic /world/.../image --out flight.mp4 \
        --update-rate 20 --fps 30          # -> 1.5x speed, smooth
"""

import argparse
import subprocess
import sys
import threading
import time

from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node

# gz.msgs Image pixel_format_type enum values we can hand to ffmpeg directly.
PIX_FMT = {
    3: "rgb24",     # RGB_INT8
    4: "bgr24",     # BGR_INT8
    9: "gray",      # L_INT8
}


class Recorder:
    def __init__(self, out, fps, crf, preset):
        self.out, self.fps, self.crf, self.preset = out, fps, crf, preset
        self.proc = None
        self.count = 0
        self.dropped = 0
        self.size = None
        self.lock = threading.Lock()
        self.last_report = time.time()

    def _start(self, width, height, pix_fmt):
        cmd = [
            "ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
            "-f", "rawvideo", "-pixel_format", pix_fmt,
            "-video_size", f"{width}x{height}",
            # Input is declared at the OUTPUT rate: every frame is one tick of
            # simulated time, so the encoder must not resample or drop.
            "-framerate", str(self.fps),
            "-i", "-",
            "-an", "-c:v", "libx264", "-preset", self.preset, "-crf", str(self.crf),
            "-pix_fmt", "yuv420p", "-movflags", "+faststart",
            self.out,
        ]
        print(f"[rec] {width}x{height} {pix_fmt} -> {self.out} @ {self.fps} fps",
              flush=True)
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)

    def on_image(self, msg: Image):
        pix_fmt = PIX_FMT.get(msg.pixel_format_type)
        if pix_fmt is None:
            print(f"[rec] unsupported pixel_format_type={msg.pixel_format_type}",
                  file=sys.stderr, flush=True)
            return
        with self.lock:
            if self.proc is None:
                self.size = (msg.width, msg.height)
                self._start(msg.width, msg.height, pix_fmt)
            elif (msg.width, msg.height) != self.size:
                return  # resolution change mid-stream: ignore rather than corrupt
            try:
                self.proc.stdin.write(msg.data)
                self.count += 1
            except (BrokenPipeError, ValueError):
                self.dropped += 1
            now = time.time()
            if now - self.last_report >= 10.0:
                sim_s = self.count / self.fps
                print(f"[rec] {self.count} frames "
                      f"({sim_s:.0f}s of output video) wall={now - self.t0:.0f}s",
                      flush=True)
                self.last_report = now

    def close(self):
        with self.lock:
            if self.proc is not None:
                try:
                    self.proc.stdin.close()
                except Exception:
                    pass
                self.proc.wait(timeout=120)
        print(f"[rec] finished: {self.count} frames -> {self.out}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", required=True, help="gz camera image topic")
    ap.add_argument("--out", required=True, help="output .mp4")
    ap.add_argument("--fps", type=int, default=30,
                    help="encoder frame rate; playback_speed = fps / update_rate")
    ap.add_argument("--crf", type=int, default=20)
    ap.add_argument("--preset", default="veryfast")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="stop after N wall seconds (0 = until SIGINT)")
    args = ap.parse_args()

    rec = Recorder(args.out, args.fps, args.crf, args.preset)
    rec.t0 = time.time()

    node = Node()
    if not node.subscribe(Image, args.topic, rec.on_image):
        print(f"[rec] FAILED to subscribe to {args.topic}", file=sys.stderr)
        return 1
    print(f"[rec] subscribed to {args.topic}", flush=True)

    try:
        if args.duration > 0:
            deadline = time.time() + args.duration
            while time.time() < deadline:
                time.sleep(0.5)
        else:
            while True:
                time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        rec.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
