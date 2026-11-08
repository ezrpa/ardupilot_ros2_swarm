#!/usr/bin/env python3
"""Local TAK common operating picture: a browser map of the swarm's CoT feed.

The swarm's tak_bridge.py publishes Cursor-on-Target; normally you watch it in
ATAK / WinTAK / iTAK. This viewer is the no-install alternative (and, with the
synthetic camera, the no-Gazebo way to SEE a mission): it subscribes to the
same CoT feed with PyTAK, keeps the live picture (tracks, drawings, spot
markers, CASEVAC, GeoChat, t-x-d-d deletes, stale times) and serves it to a
Leaflet page on http://localhost:<port>.

    tak_viewer.py                                  # ATAK multicast 239.2.3.1:6969
    tak_viewer.py --listen udp://127.0.0.1:6970    # unicast from the bridge
    tak_viewer.py --logs ./logs                    # also serve confirm images

Receive transport: PyTAK (udp+ro://, joins the multicast group with
SO_REUSEADDR so ATAK clients on the same host still work). Without PyTAK it
falls back to a plain stdlib socket, so it also runs on a bare host Python.

Only stdlib + optional pytak; no ROS. Read-only: never sends CoT.
"""
import argparse
import asyncio
import json
import math
import os
import re
import socket
import struct
import threading
import time
import xml.etree.ElementTree as ET
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, unquote

HERE = os.path.dirname(os.path.abspath(__file__))
PAGE_CANDIDATES = [os.path.join(HERE, "..", "viewer", "index.html"),
                   os.path.join(HERE, "viewer", "index.html")]
TRAIL_MAX = 900            # points kept per track (~15 min at 1 Hz)
FOOTPRINT_STEP_M = 12.0    # new coverage footprint every N m of search travel
FEED_MAX = 80
MIN_FOOTPRINT_AGL, MAX_FOOTPRINT_AGL = 5.0, 150.0
TYPE_DELETE = "t-x-d-d"
TYPE_CHAT = "b-t-f"
M_PER_DEG = 111111.0


def _parse_time(s):
    if not s:
        return None
    try:
        return datetime.fromisoformat(s.replace("Z", "+00:00")).timestamp()
    except ValueError:
        return None


def _dist_m(a_lat, a_lon, b_lat, b_lon):
    dn = (b_lat - a_lat) * M_PER_DEG
    de = (b_lon - a_lon) * M_PER_DEG * math.cos(math.radians(a_lat))
    return math.hypot(dn, de)


def parse_cot(data):
    """bytes/str -> dict or None. Tolerates XML declarations and
    concatenated events (TCP framing): returns the first event only."""
    if isinstance(data, bytes):
        data = data.decode("utf-8", "replace")
    i = data.find("<event")
    if i < 0:
        return None
    j = data.find("</event>", i)
    if j < 0:
        return None
    try:
        root = ET.fromstring(data[i:j + len("</event>")])
    except ET.ParseError:
        return None
    pt = root.find("point")
    det = root.find("detail")
    ev = {
        "uid": root.get("uid"), "type": root.get("type", ""),
        "how": root.get("how"),
        "time": _parse_time(root.get("time")),
        "stale": _parse_time(root.get("stale")),
        "lat": float(pt.get("lat", 0)) if pt is not None else 0.0,
        "lon": float(pt.get("lon", 0)) if pt is not None else 0.0,
        "hae": float(pt.get("hae", 0)) if pt is not None else 0.0,
    }
    if det is None:
        return ev
    c = det.find("contact")
    if c is not None:
        ev["callsign"] = c.get("callsign")
    tr = det.find("track")
    if tr is not None:
        ev["course"] = float(tr.get("course", 0) or 0)
        ev["speed"] = float(tr.get("speed", 0) or 0)
    st = det.find("status")
    if st is not None and st.get("battery") is not None:
        ev["battery"] = float(st.get("battery"))
    rm = det.find("remarks")
    if rm is not None and rm.text:
        ev["remarks"] = rm.text
    col = det.find("color")
    if col is not None and col.get("argb") is not None:
        ev["color"] = int(col.get("argb"))
    for tag in ("strokeColor", "fillColor"):
        e = det.find(tag)
        if e is not None and e.get("value") is not None:
            ev[tag] = int(e.get("value"))
    poly, links = [], []
    for ln in det.findall("link"):
        if ln.get("point"):
            la, lo = ln.get("point").split(",")[:2]
            poly.append([float(la), float(lo)])
        elif ln.get("uid"):
            links.append({"uid": ln.get("uid"),
                          "relation": ln.get("relation")})
    if poly:
        ev["polygon"] = poly
    if links:
        ev["links"] = links
    ch = det.find("__chat")
    if ch is not None:
        ev["chat"] = {"sender": ch.get("senderCallsign"),
                      "room": ch.get("chatroom"),
                      "text": ev.get("remarks", "")}
    return ev


_AGL = re.compile(r"(-?\d+(?:\.\d+)?) m AGL")
_SAR = re.compile(r"SAR: ([^|]+)")
_MODE = re.compile(r"^\w+ \| ([A-Z_]+)")


class Picture:
    """Thread-safe live picture built from CoT events."""

    def __init__(self, now=time.time):
        self._now = now
        self._lock = threading.Lock()
        self.items = {}      # uid -> event dict (+ trail / footprints)
        self.feed = []       # [{'t', 'kind', 'text'}], newest last
        self.events_rx = 0
        self.started = now()
        self._sar_state = {}
        self.first_track = None

    def _log(self, kind, text, t=None):
        self.feed.append({"t": t or self._now(), "kind": kind, "text": text})
        del self.feed[:-FEED_MAX]

    def ingest(self, data):
        ev = parse_cot(data)
        if ev is None or not ev.get("uid"):
            return None
        with self._lock:
            self.events_rx += 1
            t = ev["type"]
            if t == TYPE_DELETE:
                for ln in ev.get("links", []):
                    if self.items.pop(ln["uid"], None) is not None:
                        self._log("delete", f"removed {ln['uid']}")
                return ev
            if t == TYPE_CHAT and "chat" in ev:
                if ev["uid"] not in self.items:
                    self._log("chat", f"{ev['chat']['sender']}: "
                                      f"{ev['chat']['text']}")
                self.items[ev["uid"]] = ev
                return ev
            prev = self.items.get(ev["uid"])
            if t.startswith("a-"):
                self._track(ev, prev)
            elif prev is None:
                if t.startswith("b-r-f-h-c"):
                    self._log("alert", f"CASEVAC point '{ev.get('callsign')}'"
                                       f" at {ev['lat']:.6f}, {ev['lon']:.6f}")
                elif t == "u-d-r" and "-cell-" not in ev["uid"]:
                    self._log("info", f"area '{ev.get('callsign')}' received")
            self.items[ev["uid"]] = ev
            return ev

    def _track(self, ev, prev):
        rem = ev.get("remarks", "")
        m = _AGL.search(rem)
        ev["agl"] = float(m.group(1)) if m else None
        m = _SAR.search(rem)
        ev["sar"] = m.group(1).strip() if m else None
        m = _MODE.search(rem)
        ev["mode"] = m.group(1) if m else None
        trail = prev.get("trail", []) if prev else []
        fps = prev.get("footprints", []) if prev else []
        pt = [round(ev["lat"], 7), round(ev["lon"], 7)]
        if not trail or trail[-1] != pt:
            trail.append(pt)
            del trail[:-TRAIL_MAX]
        # Camera coverage: drop a footprint every FOOTPRINT_STEP_M while the
        # drone is actually searching at altitude.
        # agl window: the bridge reports AMSL-minus-home, which reads ~1586 m
        # for a moment before the home altitude arrives - one such fix would
        # paint a 2 km "footprint" over everything.
        searching = re.match(r"(searching wp|confirming)", ev["sar"] or "")
        if searching and MIN_FOOTPRINT_AGL < (ev["agl"] or 0) < MAX_FOOTPRINT_AGL:
            last = fps[-1] if fps else None
            if last is None or _dist_m(last["lat"], last["lon"],
                                       ev["lat"], ev["lon"]) >= FOOTPRINT_STEP_M:
                fps.append({"lat": ev["lat"], "lon": ev["lon"],
                            "agl": ev["agl"], "course": ev.get("course", 0)})
        ev["trail"], ev["footprints"] = trail, fps
        name = ev.get("callsign") or ev["uid"]
        if prev is None:
            self._log("info", f"{name} joined the picture")
            if self.first_track is None:
                self.first_track = self._now()
        old = self._sar_state.get(ev["uid"])
        new = ev["sar"]
        if new and old and new.split(",")[0].split(" wp")[0] != \
                old.split(",")[0].split(" wp")[0]:
            kind = "alert" if new.startswith(("calling", "confirming")) \
                else "info"
            self._log(kind, f"{name}: {new}")
        if new:
            self._sar_state[ev["uid"]] = new

    def snapshot(self):
        now = self._now()
        with self._lock:
            items = [dict(v, is_stale=bool(v.get("stale") and v["stale"] < now))
                     for v in self.items.values()]
            return {"now": now, "uptime": now - self.started,
                    "events_rx": self.events_rx, "items": items,
                    "first_track": self.first_track,
                    "feed": list(self.feed)}


# ------------------------------------------------------------- receivers

def _pytak():
    import warnings
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        import pytak
    return pytak


async def _pytak_rx(url, picture, log):
    """Receive CoT on url with PyTAK; reconnect on error."""
    pytak = _pytak()
    from configparser import ConfigParser
    u = urlparse(url)
    cfg = ConfigParser(interpolation=None)
    cfg["pytak"] = {"COT_URL": f"{u.scheme}+ro://{u.hostname}:{u.port}"}
    while True:
        try:
            reader, _ = await pytak.protocol_factory(cfg["pytak"])
            log(f"[viewer] PyTAK listening on {url}")
            while True:
                data, _addr = await reader.recv()
                picture.ingest(data)
        except asyncio.CancelledError:
            raise
        except Exception as e:
            log(f"[viewer] PyTAK rx {url} failed: {e}; retry in 3s")
            await asyncio.sleep(3)


def _socket_rx(url, picture, log):
    """stdlib fallback receiver (unicast or multicast UDP)."""
    u = urlparse(url)
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    except (AttributeError, OSError):
        pass
    mcast = int(u.hostname.split(".")[0]) in range(224, 240)
    s.bind((u.hostname, u.port))
    if mcast:
        s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                     struct.pack("4sl", socket.inet_aton(u.hostname),
                                 socket.INADDR_ANY))
    log(f"[viewer] stdlib socket listening on {url}")
    while True:
        data = s.recv(65535)
        picture.ingest(data)


def start_receivers(urls, picture, transport="auto", log=print):
    use_pytak = transport == "pytak"
    if transport == "auto":
        try:
            _pytak()
            use_pytak = True
        except ImportError:
            use_pytak = False
    if use_pytak:
        loop = asyncio.new_event_loop()

        def run():
            asyncio.set_event_loop(loop)
            loop.run_until_complete(asyncio.gather(
                *(_pytak_rx(u, picture, log) for u in urls)))
        threading.Thread(target=run, daemon=True, name="pytak-rx").start()
    else:
        for u in urls:
            threading.Thread(target=_socket_rx, args=(u, picture, log),
                             daemon=True).start()
    return "pytak" if use_pytak else "stdlib"


# ------------------------------------------------------------------ http

def make_handler(picture, page_path, logs_dir, meta):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, code, body, ctype):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = urlparse(self.path).path
            if path in ("/", "/index.html"):
                with open(page_path, "rb") as f:
                    return self._send(200, f.read(), "text/html; charset=utf-8")
            if path == "/api/state":
                snap = picture.snapshot()
                snap["meta"] = meta
                return self._send(200, json.dumps(snap).encode(),
                                  "application/json")
            if path.startswith("/logs/") and logs_dir:
                rel = unquote(path[len("/logs/"):])
                full = os.path.realpath(os.path.join(logs_dir, rel))
                root = os.path.realpath(logs_dir)
                if (full.startswith(root + os.sep) and full.endswith(".jpg")
                        and os.path.isfile(full)):
                    with open(full, "rb") as f:
                        return self._send(200, f.read(), "image/jpeg")
            self._send(404, b"not found", "text/plain")
    return Handler


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--listen", nargs="+",
                    default=(os.environ.get("TAK_VIEWER_LISTEN")
                             or "udp://239.2.3.1:6969").split(),
                    help="CoT feeds to subscribe to (udp://, multicast ok)")
    ap.add_argument("--port", type=int,
                    default=int(os.environ.get("TAK_VIEWER_PORT", "8088")))
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--logs", default=None,
                    help="logs dir; serves CASEVAC confirm images")
    ap.add_argument("--transport", choices=("auto", "pytak", "stdlib"),
                    default=os.environ.get("TAK_TRANSPORT", "auto"))
    ap.add_argument("--title", default=os.environ.get(
        "TAK_VIEWER_TITLE", "Swarm Search & Rescue"))
    args = ap.parse_args(argv)

    page = next((p for p in PAGE_CANDIDATES if os.path.isfile(p)), None)
    if page is None:
        raise SystemExit("viewer/index.html not found next to tak_viewer.py")
    log = lambda m: print(m, flush=True)  # noqa: E731
    picture = Picture()
    rx = start_receivers(args.listen, picture, args.transport, log)
    meta = {"title": args.title, "listen": args.listen, "transport": rx}
    srv = ThreadingHTTPServer((args.bind, args.port),
                              make_handler(picture, page, args.logs, meta))
    log(f"[viewer] http://localhost:{args.port}  (CoT via {rx}: "
        f"{' '.join(args.listen)})")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
