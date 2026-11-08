#!/usr/bin/env python3
"""Cursor-on-Target (CoT) encoding + transports for TAK integration.

Pure standard library (no pytak / asyncio) so it runs unchanged inside the
swarm_controller container and on the host. tak_bridge.py feeds it swarm
state; this module only knows CoT.

CoT event anatomy (what ATAK / WinTAK / iTAK / TAK Server consume):

    <event version="2.0" uid=".." type="a-f-A-M-F-Q-r" how="m-g"
           time=".." start=".." stale="..">
      <point lat=".." lon=".." hae=".." ce=".." le=".."/>
      <detail> contact / track / status / remarks / __video ... </detail>
    </event>

`hae` is height above the WGS84 ELLIPSOID, not MSL. ArduPilot reports AMSL,
so callers add the local geoid undulation N (hae = amsl + N; N is about
-17 m at the Boulder field origin). ce/le are 1-sigma errors in metres;
9999999 means "unknown".

Transports (URL style matches PyTAK / FreeTAKServer conventions):
    udp://239.2.3.1:6969   ATAK default mesh-SA multicast (no server needed)
    udp://10.0.0.5:4242    unicast to one EUD's network input
    tcp://takserver:8087   TAK Server plain streaming input
    tls://takserver:8089   TAK Server TLS streaming (client cert required)
    file:///root/logs/tak/cot.log   append one event per line (debug/replay)
"""
import math
import os
import socket
import ssl
import threading
import time
import uuid as _uuid
from datetime import datetime, timedelta, timezone
from urllib.parse import urlparse
from xml.sax.saxutils import quoteattr, escape

UNKNOWN_ERR = 9999999.0

# MIL-STD-2525 derived CoT types
TYPE_UAV_ROTARY = "a-f-A-M-F-Q-r"   # friendly / air / military / UAV, rotary
TYPE_SPOT_MAP = "b-m-p-s-m"         # spot-map marker (user-placeable point)
TYPE_RECTANGLE = "u-d-r"            # drawing: rectangle
TYPE_DELETE = "t-x-d-d"             # tasking: delete the linked uid

# ARGB colours as signed 32-bit ints (the encoding TAK uses in XML)
def argb(a, r, g, b):
    v = (a << 24) | (r << 16) | (g << 8) | b
    return v - (1 << 32) if v >= (1 << 31) else v


MAGENTA = argb(255, 255, 0, 255)
CYAN = argb(255, 0, 255, 255)
YELLOW = argb(255, 255, 255, 0)
WHITE = argb(255, 255, 255, 255)
GREEN_FILL = argb(40, 0, 200, 0)
GREEN = argb(255, 0, 200, 0)

# Distinct per-sprayer target colours (cycled by sprayer index)
SPRAYER_PALETTE = [
    argb(255, 255, 140, 0),    # orange
    argb(255, 0, 170, 255),    # azure
    argb(255, 255, 255, 0),    # yellow
    argb(255, 0, 255, 120),    # spring green
    argb(255, 255, 60, 60),    # red
    argb(255, 180, 120, 255),  # lavender
]


def cot_time(t=None):
    """CoT timestamp: ISO-8601 UTC with millisecond precision and 'Z'."""
    dt = (datetime.fromtimestamp(t, tz=timezone.utc) if t is not None
          else datetime.now(timezone.utc))
    return dt.strftime("%Y-%m-%dT%H:%M:%S.") + f"{dt.microsecond // 1000:03d}Z"


def _attrs(**kw):
    return " ".join(f"{k}={quoteattr(str(v))}" for k, v in kw.items()
                    if v is not None)


def event(uid, cot_type, lat, lon, hae=0.0, ce=UNKNOWN_ERR, le=UNKNOWN_ERR,
          stale_s=30.0, how="m-g", detail="", now=None):
    """Serialise one CoT event. `detail` is pre-rendered inner XML."""
    now = time.time() if now is None else now
    head = _attrs(version="2.0", uid=uid, type=cot_type, how=how,
                  time=cot_time(now), start=cot_time(now),
                  stale=cot_time(now + stale_s))
    pt = _attrs(lat=f"{lat:.8f}", lon=f"{lon:.8f}", hae=f"{hae:.2f}",
                ce=f"{ce:.1f}", le=f"{le:.1f}")
    return (f"<?xml version='1.0' encoding='UTF-8' standalone='yes'?>"
            f"<event {head}><point {pt}/><detail>{detail}</detail></event>")


# ---------------------------------------------------------------- builders

def drone_event(uid, callsign, lat, lon, hae, course=None, speed=None,
                battery=None, remarks="", video_url=None, stale_s=10.0,
                cot_type=TYPE_UAV_ROTARY, group=None, now=None):
    """Position report (PLI) for one swarm drone.

    course: degrees true (0 = north, clockwise); speed: m/s over ground.
    """
    d = [f"<contact {_attrs(callsign=callsign)}/>",
         "<precisionlocation altsrc=\"GPS\" geopointsrc=\"GPS\"/>",
         "<takv device=\"ArduPilot SITL\" platform=\"ardupilot_ros2_swarm\" "
         "os=\"ROS2 Humble\" version=\"1\"/>"]
    if group:
        d.append(f"<__group {_attrs(name=group, role='Team Member')}/>")
    if course is not None or speed is not None:
        d.append("<track " + _attrs(
            course=f"{(course or 0.0) % 360.0:.1f}",
            speed=f"{max(speed or 0.0, 0.0):.2f}") + "/>")
    if battery is not None:
        d.append(f"<status {_attrs(battery=int(round(battery)))}/>")
    if video_url:
        # ATAK shows a video button on the marker; the URL must be reachable
        # from the EUD (RTSP restream of the drone's RTP feed, see docs/TAK.md)
        d.append(f"<__video {_attrs(url=video_url)}/>")
    if remarks:
        d.append(f"<remarks>{escape(remarks)}</remarks>")
    return event(uid, cot_type, lat, lon, hae=hae, ce=3.0, le=5.0,
                 stale_s=stale_s, how="m-g", detail="".join(d), now=now)


def spot_event(uid, callsign, lat, lon, color=MAGENTA, remarks="",
               stale_s=86400.0, parent_uid=None, now=None):
    """Spot-map marker (weed detection / spray target). Ground level."""
    d = [f"<contact {_attrs(callsign=callsign)}/>",
         f"<color {_attrs(argb=color)}/>",
         f"<usericon {_attrs(iconsetpath=f'COT_MAPPING_SPOTMAP/{TYPE_SPOT_MAP}/{color}')}/>"]
    if parent_uid:
        d.append(f"<link {_attrs(uid=parent_uid, relation='p-p', type=TYPE_UAV_ROTARY)}/>")
    d.append("<archive/>")  # keep across EUD restarts
    if remarks:
        d.append(f"<remarks>{escape(remarks)}</remarks>")
    return event(uid, TYPE_SPOT_MAP, lat, lon, hae=0.0, ce=5.0,
                 stale_s=stale_s, how="h-g-i-g-o", detail="".join(d), now=now)


def rectangle_event(uid, callsign, corners, stroke=GREEN, fill=GREEN_FILL,
                    remarks="", stale_s=86400.0, now=None):
    """Drawn rectangle; `corners` = 4 (lat, lon) in ring order."""
    clat = sum(c[0] for c in corners) / len(corners)
    clon = sum(c[1] for c in corners) / len(corners)
    d = [f"<contact {_attrs(callsign=callsign)}/>"]
    d += [f"<link {_attrs(point=f'{la:.8f},{lo:.8f}')}/>" for la, lo in corners]
    d += [f"<strokeColor {_attrs(value=stroke)}/>",
          "<strokeWeight value=\"3.0\"/>",
          f"<fillColor {_attrs(value=fill)}/>",
          "<labels_on value=\"true\"/>", "<archive/>"]
    if remarks:
        d.append(f"<remarks>{escape(remarks)}</remarks>")
    return event(uid, TYPE_RECTANGLE, clat, clon, hae=0.0, stale_s=stale_s,
                 how="h-e", detail="".join(d), now=now)


def delete_event(target_uid, target_type=TYPE_SPOT_MAP, now=None):
    """Ask EUDs to remove a previously sent item (e.g. superseded markers)."""
    d = (f"<link {_attrs(uid=target_uid, relation='none', type=target_type)}/>"
         "<__forcedelete/>")
    return event(f"delete-{_uuid.uuid4()}", TYPE_DELETE, 0.0, 0.0,
                 stale_s=20.0, how="h-g-i-g-o", detail=d, now=now)


# ------------------------------------------------------------- geo helpers

M_PER_DEG_LAT = 111111.0  # same equirectangular constant as the planner


def ne_to_latlon(origin_lat, origin_lon, north, east):
    lat = origin_lat + north / M_PER_DEG_LAT
    lon = origin_lon + east / (M_PER_DEG_LAT * math.cos(math.radians(origin_lat)))
    return lat, lon


def course_speed(lat1, lon1, t1, lat2, lon2, t2):
    """Course (deg true) + ground speed (m/s) between two fixes, or None."""
    dt = t2 - t1
    if dt <= 0:
        return None
    dn = (lat2 - lat1) * M_PER_DEG_LAT
    de = (lon2 - lon1) * M_PER_DEG_LAT * math.cos(math.radians(lat1))
    dist = math.hypot(dn, de)
    course = math.degrees(math.atan2(de, dn)) % 360.0
    return course, dist / dt


# --------------------------------------------------------------- transport

class CotSender:
    """Thread-safe, self-reconnecting CoT sink for udp/tcp/tls URLs.

    Never raises from send(): TAK being down must not affect the swarm.
    """

    def __init__(self, url, certfile=None, keyfile=None, cafile=None,
                 key_password=None, verify_hostname=True, logger=print):
        u = urlparse(url)
        if u.scheme not in ("udp", "tcp", "tls", "ssl", "file"):
            raise ValueError(f"unsupported TAK URL scheme: {url}")
        self.path = u.path
        if u.scheme == "file":
            if not u.path:
                raise ValueError(f"file URL needs a path: {url}")
        elif not u.hostname or not u.port:
            raise ValueError(f"TAK URL needs host and port: {url}")
        self.url = url
        self.scheme = "tls" if u.scheme == "ssl" else u.scheme
        self.host, self.port = u.hostname, u.port
        self.certfile, self.keyfile, self.cafile = certfile, keyfile, cafile
        self.key_password = key_password
        self.verify_hostname = verify_hostname
        self.log = logger
        self._sock = None
        self._lock = threading.Lock()
        self._next_retry = 0.0
        self.sent = 0

    def _connect(self):
        if self.scheme == "file":
            d = os.path.dirname(self.path)
            if d:
                os.makedirs(d, exist_ok=True)
            return open(self.path, "a", encoding="utf-8")
        if self.scheme == "udp":
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 32)
            return s
        raw = socket.create_connection((self.host, self.port), timeout=5)
        raw.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        if self.scheme == "tcp":
            return raw
        ctx = ssl.create_default_context(cafile=self.cafile)
        if not self.cafile:
            # TAK Server ships with a private CA; without its truststore we
            # can only encrypt, not authenticate the server.
            ctx.check_hostname = False
            ctx.verify_mode = ssl.CERT_NONE
        else:
            ctx.check_hostname = self.verify_hostname
        if self.certfile:
            ctx.load_cert_chain(self.certfile, self.keyfile,
                                password=self.key_password)
        return ctx.wrap_socket(raw, server_hostname=self.host)

    def send(self, xml):
        data = xml.encode("utf-8")
        with self._lock:
            if self._sock is None:
                if time.time() < self._next_retry:
                    return False
                try:
                    self._sock = self._connect()
                    self.log(f"[tak] connected {self.url}")
                except (OSError, ssl.SSLError) as e:
                    self._next_retry = time.time() + 5.0
                    self.log(f"[tak] connect {self.url} failed: {e}; retry in 5s")
                    return False
            try:
                if self.scheme == "file":
                    self._sock.write(xml + "\n")
                    self._sock.flush()
                elif self.scheme == "udp":
                    self._sock.sendto(data, (self.host, self.port))
                else:
                    self._sock.sendall(data)
                self.sent += 1
                return True
            except (OSError, ssl.SSLError) as e:
                self.log(f"[tak] send failed: {e}; reconnecting")
                try:
                    self._sock.close()
                except OSError:
                    pass
                self._sock = None
                self._next_retry = time.time() + 2.0
                return False

    def close(self):
        with self._lock:
            if self._sock is not None:
                try:
                    self._sock.close()
                except OSError:
                    pass
                self._sock = None
