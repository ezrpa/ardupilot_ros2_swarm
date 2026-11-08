#!/usr/bin/env python3
"""TAK bridge: publish the swarm's picture to ATAK / WinTAK / TAK Server.

Outbound Cursor-on-Target (CoT) feed, built from what the swarm already
produces — no changes to the drone nodes:

  * LIVE DRONE TRACKS (PLI, --rate Hz): one rotary-UAV marker per drone with
    callsign MAPPER-<id>/SPRAYER-<id>, course/speed, battery, flight mode,
    AGL, and (optionally) a video button for the mapper camera feed.
    Source: ArduPilot's native DDS topics, subscribed per drone domain
    (ROS_DOMAIN_ID = instance + 1) via one rclpy Context per domain:
      /ap/geopose/filtered           position (AMSL altitude)
      /ap/status                     armed / mode / flying
      /ap/battery                    battery percentage
      /ap/gps_global_origin/filtered home altitude -> AGL
    Using DDS (not MAVLink) avoids contending with QGC for UDP 14550+i and
    with the sprayer node / spray visualizer for SERIAL1/2 TCP.

  * WEED DETECTIONS (live during the survey): magenta spot markers from the
    mappers' sidecars, logs/drone_<id>/wp_*.json -> weed_detections[].
  * SPRAY TARGETS: once prescription_planner.py writes
    logs/prescriptions/sprayer_<i>.json, the raw detections are deleted from
    the EUDs and replaced by the clustered targets, colour-coded per sprayer.
  * FIELD BOUNDARY: the surveyed rectangle, with mission progress in remarks.

Static items (markers, field) are re-broadcast every --static-period s so
late-joining EUDs on multicast still receive them.

TAK being unreachable never affects the mission: every send is best-effort.

Usage (normally started by compose_decentralized.launch.py when TAK_URL set):
    tak_bridge.py --url udp://239.2.3.1:6969 --instances 0 1 2 3 --sprayers 2 3
    tak_bridge.py --url tls://tak.example:8089 --cert c.pem --key k.pem ...
    tak_bridge.py --no-telemetry --logs ./logs --url udp://239.2.3.1:6969
        (host-side: replay a finished mission's detections/plan only)
"""
import argparse
import glob
import json
import os
import threading
import time

try:
    from tak_cot import (CotSender, drone_event, spot_event, rectangle_event,
                         delete_event, ne_to_latlon, course_speed, MAGENTA,
                         SPRAYER_PALETTE)
except ImportError:  # installed as the `scripts` python package
    from scripts.tak_cot import (CotSender, drone_event, spot_event,
                                 rectangle_event, delete_event, ne_to_latlon,
                                 course_speed, MAGENTA, SPRAYER_PALETTE)

# Must match compose_decentralized.launch.py / start_drone_container.sh
FIELD_ORIGIN_LAT = 40.072842
FIELD_ORIGIN_LON = -105.230575
# Geoid undulation N at the field origin (EGM96, Boulder CO ~ -17 m).
# CoT hae = AMSL + N. Site-specific: change it with the field origin.
DEFAULT_GEOID_OFFSET = -17.0

MODE_NAMES = {0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO",
              4: "GUIDED", 5: "LOITER", 6: "RTL", 7: "CIRCLE", 9: "LAND",
              11: "DRIFT", 13: "SPORT", 14: "FLIP", 15: "AUTOTUNE",
              16: "POSHOLD", 17: "BRAKE", 20: "GUIDED_NOGPS", 21: "SMART_RTL"}

TRACK_STALE_S = 10.0     # drone marker greys out this long after last fix
FIX_TIMEOUT_S = 5.0      # stop reporting a drone whose fixes stopped


class SwarmTakBridge:
    """ROS-free core: holds swarm state, renders CoT. Unit-testable."""

    def __init__(self, instances, sprayers=(), logs_dir="/root/logs",
                 uid_prefix="ardupilot-swarm", geoid_offset=DEFAULT_GEOID_OFFSET,
                 field_half_m=40.0, video_url=None, replace_detections=True,
                 origin=(FIELD_ORIGIN_LAT, FIELD_ORIGIN_LON)):
        self.instances = list(instances)
        self.sprayers = set(sprayers)
        self.logs_dir = logs_dir
        self.prefix = uid_prefix
        self.geoid = geoid_offset
        self.field_half = field_half_m
        self.video_url = video_url
        self.replace_detections = replace_detections
        self.origin = origin
        self._lock = threading.Lock()
        self.drones = {i: {} for i in self.instances}
        self.detections = {}       # uid -> xml-builder kwargs
        self.targets = {}          # uid -> kwargs
        self._sidecar_mtime = {}   # path -> mtime
        self._plan_mtime = {}      # path -> mtime
        self._frames = 0
        self._plan_ready = False
        self._field_dirty = True

    # --------------------------------------------------------- identity
    def role(self, inst):
        return "sprayer" if inst in self.sprayers else "mapper"

    def callsign(self, inst):
        return f"{self.role(inst).upper()}-{inst + 1}"

    def drone_uid(self, inst):
        return f"{self.prefix}-drone-{inst}"

    # ---------------------------------------------------- telemetry in
    def update_pose(self, inst, lat, lon, alt_amsl, t=None):
        t = time.time() if t is None else t
        with self._lock:
            d = self.drones.setdefault(inst, {})
            d["fix"] = (lat, lon, alt_amsl, t)
            # Course/speed from fixes >= 0.5 s apart (DDS pose arrives at
            # tens of Hz; tiny deltas would make course jitter wildly).
            anchor = d.get("anchor")
            if anchor is None:
                d["anchor"] = (lat, lon, t)
            elif t - anchor[2] >= 0.5:
                cs = course_speed(anchor[0], anchor[1], anchor[2], lat, lon, t)
                if cs is not None:
                    course, speed = cs
                    # hold the last course while hovering (< 0.3 m/s)
                    if speed < 0.3:
                        course = d.get("course", 0.0)
                    d["course"], d["speed"] = course, speed
                d["anchor"] = (lat, lon, t)

    def update_status(self, inst, armed=None, mode=None, flying=None):
        with self._lock:
            d = self.drones.setdefault(inst, {})
            if armed is not None:
                d["armed"] = bool(armed)
            if mode is not None:
                d["mode"] = int(mode)
            if flying is not None:
                d["flying"] = bool(flying)

    def update_battery(self, inst, percentage):
        if percentage is None or percentage != percentage or percentage < 0:
            return  # NaN / unknown
        pct = percentage * 100.0 if percentage <= 1.0 else percentage
        with self._lock:
            self.drones.setdefault(inst, {})["battery"] = pct

    def update_home(self, inst, alt_amsl):
        with self._lock:
            self.drones.setdefault(inst, {})["home_amsl"] = alt_amsl

    # ------------------------------------------------------- CoT out
    def drone_events(self, now=None):
        now = time.time() if now is None else now
        out = []
        with self._lock:
            for inst in self.instances:
                d = self.drones.get(inst, {})
                fix = d.get("fix")
                if fix is None or now - fix[3] > FIX_TIMEOUT_S:
                    continue  # let the last marker go stale on the EUD
                lat, lon, amsl, _ = fix
                parts = [self.role(inst)]
                if "mode" in d:
                    parts.append(MODE_NAMES.get(d["mode"], f"mode {d['mode']}"))
                if "armed" in d:
                    parts.append("ARMED" if d["armed"] else "disarmed")
                if "home_amsl" in d:
                    parts.append(f"{amsl - d['home_amsl']:.1f} m AGL")
                if "speed" in d:
                    parts.append(f"{d['speed']:.1f} m/s")
                video = None
                if self.video_url and self.role(inst) == "mapper":
                    video = self.video_url.format(instance=inst, id=inst + 1)
                out.append(drone_event(
                    self.drone_uid(inst), self.callsign(inst), lat, lon,
                    hae=amsl + self.geoid, course=d.get("course"),
                    speed=d.get("speed"), battery=d.get("battery"),
                    remarks=" | ".join(parts), video_url=video,
                    stale_s=TRACK_STALE_S, now=now))
        return out

    def field_corners(self):
        h = self.field_half
        lat0, lon0 = self.origin
        return [ne_to_latlon(lat0, lon0, n, e)
                for n, e in ((h, -h), (h, h), (-h, h), (-h, -h))]

    def field_event(self, now=None):
        if self._plan_ready:
            status = f"plan ready: {len(self.targets)} spray targets"
        else:
            status = (f"surveying: {self._frames} frames, "
                      f"{len(self.detections)} weed detections")
        return rectangle_event(
            f"{self.prefix}-field", "SURVEY FIELD", self.field_corners(),
            remarks=f"{2 * self.field_half:.0f}x{2 * self.field_half:.0f} m "
                    f"survey area | {status}", now=now)

    def static_events(self, now=None):
        """Everything that must survive late EUD joins (re-broadcast)."""
        out = [self.field_event(now)]
        with self._lock:
            items = (list(self.targets.items()) if self._plan_ready
                     else list(self.detections.items()))
        for uid, kw in items:
            out.append(spot_event(uid, now=now, **kw))
        return out

    # ------------------------------------------------ mission artifacts
    def scan_artifacts(self, now=None):
        """Pick up new sidecars / prescriptions. Returns CoT to send now."""
        out = []
        out += self._scan_sidecars(now)
        out += self._scan_prescriptions(now)
        if self._field_dirty:
            out.append(self.field_event(now))
            self._field_dirty = False
        return out

    def _scan_sidecars(self, now):
        out = []
        for path in sorted(glob.glob(
                os.path.join(self.logs_dir, "drone_*", "wp_*.json"))):
            try:
                mtime = os.path.getmtime(path)
            except OSError:
                continue
            if self._sidecar_mtime.get(path) == mtime:
                continue
            try:
                with open(path) as f:
                    meta = json.load(f)
            except (json.JSONDecodeError, OSError):
                continue  # mid-write; next scan gets it
            if path not in self._sidecar_mtime:
                self._frames += 1
                self._field_dirty = True
            self._sidecar_mtime[path] = mtime
            drone_id = meta.get("drone_id")
            wp = meta.get("wp_index")
            for k, det in enumerate(meta.get("weed_detections") or []):
                if det.get("lat") is None or det.get("lon") is None:
                    continue
                uid = f"{self.prefix}-weed-d{drone_id}-wp{wp}-{k}"
                kw = dict(callsign=f"weed {drone_id}.{wp}.{k}",
                          lat=det["lat"], lon=det["lon"], color=MAGENTA,
                          remarks=f"Weed detected by MAPPER-{drone_id} at "
                                  f"WP {wp} (area {det.get('area_px', 0):.0f} "
                                  f"px, nadir georef)")
                with self._lock:
                    is_new = uid not in self.detections
                    self.detections[uid] = kw
                if is_new:
                    self._field_dirty = True
                if not self._plan_ready:
                    out.append(spot_event(uid, now=now, **kw))
        return out

    def _scan_prescriptions(self, now):
        paths = sorted(glob.glob(os.path.join(
            self.logs_dir, "prescriptions", "sprayer_*.json")))
        changed = False
        for p in paths:
            try:
                m = os.path.getmtime(p)
            except OSError:
                continue
            if self._plan_mtime.get(p) != m:
                self._plan_mtime[p] = m
                changed = True
        if not changed:
            return []

        new_targets = {}
        for p in paths:
            try:
                with open(p) as f:
                    plan = json.load(f)
            except (json.JSONDecodeError, OSError):
                continue
            inst = int(plan.get("sprayer_instance", -1))
            order = sorted(self.sprayers) or [inst]
            color = SPRAYER_PALETTE[
                (order.index(inst) if inst in order else 0)
                % len(SPRAYER_PALETTE)]
            clusters = plan.get("clusters") or []
            for k, c in enumerate(clusters):
                if c.get("lat") is None or c.get("lon") is None:
                    continue
                uid = f"{self.prefix}-target-s{inst}-{k}"
                new_targets[uid] = dict(
                    callsign=f"S{inst + 1}-T{k + 1}", lat=c["lat"],
                    lon=c["lon"], color=color,
                    parent_uid=self.drone_uid(inst),
                    remarks=f"Spray target {k + 1}/{len(clusters)} for "
                            f"SPRAYER-{inst + 1} | "
                            f"{c.get('num_detections', '?')} detections "
                            f"clustered")

        out = []
        with self._lock:
            # Remove superseded targets (plan regenerated with fewer items)
            for uid in set(self.targets) - set(new_targets):
                out.append(delete_event(uid, now=now))
            first_plan = not self._plan_ready
            self.targets = new_targets
            self._plan_ready = True
            dets = list(self.detections)
        if first_plan and self.replace_detections:
            out += [delete_event(uid, now=now) for uid in dets]
        out += [spot_event(uid, now=now, **kw)
                for uid, kw in new_targets.items()]
        self._field_dirty = True
        return out


# ------------------------------------------------------------- ROS input

def start_ros_sources(bridge, log=print):
    """Subscribe to each drone's ArduPilot DDS topics in its own domain."""
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from geographic_msgs.msg import GeoPoseStamped, GeoPointStamped
    from sensor_msgs.msg import BatteryState
    from ardupilot_msgs.msg import Status

    qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    contexts = []
    for inst in bridge.instances:
        ctx = rclpy.Context()
        rclpy.init(context=ctx, domain_id=inst + 1)
        node = rclpy.create_node(f"tak_bridge_{inst}", context=ctx)

        def on_pose(msg, i=inst):
            p = msg.pose.position
            bridge.update_pose(i, p.latitude, p.longitude, p.altitude)

        def on_status(msg, i=inst):
            bridge.update_status(i, armed=getattr(msg, "armed", None),
                                 mode=getattr(msg, "mode", None),
                                 flying=getattr(msg, "flying", None))

        def on_batt(msg, i=inst):
            bridge.update_battery(i, msg.percentage)

        def on_origin(msg, i=inst):
            bridge.update_home(i, msg.position.altitude)

        node.create_subscription(GeoPoseStamped, "/ap/geopose/filtered",
                                 on_pose, qos)
        node.create_subscription(Status, "/ap/status", on_status, qos)
        node.create_subscription(BatteryState, "/ap/battery", on_batt, qos)
        node.create_subscription(GeoPointStamped,
                                 "/ap/gps_global_origin/filtered",
                                 on_origin, qos)
        ex = SingleThreadedExecutor(context=ctx)
        ex.add_node(node)
        threading.Thread(target=ex.spin, daemon=True,
                         name=f"tak-dds-{inst}").start()
        contexts.append((ctx, node, ex))
        log(f"[tak] subscribed to drone instance {inst} (domain {inst + 1})")
    return contexts


# -------------------------------------------------------------------- main

def _env(name, default=None):
    v = os.environ.get(name)
    return v if v not in (None, "") else default


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--url", nargs="+",
                    default=(_env("TAK_URL") or "udp://239.2.3.1:6969").split(),
                    help="One or more CoT sinks: udp:// tcp:// tls:// file:// "
                         "(env TAK_URL, space-separated)")
    ap.add_argument("--instances", nargs="+", type=int,
                    help="0-indexed drone instances (default: 0..NUM_DRONES-1)")
    ap.add_argument("--sprayers", nargs="*", type=int, default=None,
                    help="sprayer instances (default: env SPRAYER_INSTANCES)")
    ap.add_argument("--logs", default="/root/logs")
    ap.add_argument("--rate", type=float, default=1.0,
                    help="drone track update rate, Hz")
    ap.add_argument("--scan-period", type=float, default=3.0)
    ap.add_argument("--static-period", type=float, default=30.0)
    ap.add_argument("--uid-prefix", default=_env("TAK_UID_PREFIX",
                                                 "ardupilot-swarm"))
    ap.add_argument("--geoid-offset", type=float,
                    default=float(_env("TAK_GEOID_OFFSET",
                                       DEFAULT_GEOID_OFFSET)))
    ap.add_argument("--field-half", type=float, default=40.0,
                    help="half-width of the surveyed square, m")
    ap.add_argument("--video-url", default=_env("TAK_VIDEO_URL"),
                    help="mapper video URL template, e.g. "
                         "rtsp://10.0.0.2:8554/drone{instance}")
    ap.add_argument("--keep-detections", action="store_true",
                    help="keep raw detection markers after the plan lands")
    ap.add_argument("--cert", default=_env("TAK_CERT"),
                    help="client cert PEM (tls://)")
    ap.add_argument("--key", default=_env("TAK_KEY"),
                    help="client key PEM (tls://)")
    ap.add_argument("--key-password", default=_env("TAK_KEY_PASSWORD"))
    ap.add_argument("--ca", default=_env("TAK_CA"),
                    help="TAK Server CA PEM; omit = encrypt without verifying")
    ap.add_argument("--no-telemetry", action="store_true",
                    help="skip ROS/DDS drone tracks (artifacts only)")
    args = ap.parse_args(argv)
    if args.instances is None:
        args.instances = list(range(int(_env("NUM_DRONES", "2"))))
    if args.sprayers is None:
        args.sprayers = [int(s) for s in
                         (_env("SPRAYER_INSTANCES", "") or "").split()]
    return args


def main(argv=None):
    args = parse_args(argv)
    log = lambda m: print(m, flush=True)  # noqa: E731
    sinks = [CotSender(u, certfile=args.cert, keyfile=args.key,
                       cafile=args.ca, key_password=args.key_password,
                       logger=log) for u in args.url]
    bridge = SwarmTakBridge(
        args.instances, args.sprayers, logs_dir=args.logs,
        uid_prefix=args.uid_prefix, geoid_offset=args.geoid_offset,
        field_half_m=args.field_half, video_url=args.video_url,
        replace_detections=not args.keep_detections)

    def send_all(events):
        for xml in events:
            for s in sinks:
                s.send(xml)

    if not args.no_telemetry:
        try:
            start_ros_sources(bridge, log)
        except Exception as e:  # ROS missing / DDS down: artifacts still flow
            log(f"[tak] drone telemetry unavailable ({e}); "
                f"publishing mission artifacts only")

    log(f"[tak] bridge up: sinks={args.url} instances={args.instances} "
        f"sprayers={args.sprayers} logs={args.logs}")
    next_scan = next_static = 0.0
    period = 1.0 / max(args.rate, 0.1)
    try:
        while True:
            now = time.time()
            send_all(bridge.drone_events(now))
            if now >= next_scan:
                send_all(bridge.scan_artifacts(now))
                next_scan = now + args.scan_period
            if now >= next_static:
                send_all(bridge.static_events(now))
                next_static = now + args.static_period
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        for s in sinks:
            s.close()


if __name__ == "__main__":
    main()
