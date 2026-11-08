#!/usr/bin/env python3
"""TAK bridge node: publish the swarm's picture to ATAK / WinTAK / TAK Server.

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

  --mission sar (search and rescue, sar_mission.launch.py):
  * Drones are SEARCH-<id>; remarks carry each drone's SAR state
    (searching wp k/n, confirming, converging 23 m out, on scene...).
  * THE CALL (logs/sar/call.json, raised by the drone that confirms the
    missing person): a CASEVAC point (b-r-f-h-c) at the survivor, linked to
    the finder, plus ONE GeoChat broadcast to All Chat Rooms so every
    operator is paged at the same moment the drones are.
  * Search-area rectangle with coverage / call / responders-on-scene status.

Static items (markers, field) are re-broadcast every --static-period s so
late-joining EUDs on multicast still receive them.

TAK being unreachable never affects the mission: every send is best-effort.

Usage (normally started by compose_decentralized.launch.py when TAK_URL set):
    tak_bridge.py --url udp://239.2.3.1:6969 --instances 0 1 2 3 --sprayers 2 3
    tak_bridge.py --url tls://tak.example:8089 --cert c.pem --key k.pem ...
    tak_bridge.py --no-telemetry --logs ./logs --url udp://239.2.3.1:6969
        (host-side: replay a finished mission's detections/plan only)

Structure: the picture logic is ROS-free in swarm_control.tak.picture; only
this module touches rclpy. Executable name kept: lib/swarm_control/tak_bridge.py.
"""
import argparse
import os
import threading
import time

from swarm_control.tak.cot import TRANSPORTS, make_sender
from swarm_control.tak.picture import (DEFAULT_GEOID_OFFSET, SinkFanout,
                                       SwarmTakBridge)

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
    ap.add_argument("--transport", choices=TRANSPORTS,
                    default=_env("TAK_TRANSPORT", "auto"),
                    help="CoT transport: pytak, stdlib, or auto (pytak when "
                         "installed; env TAK_TRANSPORT)")
    ap.add_argument("--mission", choices=("survey", "sar"),
                    default=_env("SWARM_MISSION", "survey"))
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
    sinks = [make_sender(u, transport=args.transport, certfile=args.cert,
                         keyfile=args.key, cafile=args.ca,
                         key_password=args.key_password, logger=log)
             for u in args.url]
    bridge = SwarmTakBridge(
        args.instances, args.sprayers, logs_dir=args.logs,
        uid_prefix=args.uid_prefix, geoid_offset=args.geoid_offset,
        field_half_m=args.field_half, video_url=args.video_url,
        replace_detections=not args.keep_detections, mission=args.mission)

    fanout = SinkFanout(sinks, log=log)
    send_all = fanout.send

    if not args.no_telemetry:
        try:
            start_ros_sources(bridge, log)
        except Exception as e:  # ROS missing / DDS down: artifacts still flow
            log(f"[tak] drone telemetry unavailable ({e}); "
                f"publishing mission artifacts only")

    log(f"[tak] bridge up ({args.mission}): sinks="
        f"{[f'{u} [{type(s).__name__}]' for u, s in zip(args.url, sinks)]} "
        f"instances={args.instances} "
        f"sprayers={args.sprayers} logs={args.logs}")
    next_scan = next_static = 0.0
    period = 1.0 / max(args.rate, 0.1)
    try:
        while True:
            now = time.time()
            fanout.retry(bridge.one_shot)
            send_all(bridge.drone_events(now))
            if now >= next_scan:
                send_all(bridge.scan_artifacts(now), bridge.one_shot)
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
