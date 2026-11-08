#!/usr/bin/env python3
"""The swarm's deployment contract: the ONE place that defines the field
origin, per-instance ports / ROS domain / MAVLink sysid, the camera
intrinsics and the spawn layout.

Everything else reads these values instead of re-deriving them. Python
imports this module. Shell scripts (start_swarm.sh on the host,
start_drone_container.sh in the ArduPilot container, where this directory is
mounted read-only at /root/swarm_core) evaluate its env view:

    eval "$(python3 config.py env --instance 2 --gazebo 1)"

Stdlib only, so it also runs where the swarm_control package is not
installed. Values the simulator assets bake in (the camera in
base_drone_cam*/model.sdf, the FDM port in base_drone*/model.sdf) cannot read
this module; tests/test_contracts.py checks them against it instead.
"""
import argparse
import sys
from typing import Dict, List, Optional, Sequence, Tuple, Union

if __package__:
    from . import geo
else:  # run as a script (e.g. /root/swarm_core/config.py in a container)
    import geo  # type: ignore[no-redef,import-not-found]

# ------------------------------------------------------------ field / home
# World (0, 0) in Gazebo; every drone's GPS home in Gazebo mode; centre of
# the surveyed field and the SAR search area.
FIELD_ORIGIN_LAT = 40.072842
FIELD_ORIGIN_LON = -105.230575
HOME_ALT_AMSL_M = 1586           # SITL --custom-location altitude

# ------------------------------------------------------------------ camera
# Down-facing camera of base_drone_cam*/model.sdf (pitch 90 deg, body-fixed).
CAMERA_HFOV_RAD = 1.2
CAMERA_WIDTH_PX = 640
CAMERA_HEIGHT_PX = 480

# ----------------------------------------------------------- spawn layout
DEFAULT_SPAWN_AREA_M = 120.0     # SAR search square side
SPAWN_INSET_M = 8.0              # launch points sit this far inside the square
BUILTIN_SITL_SPACING_DEG = 0.000045   # ~5 m between homes with built-in physics
SPAWN_LAYOUTS = ("", "corners")


# ------------------------------------------------- per-instance contract
# `instance` is 0-based everywhere.

def sysid(instance: int) -> int:
    """MAVLink SYSID_THISMAV."""
    return instance + 1


def ros_domain_id(instance: int) -> int:
    """Each drone lives in its own ROS 2 domain (DDS_DOMAIN_ID too)."""
    return instance + 1


def dds_udp_port(instance: int) -> int:
    """ArduPilot DDS client -> micro-ROS agent (DDS_UDP_PORT)."""
    return 2019 + instance


def mavlink_tcp_port(instance: int) -> int:
    """SITL SERIAL0 TCP: ground stations (QGroundControl)."""
    return 5760 + 10 * instance


def serial1_tcp_port(instance: int) -> int:
    """SITL SERIAL1 TCP: SprayerDrone's MAVLink link (sprayer commands)."""
    return 5762 + 10 * instance


def serial2_tcp_port(instance: int) -> int:
    """SITL SERIAL2 TCP: spray_visualizer (pump servo readback)."""
    return 5763 + 10 * instance


def sitl_port(instance: int) -> int:
    """SITL base port (RC input) for this instance."""
    return 5501 + 10 * instance


def mavproxy_udp_port(instance: int) -> int:
    """MAVProxy UDP output for host tools (pymavlink, QGC)."""
    return 14550 + instance


def gazebo_fdm_port(instance: int) -> int:
    """Gazebo ArduPilotPlugin JSON FDM port baked into base_drone*/model.sdf."""
    return 9002 + 10 * instance


def camera_udp_port(instance: int) -> int:
    """GstCameraPlugin H.264/RTP stream of base_drone_cam*/model.sdf."""
    return 5600 + instance


def launch_points(n_drones: int, size_m: float = DEFAULT_SPAWN_AREA_M,
                  inset_m: float = SPAWN_INSET_M) -> List[Tuple[float, float]]:
    """SAR separate launch points as (north, east) metres from the field
    origin: the search square's corners (NE, SW, NW, SE), then the edge
    midpoints; instances beyond 8 wrap around."""
    h = size_m / 2.0 - inset_m
    pts = [(h, h), (-h, -h), (h, -h), (-h, h),
           (0.0, h), (0.0, -h), (h, 0.0), (-h, 0.0)]
    return [pts[i % len(pts)] for i in range(n_drones)]


def home_location(instance: int, gazebo: bool = False, layout: str = "",
                  area_m: float = DEFAULT_SPAWN_AREA_M) -> Tuple[str, str]:
    """SITL --custom-location lat/lon for an instance, as the strings
    start_drone_container.sh passes on.

    Gazebo: every drone uses the field origin (physical separation comes
    from the world spawn poses; SITL GPS = home + world position).
    layout="corners": the SAR launch points. Otherwise built-in physics,
    homes ~5 m apart so airframes don't stack.
    """
    if layout not in SPAWN_LAYOUTS:
        raise ValueError(f"unknown spawn layout {layout!r}")
    if gazebo:
        return f"{FIELD_ORIGIN_LAT}", f"{FIELD_ORIGIN_LON}"
    if layout == "corners":
        n, e = launch_points(instance + 1, area_m)[instance]
        lat, lon = geo.offset_latlon(FIELD_ORIGIN_LAT, FIELD_ORIGIN_LON, n, e)
        return f"{lat:.7f}", f"{lon:.7f}"
    return (f"{FIELD_ORIGIN_LAT + instance * BUILTIN_SITL_SPACING_DEG:.6f}",
            f"{FIELD_ORIGIN_LON + instance * BUILTIN_SITL_SPACING_DEG:.6f}")


def instance_env(instance: int, gazebo: bool = False, layout: str = "",
                 area_m: float = DEFAULT_SPAWN_AREA_M) -> Dict[str, Union[int, str]]:
    """Shell variables for one drone instance (names used by the scripts)."""
    lat, lon = home_location(instance, gazebo, layout, area_m)
    return {
        "SYSID_THISMAV": sysid(instance),
        "ROS_DOMAIN_ID": ros_domain_id(instance),
        "MICRO_ROS_AGENT_PORT": dds_udp_port(instance),
        "DDS_UDP_PORT": dds_udp_port(instance),
        "MAVLINK_TCP_PORT": mavlink_tcp_port(instance),
        "SITL_PORT": sitl_port(instance),
        "MAVPROXY_UDP_PORT": mavproxy_udp_port(instance),
        "GAZEBO_JSON_PORT": gazebo_fdm_port(instance),
        "LAT_OFFSET": lat,
        "LON_OFFSET": lon,
        "ALT_BASE": HOME_ALT_AMSL_M,
    }


def _main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    env = sub.add_parser("env", help="print KEY=value lines for one instance")
    env.add_argument("--instance", type=int, required=True)
    env.add_argument("--gazebo", default="0", help="1 = Gazebo JSON physics")
    env.add_argument("--layout", default="", choices=SPAWN_LAYOUTS)
    env.add_argument("--area", type=float, default=DEFAULT_SPAWN_AREA_M)
    args = ap.parse_args(argv)
    if args.instance < 0:
        ap.error("--instance must be >= 0")
    for k, v in instance_env(args.instance, args.gazebo == "1", args.layout,
                             args.area).items():
        print(f"{k}={v}")
    return 0


if __name__ == "__main__":
    sys.exit(_main())
