#!/usr/bin/env python3
"""Regressions for the two defects that invalidated a whole survey run:

  1. The mission loop advanced on a fixed 10 s timer, so a node respawned onto
     a landed aircraft "reached" every waypoint and captured 32 of 33 weed
     detections from a drone parked on the ground.
  2. Captures were geotagged with the COMMANDED waypoint and georeferenced as
     if the camera were north-aligned, though it is body-fixed.

Run directly:
    python3 tests/test_mission_integrity.py
"""
import math
import os
import sys

import cv2
import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "src", "swarm_control", "scripts"))
from camera_processor import CameraProcessor, CAMERA_HFOV_RAD  # noqa: E402

CP = CameraProcessor(0, 0)  # udp_port=0 -> no stream, detection still usable

GREEN_BGR = (40, 120, 60)
MAGENTA_BGR = (255, 0, 255)
LAT, LON, ALT = 40.072842, -105.230575, 30.0
M_PER_DEG_LAT = 111111.0


def _frame_with_blob_at(px, py, radius=14):
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    img[:] = GREEN_BGR
    cv2.circle(img, (px, py), radius, MAGENTA_BGR, -1)
    return img


def _offset_m(det):
    """Detection offset from the capture point, in (north, east) metres."""
    north = (det["lat"] - LAT) * M_PER_DEG_LAT
    east = (det["lon"] - LON) * M_PER_DEG_LAT * math.cos(math.radians(LAT))
    return north, east


def _single(frame, heading):
    dets = CP._detect_weeds(frame, lat=LAT, lon=LON, alt=ALT, heading=heading)
    assert len(dets) == 1, f"expected one blob, got {len(dets)}"
    return dets[0]


# ---------------------------------------------------------------- georeference

def test_blob_ahead_maps_north_when_heading_north():
    """A blob above image centre is ahead of the nose; heading 0 -> north."""
    det = _single(_frame_with_blob_at(320, 140), heading=0.0)
    north, east = _offset_m(det)
    assert north > 5.0, f"expected a northward offset, got {north:.2f} m"
    assert abs(east) < 1.0, f"expected no east offset, got {east:.2f} m"


def test_same_blob_maps_east_when_heading_east():
    """Same pixel, vehicle turned 90 deg: the target is now due east.

    This is the regression. Without yaw compensation the detection stayed
    stubbornly north no matter which way the aircraft pointed.
    """
    det = _single(_frame_with_blob_at(320, 140), heading=math.pi / 2)
    north, east = _offset_m(det)
    assert east > 5.0, f"expected an eastward offset, got {east:.2f} m"
    assert abs(north) < 1.0, f"expected no north offset, got {north:.2f} m"


def test_reversed_heading_flips_the_offset():
    """Alternating serpentine legs fly 180 deg apart - the worst case."""
    ahead = _offset_m(_single(_frame_with_blob_at(320, 140), heading=0.0))
    behind = _offset_m(_single(_frame_with_blob_at(320, 140), heading=math.pi))
    assert ahead[0] > 0 > behind[0], (
        f"180 deg of heading must flip north: {ahead[0]:.2f} vs {behind[0]:.2f}")
    assert abs(ahead[0] + behind[0]) < 0.5, "flip should be symmetric"


def test_heading_none_keeps_north_aligned_behaviour():
    """Missing heading must degrade to the old assumption, not crash."""
    det = _single(_frame_with_blob_at(320, 140), heading=None)
    north, east = _offset_m(det)
    assert north > 5.0 and abs(east) < 1.0


def test_magnitude_matches_pinhole_geometry():
    """Offset magnitude must not change with heading - only direction does."""
    frame = _frame_with_blob_at(480, 140)
    mags = []
    for psi in (0.0, math.pi / 3, math.pi, 3 * math.pi / 2):
        n, e = _offset_m(_single(frame, heading=psi))
        mags.append(math.hypot(n, e))
    assert max(mags) - min(mags) < 0.1, f"magnitude drifted: {mags}"

    f_px = (640 / 2.0) / math.tan(CAMERA_HFOV_RAD / 2.0)
    expected = math.hypot((480 - 320) / f_px * ALT, (140 - 240) / f_px * ALT)
    assert abs(mags[0] - expected) < 0.5, f"{mags[0]:.2f} != {expected:.2f}"


# -------------------------------------------------------------------- liveness

class SkipTest(Exception):
    """Raised when a test needs ROS, which only exists inside the container."""


try:
    from base_drone_dds import BaseDrone  # needs rclpy + ardupilot_msgs
    _HAVE_ROS = True
except Exception:  # noqa: BLE001 - host has no ROS; container does
    BaseDrone = None
    _HAVE_ROS = False


class _FakeDrone:
    """Exercises the REAL liveness predicates without a ROS graph.

    The methods are borrowed off BaseDrone rather than reimplemented, so these
    tests fail if the production predicates regress.
    """

    def __init__(self, armed, alt_amsl, home_alt=1586.0):
        self.armed = armed
        self.home_alt = home_alt
        self.current_geopose = {
            'lat': LAT, 'lon': LON, 'alt_amsl': alt_amsl, 'heading': 0.0}

    def __getattr__(self, name):
        if not _HAVE_ROS:
            raise SkipTest("needs ROS (run inside swarm_controller)")
        if name in ('is_airborne', 'rel_alt', 'distance_to'):
            return getattr(BaseDrone, name).__get__(self, _FakeDrone)
        raise AttributeError(name)


def test_flying_vehicle_is_airborne():
    assert _FakeDrone(armed=True, alt_amsl=1616.0).is_airborne()


def test_disarmed_vehicle_is_not_airborne():
    """The exact state that fabricated 32 detections: armed goes false."""
    assert not _FakeDrone(armed=False, alt_amsl=1616.0).is_airborne()


def test_landed_but_still_armed_is_not_airborne():
    assert not _FakeDrone(armed=True, alt_amsl=1586.1).is_airborne()


def test_unknown_pose_is_not_airborne():
    d = _FakeDrone(armed=True, alt_amsl=1616.0)
    d.current_geopose = None
    assert not d.is_airborne()


def test_distance_to_is_metres():
    d = _FakeDrone(armed=True, alt_amsl=1616.0)
    assert d.distance_to(LAT, LON) < 0.01
    north_100m = LAT + 100.0 / 111320.0
    assert abs(d.distance_to(north_100m, LON) - 100.0) < 1.0


if __name__ == '__main__':
    mod = sys.modules[__name__]
    tests = [n for n in dir(mod) if n.startswith('test_')]
    failed = skipped = 0
    for name in sorted(tests):
        try:
            getattr(mod, name)()
            print(f"PASS {name}")
        except SkipTest as e:
            skipped += 1
            print(f"SKIP {name}: {e}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL {name}: {e}")
        except Exception as e:  # noqa: BLE001
            failed += 1
            print(f"ERROR {name}: {type(e).__name__}: {e}")
    print(f"\n{len(tests) - failed - skipped}/{len(tests)} passed, "
          f"{skipped} skipped, {failed} failed")
    sys.exit(1 if failed else 0)
