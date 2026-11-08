#!/usr/bin/env python3
"""geo.py is the single local-metres <-> GPS model, and it is ArduPilot's.

SITL reports Gazebo positions as home.offset(north, east) using
AP_Math LATLON_TO_M = 0.011131884502145034 m per 1e-7 deg, so every module
(waypoints, georeferencing, planner, SAR, TAK, ground-truth generators) must
use exactly that model or positions disagree with the autopilot.

Run directly:  python3 tests/test_geo.py
"""
import pytest
import os
import re
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG_ROOT = os.path.join(REPO, "src", "swarm_control")
sys.path.insert(0, PKG_ROOT)
from swarm_control.core import config, geo  # noqa: E402

O = (config.FIELD_ORIGIN_LAT, config.FIELD_ORIGIN_LON)


def test_constant_is_ardupilot_latlon_to_m():
    # libraries/AP_Math/definitions.h: #define LATLON_TO_M 0.011131884502145034
    assert geo.M_PER_DEG_LAT == 0.011131884502145034 * 1e7


def test_offset_matches_ardupilot_location_offset():
    """Location::offset_latlng: dlat = n/M; dlng = e/M/longitude_scale(lat+dlat/2)."""
    import math
    n, e = 55.0, -38.0
    dlat = n / 111318.84502145034
    exp = (O[0] + dlat,
           O[1] + e / (111318.84502145034 * math.cos(math.radians(O[0] + dlat / 2))))
    got = geo.offset_latlon(*O, n, e)
    assert abs(got[0] - exp[0]) < 1e-12 and abs(got[1] - exp[1]) < 1e-12


def test_round_trip_is_exact():
    f = geo.LocalFrame(*O)
    for n, e in ((0, 0), (60, 60), (-60, 45), (12.5, -59.9)):
        n2, e2 = f.to_ne(*f.to_latlon(n, e))
        assert abs(n2 - n) < 1e-6 and abs(e2 - e) < 1e-6, (n, e, n2, e2)


def test_distance_is_symmetric_metres():
    p = geo.offset_latlon(*O, 30.0, 40.0)
    assert abs(geo.distance_m(*O, *p) - 50.0) < 1e-3
    assert abs(geo.distance_m(*O, *p) - geo.distance_m(*p, *O)) < 1e-9


def test_modules_reexport_the_one_constant():
    from swarm_control.mission import sar_coordination
    from swarm_control.tak import cot as tak_cot
    assert sar_coordination.M_PER_DEG_LAT == geo.M_PER_DEG_LAT
    assert tak_cot.M_PER_DEG_LAT == geo.M_PER_DEG_LAT
    assert sar_coordination.distance_m is geo.distance_m


def test_no_private_metres_per_degree_constants():
    """Regression for three coexisting models (111111, 111320, pyproj tmerc)."""
    bad = re.compile(r"\b111[_]?(111|320)(\.0)?\b|^\s*(import|from) pyproj|Transformer\.from_crs")
    hits = []
    for root in ("src",):
        for dp, _, files in os.walk(os.path.join(REPO, root)):
            if "__pycache__" in dp:
                continue
            for fn in files:
                if fn.endswith((".py", ".sh")) and fn != "geo.py":
                    path = os.path.join(dp, fn)
                    for i, line in enumerate(open(path, errors="replace"), 1):
                        if bad.search(line):
                            hits.append(f"{os.path.relpath(path, REPO)}:{i}: "
                                        f"{line.strip()[:80]}")
    assert not hits, "private geo model found:\n" + "\n".join(hits)


def test_ground_truth_files_use_geo():
    """Generated ground truth must be geo.offset of its own world x/y."""
    import json
    models = os.path.join(REPO, "src", "custom_gz", "models")
    p = json.load(open(os.path.join(models, "missing_person", "ground_truth.json")))
    lat, lon = geo.offset_latlon(*O, p["person"]["y"], p["person"]["x"])
    assert abs(lat - p["person"]["lat"]) < 1e-7 and abs(lon - p["person"]["lon"]) < 1e-7
    s = json.load(open(os.path.join(models, "sar_scenario", "ground_truth.json")))
    for d in s["distractors"]:
        lat, lon = geo.offset_latlon(*O, d["north_m"], d["east_m"])
        assert abs(lat - d["lat"]) < 1e-7 and abs(lon - d["lon"]) < 1e-7


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q"]))
