#!/usr/bin/env python3
"""Golden-master (characterization) tests: pin current behaviour so the
refactor can prove it changed nothing.

Inputs are recorded runs in tests/fixtures/; expected outputs in
tests/golden/*.json were produced by the code at the branch point. A
difference means behaviour changed. Regenerate ONLY for an intentional
change, and say so in the commit:

    python3 tests/test_golden.py            # check
    python3 tests/test_golden.py --regen    # rewrite tests/golden/*.json
"""
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIX = os.path.join(REPO, "tests", "fixtures")
GOLD = os.path.join(REPO, "tests", "golden")
PKG_ROOT = os.path.join(REPO, "src", "swarm_control")
sys.path.insert(0, PKG_ROOT)

from swarm_control.core import config  # noqa: E402
from swarm_control.perception import camera_processor as cp  # noqa: E402
from swarm_control.mission import points_distributor as pd  # noqa: E402
from swarm_control.mission import sar_coordination as sar  # noqa: E402
from swarm_control.mission import sar_search  # noqa: E402
from swarm_control.tak import picture as tak_bridge  # noqa: E402

ORIGIN = (config.FIELD_ORIGIN_LAT, config.FIELD_ORIGIN_LON)
FIXED_NOW = 1_800_000_000.0
REL_TOL = 1e-9
ABS_TOL = 1e-9


# ------------------------------------------------------------- comparison

def _close(a, b, path="$"):
    if isinstance(a, float) or isinstance(b, float):
        if not (isinstance(a, (int, float)) and isinstance(b, (int, float))):
            return [f"{path}: {a!r} != {b!r}"]
        return [] if math.isclose(a, b, rel_tol=REL_TOL, abs_tol=ABS_TOL) \
            else [f"{path}: {a!r} != {b!r}"]
    if isinstance(a, dict) and isinstance(b, dict):
        out = [f"{path}: keys {sorted(set(a) ^ set(b))}"] if set(a) != set(b) else []
        for k in sorted(set(a) & set(b)):
            out += _close(a[k], b[k], f"{path}.{k}")
        return out
    if isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b):
            return [f"{path}: len {len(a)} != {len(b)}"]
        out = []
        for i, (x, y) in enumerate(zip(a, b)):
            out += _close(x, y, f"{path}[{i}]")
        return out
    return [] if a == b else [f"{path}: {a!r} != {b!r}"]


def _plain(o):
    """JSON round trip: tuples -> lists, numpy scalars -> python."""
    return json.loads(json.dumps(o, default=float))


# ------------------------------------------------------------------ cases

def case_planner():
    """prescription_planner on the recorded mapping survey."""
    tmp = tempfile.mkdtemp()
    try:
        expect = []
        for d in sorted(os.listdir(os.path.join(FIX, "mapping_run"))):
            if d.startswith("drone_"):
                shutil.copytree(os.path.join(FIX, "mapping_run", d),
                                os.path.join(tmp, d))
                n = len([f for f in os.listdir(os.path.join(tmp, d))
                         if f.startswith("wp_") and f.endswith(".json")])
                expect.append(f"{d.split('_')[1]}:{n}")
        subprocess.run([sys.executable, "-B",
                        os.path.join(PKG_ROOT, "scripts", "prescription_planner.py"),
                        "--logs", tmp, "--sprayers", "2", "3", "4", "5",
                        "--expect", *expect, "--timeout", "5"],
                       check=True, capture_output=True, text=True)
        out = {}
        pdir = os.path.join(tmp, "prescriptions")
        for f in sorted(os.listdir(pdir)):
            d = json.load(open(os.path.join(pdir, f)))
            if isinstance(d, dict):
                d.pop("generated_unix", None)
            out[f] = d
        return _plain(out)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def _frames(run):
    root = os.path.join(FIX, run)
    for dp, _, files in sorted(os.walk(root)):
        for f in sorted(files):
            if f.endswith(".jpg"):
                side = json.load(open(os.path.join(dp, f[:-4] + ".json")))
                hd = side.get("heading_deg")
                heading = None if hd is None else math.radians(hd)
                yield (os.path.relpath(os.path.join(dp, f), root),
                       os.path.join(dp, f), side, heading)


def case_weed_georef():
    """Weed detection + georeferencing on recorded Gazebo frames."""
    import cv2
    lo, hi, area = cp.DEFAULT_DETECTORS["weed"]
    out = {}
    for rel, path, side, heading in _frames("mapping_run"):
        out[rel] = cp.detect_color_blobs(cv2.imread(path), lo, hi, area,
                                         lat=side["lat"], lon=side["lon"],
                                         alt=side["alt"], heading=heading)
    return _plain(out)


def case_person_detection():
    """Colour-blob and shape-verified person detection on SAR frames."""
    import cv2
    lo, hi, area = cp.PERSON_DETECTORS["person"]
    out = {}
    for rel, path, side, heading in _frames("sar_gazebo_run"):
        img = cv2.imread(path)
        kw = dict(lat=side["lat"], lon=side["lon"], alt=side["alt"],
                  heading=heading)
        out[rel] = {"blobs": cp.detect_color_blobs(img, lo, hi, area, **kw),
                    "shape": cp.detect_person(img, **kw)}
    return _plain(out)


def case_sar_geometry():
    """Search grid, launch points and rally slots for the recorded call."""
    g = sar_search.Grid(*ORIGIN)
    call = json.load(open(os.path.join(FIX, "sar_gazebo_run", "sar", "call.json")))
    swarm = [0, 1, 2, 3]
    return _plain({
        "cells": {f"r{r}c{c}": {"center": g.center_latlon((r, c)),
                                "corners": g.corners_latlon((r, c))}
                  for r, c in g.cells()},
        "cell_at_call": g.cell_at(call["lat"], call["lon"]),
        "launch_points": {n: sar_search.launch_points(n, 120.0)
                          for n in range(1, 9)},
        "rally": {i: sar.rally_slot(call, i, swarm) for i in swarm},
    })


def case_waypoints():
    """Survey patterns and the balanced 1/n serpentine split."""
    grid = pd.generate_grid_waypoints()
    return _plain({
        "grid": grid,
        "circular": pd.generate_circular_waypoints(),
        "split_serpentine": {n: pd.split_serpentine(grid, n) for n in range(1, 7)},
    })


_UUID = re.compile(r"[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}")
_VOLATILE_ATTRS = {"time", "start", "stale"}


def _xml_to_dict(el):
    attrs = {k: _UUID.sub("<uuid>", v) for k, v in el.attrib.items()
             if k not in _VOLATILE_ATTRS}
    return {"tag": el.tag, "attrs": attrs,
            "text": _UUID.sub("<uuid>", (el.text or "").strip()),
            "children": [_xml_to_dict(c) for c in el]}


def _cot(events):
    out = [_xml_to_dict(ET.fromstring(x.split("?>", 1)[-1])) for x in events]
    return sorted(out, key=lambda e: (e["attrs"].get("type", ""),
                                      e["attrs"].get("uid", "")))


def case_tak_sar():
    """TAK picture for the recorded SAR run (field, CASEVAC, GeoChat, statuses)."""
    tmp = tempfile.mkdtemp()
    try:
        shutil.copytree(os.path.join(FIX, "sar_gazebo_run"), tmp, dirs_exist_ok=True)
        b = tak_bridge.SwarmTakBridge([0, 1, 2, 3], logs_dir=tmp, mission="sar",
                                      field_half_m=60.0)
        return _cot(b.scan_artifacts(FIXED_NOW) + b.static_events(FIXED_NOW))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def case_tak_mapping():
    """TAK picture for the recorded mapping run (weed spots, field)."""
    tmp = tempfile.mkdtemp()
    try:
        shutil.copytree(os.path.join(FIX, "mapping_run"), tmp, dirs_exist_ok=True)
        b = tak_bridge.SwarmTakBridge([0, 1], sprayers=[2, 3, 4, 5], logs_dir=tmp)
        return _cot(b.scan_artifacts(FIXED_NOW) + b.static_events(FIXED_NOW))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


CASES = {n[5:]: f for n, f in sorted(globals().items()) if n.startswith("case_")}


def run(regen=False):
    os.makedirs(GOLD, exist_ok=True)
    failed = 0
    for name, fn in CASES.items():
        path = os.path.join(GOLD, f"{name}.json")
        got = fn()
        if regen:
            with open(path, "w") as f:
                json.dump(got, f, indent=1, sort_keys=True)
                f.write("\n")
            print(f"REGEN {name}")
            continue
        if not os.path.exists(path):
            failed += 1
            print(f"FAIL {name}: no golden (run with --regen at the branch point)")
            continue
        diffs = _close(json.load(open(path)), got)
        if diffs:
            failed += 1
            print(f"FAIL {name}: {len(diffs)} differences, e.g.\n    "
                  + "\n    ".join(diffs[:5]))
        else:
            print(f"PASS {name}")
    return failed


if __name__ == "__main__":
    sys.exit(1 if run(regen="--regen" in sys.argv) else 0)
