#!/usr/bin/env python3
"""Search-and-rescue tests (no ROS / Gazebo needed).

Covers the call channel, rally geometry, the person detector (needs cv2 +
numpy; skipped otherwise), the missing-person generator, the TAK relay, and
an end-to-end swarm run: four REAL SearchDrone mission loops in threads with
ROS stubbed out, a fake camera that sees the person only inside its ground
footprint, and fake GPS that flies to each commanded target.

Run directly:  python3 src/swarm_control/tests/test_sar.py
"""
import importlib.util
import json
import math
import os
import sys
import tempfile
import threading
import time
import types
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)                       # src/swarm_control
REPO = os.path.dirname(os.path.dirname(PKG))
sys.path.insert(0, PKG)                           # the swarm_control package

from swarm_control.mission import sar_coordination as sar  # noqa: E402

try:
    import cv2
    import numpy as np
    _CV = True
except ImportError:
    _CV = False

LAT0, LON0 = 40.072842, -105.230575


# ------------------------------------------------------------ coordination

def test_call_first_writer_wins_under_race():
    with tempfile.TemporaryDirectory() as logs:
        results = []
        barrier = threading.Barrier(8)

        def caller(i):
            bus = sar.SarBus(logs)
            barrier.wait()
            results.append(bus.try_call(i, i + 1, LAT0 + i * 1e-5, LON0, 15.0))

        ts = [threading.Thread(target=caller, args=(i,)) for i in range(8)]
        [t.start() for t in ts]
        [t.join() for t in ts]
        winners = [c for won, c in results if won]
        assert len(winners) == 1, winners
        # losers all see the winner's call, not their own
        for won, c in results:
            assert c["found_by_instance"] == winners[0]["found_by_instance"]


def test_rally_slots_distinct():
    call = {"found_by_instance": 2, "found_by_id": 3, "lat": LAT0,
            "lon": LON0, "confirm_alt": 15.0}
    swarm = [0, 1, 2, 3, 4, 5]
    slots = {i: sar.rally_slot(call, i, swarm, ring_radius=15.0) for i in swarm}
    assert slots[2] == (LAT0, LON0, 15.0)               # finder overhead
    responders = [slots[i] for i in swarm if i != 2]
    alts = [s[2] for s in responders]
    assert len(set(alts)) == len(alts) and min(alts) > 30.0
    for s in responders:
        assert abs(sar.distance_m(LAT0, LON0, s[0], s[1]) - 15.0) < 0.01
    for a in range(len(responders)):
        for b in range(a + 1, len(responders)):
            assert sar.distance_m(responders[a][0], responders[a][1],
                                  responders[b][0], responders[b][1]) > 10


def test_detection_selection():
    dets = [{"lat": None, "lon": None, "area_px": 999},
            {"lat": LAT0, "lon": LON0, "area_px": 100},
            {"lat": LAT0 + 5e-5, "lon": LON0, "area_px": 300}]
    assert sar.best_detection(dets)["area_px"] == 300
    assert sar.best_detection([]) is None
    m = sar.match_detection(dets, LAT0 + 1e-5, LON0, 8.0)
    assert m["area_px"] == 100                          # 1.1 m vs 4.4 m
    assert sar.match_detection(dets, LAT0 + 5e-4, LON0, 8.0) is None


# -------------------------------------------------------- person detector

def test_person_detector_vs_farm_palette():
    if not _CV:
        print("  (skipped: cv2/numpy not installed)")
        return
    from swarm_control.perception.camera_processor import (detect_color_blobs, PERSON_HSV_LOW,
                                  PERSON_HSV_HIGH, PERSON_MIN_AREA_PX,
                                  CAMERA_HFOV_RAD)

    def det(img, **kw):
        return detect_color_blobs(img, PERSON_HSV_LOW, PERSON_HSV_HIGH,
                                  PERSON_MIN_AREA_PX, **kw)

    rgb = lambda r, g, b: (b, g, r)  # noqa: E731  (OpenCV is BGR)
    img = np.zeros((480, 640, 3), np.uint8)
    # Every farm texture colour (generate_field_textures.py) + magenta weed
    palette = [(130, 100, 70), (96, 74, 50), (98, 122, 66), (66, 132, 52),
               (232, 232, 228), (224, 178, 40), (198, 174, 84),
               (188, 164, 96), (222, 206, 60), (236, 222, 80),
               (78, 148, 80), (255, 0, 255)]
    for k, c in enumerate(palette):
        x = (k % 4) * 160
        y = (k // 4) * 160
        img[y:y + 160, x:x + 160] = rgb(*c)
    assert det(img) == [], "farm palette / weeds must not look like a person"

    # Person at 30 m: 1.5 x 0.45 m at 6.4 cm/px ~ 23 x 7 px, 100 px east
    f_px = 320 / math.tan(CAMERA_HFOV_RAD / 2)
    mpp = 30.0 / f_px
    w, h = int(1.5 / mpp), int(0.45 / mpp)
    cx, cy = 420, 240
    img[cy - h // 2:cy + h // 2 + 1, cx - w // 2:cx + w // 2 + 1] = \
        rgb(255, 89, 0)
    d = det(img, lat=LAT0, lon=LON0, alt=30.0)
    assert len(d) == 1, d
    east = (d[0]["centroid_px"][0] - 320) / f_px * 30.0
    assert abs(east - 100 * mpp) < 0.2
    assert abs(sar.distance_m(LAT0, LON0, d[0]["lat"], d[0]["lon"])
               - 100 * mpp) < 0.2


# -------------------------------------------------------------- generator

def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_missing_person_generator():
    gen = _load("gmp", os.path.join(REPO, "src", "custom_gz", "worlds",
                                    "generate_missing_person.py"))
    for seed in range(200):
        p = gen.place_person(seed, 120.0)
        assert abs(p["x"]) <= 55 and abs(p["y"]) <= 55
        assert math.hypot(p["x"] - gen.SPAWN_XY[0],
                          p["y"] - gen.SPAWN_XY[1]) >= gen.SPAWN_EXCLUSION_M
    assert gen.place_person(7, 120.0) == gen.place_person(7, 120.0)
    with tempfile.TemporaryDirectory() as d:
        p = gen.place_person(3, 120.0)
        gen.write_model(p, d, {"seed": 3})
        root = ET.parse(os.path.join(d, "model.sdf")).getroot()
        assert root.find("model/pose").text.startswith(f"{p['x']} {p['y']}")
        gt = json.load(open(os.path.join(d, "ground_truth.json")))
        assert gt["person"]["lat"] == p["lat"]

    # Committed model matches its ground truth; world includes it pose-less
    mdir = os.path.join(REPO, "src", "custom_gz", "models", "missing_person")
    gt = json.load(open(os.path.join(mdir, "ground_truth.json")))
    pose = ET.parse(os.path.join(mdir, "model.sdf")).getroot().find(
        "model/pose").text.split()
    assert float(pose[0]) == gt["person"]["x"]
    world = ET.parse(os.path.join(REPO, "src", "custom_gz", "worlds",
                                  "swarm_drone_sar.sdf")).getroot()
    incs = [i for i in world.iter("include")
            if i.findtext("uri") == "model://missing_person"]
    assert len(incs) == 1 and incs[0].find("pose") is None
    assert not any(i.findtext("uri") == "model://weed_field"
                   for i in world.iter("include"))


# ---------------------------------------------------- end-to-end swarm run

def _stub_ros():
    """Minimal stand-ins so base_drone_dds / search_drone_dds import."""
    def mod(name, **attrs):
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        sys.modules[name] = m
        return m

    class Anything:
        def __init__(self, *a, **k):
            pass

    class Node:
        def __init__(self, *a, **k):
            pass

    mod("rclpy", spin_once=lambda *a, **k: None, init=lambda *a, **k: None)
    mod("rclpy.node", Node=Node)
    mod("rclpy.qos", QoSProfile=Anything, ReliabilityPolicy=Anything,
        DurabilityPolicy=Anything)
    mod("ardupilot_msgs")
    mod("ardupilot_msgs.msg", Status=Anything, GlobalPosition=Anything)
    mod("ardupilot_msgs.srv", ArmMotors=Anything, ModeSwitch=Anything,
        Takeoff=Anything)
    mod("geometry_msgs")
    mod("geometry_msgs.msg", PoseStamped=Anything)
    mod("geographic_msgs")
    mod("geographic_msgs.msg", GeoPoseStamped=Anything,
        GeoPointStamped=Anything)


class FakeCamera:
    """Sees the person iff inside the nadir footprint (HFOV 1.2, 4:3)."""

    def __init__(self, person, gps):
        self.person, self.gps, self.last_meta = person, gps, None
        self.captures = 0

    def capture(self, idx, lat=None, lon=None, alt=None, prefix="wp",
                heading=None):
        self.captures += 1
        dlat, dlon, _ = self.gps()          # where the drone REALLY is
        dn = (self.person[0] - dlat) * sar.M_PER_DEG_LAT
        de = (self.person[1] - dlon) * sar.M_PER_DEG_LAT * math.cos(
            math.radians(dlat))
        half_e = alt * math.tan(0.6)
        dets = []
        if abs(de) <= half_e and abs(dn) <= half_e * 0.75:
            err = 0.02 * alt                # georef error grows with height
            elat, elon = sar.offset_latlon(self.person[0], self.person[1],
                                           err, -err)
            dets = [{"lat": elat, "lon": elon,
                     "area_px": 160 * (30.0 / alt) ** 2}]
        self.last_meta = {"person_detections": dets}

    def close(self):
        pass


def _make_drone(SearchDrone, inst, swarm, wps, logs, person, algo="strips",
                start=None):
    from swarm_control.mission import sar_search
    d = SearchDrone.__new__(SearchDrone)       # skip ROS-bound __init__
    d.search_algo, d.logs_dir = algo, logs
    d.grid = sar_search.Grid(LAT0, LON0, 120.0, 7)
    d.drone_id, d.instance, d.swarm = inst + 1, inst, swarm
    d.waypoints = wps
    d.transformer = object()
    d.search_alt, d.confirm_alt, d.confirm_radius = 30.0, 15.0, 8.0
    d.ring_radius, d.alt_step, d.dwell_s = 15.0, 5.0, 0.05
    d.rally_hold_s, d.arrive_radius, d.converge_timeout = 0.3, 4.0, 5.0
    d.standby_timeout = 10.0
    d.leg_timeout = 5.0
    d.current_geopose = None
    d.level_tilt, d.level_timeout = 0.05, 1.0
    d.poll_s, d.converge_poll_s = 0.005, 0.01
    d.home_lat = d.home_lon = d.home_alt = None
    d.bus = sar.SarBus(logs)
    d.gps = (start or (LAT0, LON0)) + (1586.0,)
    d.track = []
    d.mode = None

    class Log:
        def info(self, m):
            pass
        warning = error = info
    d.get_logger = lambda: Log()
    d.xy_to_latlon = lambda x, y: sar.offset_latlon(LAT0, LON0, x, y)

    def send(lat, lon, alt):               # "fly": arrive almost at once
        d.track.append((lat, lon, alt))
        d.gps = (lat, lon, 1586.0 + alt)
    d.send_position_command_dds = send
    d.switch_mode = lambda m: setattr(d, "mode", m) or True
    d.camera = FakeCamera(person, lambda: d.gps)
    return d


def test_end_to_end_search_call_rally():
    _stub_ros()
    from swarm_control.nodes.search_drone import SearchDrone
    from swarm_control.mission.points_distributor import generate_grid_waypoints, split_serpentine
    from swarm_control.tak import picture as tak_bridge

    gt = json.load(open(os.path.join(REPO, "src", "custom_gz", "models",
                                     "missing_person", "ground_truth.json")))
    person = (gt["person"]["lat"], gt["person"]["lon"])
    swarm = [0, 1, 2, 3]
    chunks = split_serpentine(
        generate_grid_waypoints(field_size=120.0, grid_points=7,
                                height=30.0), 4)
    with tempfile.TemporaryDirectory() as logs:
        drones = [_make_drone(SearchDrone, i, swarm, chunks[i], logs, person)
                  for i in swarm]
        threads = [threading.Thread(target=d.start_mission) for d in drones]
        [t.start() for t in threads]
        [t.join(timeout=30) for t in threads]
        assert not any(t.is_alive() for t in threads), "mission hung"

        call = json.load(open(os.path.join(logs, "sar", "call.json")))
        err = sar.distance_m(person[0], person[1], call["lat"], call["lon"])
        assert err < 1.0, f"confirmed position {err:.2f} m off"
        finder = drones[call["found_by_instance"]]
        # The finder descended to confirm before calling
        assert any(abs(t[2] - 15.0) < 1e-9 for t in finder.track)

        statuses = sar.SarBus(logs).read_statuses()
        assert all(st["state"] == "returning" for st in statuses.values())
        for d in drones:
            assert d.mode is not None, "every drone RTLs at the end"
            slot = sar.rally_slot(call, d.instance, swarm)
            assert d.track[-1] == slot, (d.instance, d.track[-1], slot)
        # Responders aborted their strips: fewer captures than waypoints
        assert sum(d.camera.captures for d in drones) < \
            sum(len(c) for c in chunks) + 1

        # TAK relay: one CASEVAC at the survivor + one GeoChat page
        b = tak_bridge.SwarmTakBridge(swarm, logs_dir=logs, mission="sar",
                                      field_half_m=60.0)
        evs = [ET.fromstring(x.split("?>", 1)[1]) for x in b.scan_artifacts()]
        types_ = [e.get("type") for e in evs]
        assert types_.count("b-r-f-h-c") == 1 and types_.count("b-t-f") == 1
        cas = next(e for e in evs if e.get("type") == "b-r-f-h-c")
        # CoT prints 8 decimals (~1 mm): compare at that resolution, not 1e-9.
        assert abs(float(cas.find("point").get("lat")) - call["lat"]) <= 5e-9
        chat = next(e for e in evs if e.get("type") == "b-t-f")
        assert "SURVIVOR FOUND" in chat.find("detail/remarks").text
        assert chat.find("detail/__chat").get("chatroom") == "All Chat Rooms"
        field = next(e for e in evs if e.get("type") == "u-d-r")
        assert "PERSON FOUND" in field.find("detail/remarks").text
        assert b.scan_artifacts() == []        # the page is sent only once
        statics = [ET.fromstring(x.split("?>", 1)[1])
                   for x in b.static_events()]
        assert [e.get("type") for e in statics].count("b-r-f-h-c") == 1
        assert [e.get("type") for e in statics].count("b-t-f") == 0
        b.update_pose(0, LAT0, LON0, 1616.0, t=time.time())
        rem = ET.fromstring(b.drone_events()[0].split("?>", 1)[1]).find(
            "detail/remarks").text
        assert "SEARCH-1" not in rem and "search" in rem and "SAR:" in rem


def test_not_found_everyone_completes_and_rtls():
    _stub_ros()
    from swarm_control.nodes.search_drone import SearchDrone
    from swarm_control.mission.points_distributor import generate_grid_waypoints, split_serpentine
    from swarm_control.tak import picture as tak_bridge
    far = sar.offset_latlon(LAT0, LON0, 500.0, 500.0)   # outside the area
    chunks = split_serpentine(
        generate_grid_waypoints(field_size=120.0, grid_points=7,
                                height=30.0), 3)
    with tempfile.TemporaryDirectory() as logs:
        drones = [_make_drone(SearchDrone, i, [0, 1, 2], chunks[i], logs, far)
                  for i in range(3)]
        threads = [threading.Thread(target=d.start_mission) for d in drones]
        [t.start() for t in threads]
        [t.join(timeout=30) for t in threads]
        assert not os.path.exists(os.path.join(logs, "sar", "call.json"))
        assert all(d.camera.captures == len(c) for d, c in zip(drones, chunks))
        b = tak_bridge.SwarmTakBridge([0, 1, 2], logs_dir=logs, mission="sar")
        b.scan_artifacts()
        for st in b.sar_status.values():
            st["state"] = "search_complete"
        assert "NOT found" in b._sar_field_status()


def test_grid_search_from_separate_launch_points():
    """BFS and DFS: drones start at different corners, claim cells, never
    search one twice, and still find + confirm + rally on the person."""
    _stub_ros()
    from swarm_control.nodes.search_drone import SearchDrone
    from swarm_control.mission import sar_search
    gt = json.load(open(os.path.join(REPO, "src", "custom_gz", "models",
                                     "missing_person", "ground_truth.json")))
    person = (gt["person"]["lat"], gt["person"]["lon"])
    far = sar.offset_latlon(LAT0, LON0, 500.0, 500.0)
    swarm = [0, 1, 2, 3]
    starts = [sar.offset_latlon(LAT0, LON0, n, e)
              for n, e in sar_search.launch_points(4, 120.0)]
    for algo in sar_search.ALGORITHMS:
        for target, found in ((person, True), (far, False)):
            with tempfile.TemporaryDirectory() as logs:
                drones = [_make_drone(SearchDrone, i, swarm, [], logs, target,
                                      algo=algo, start=starts[i])
                          for i in swarm]
                ts = [threading.Thread(target=d.start_mission) for d in drones]
                [t.start() for t in ts]
                [t.join(timeout=30) for t in ts]
                assert not any(t.is_alive() for t in ts), "mission hung"
                claims = sar_search.ClaimBoard(logs, drones[0].grid).all_claims()
                cells = [(c["row"], c["col"]) for c in claims]
                assert len(cells) == len(set(cells)), "cell searched twice"
                # each drone's first cell is the one under its launch point
                for i, d in enumerate(drones):
                    first = d.track[0]
                    assert d.grid.cell_at(first[0], first[1]) == \
                        d.grid.cell_at(*starts[i]), (algo, i)
                call_path = os.path.join(logs, "sar", "call.json")
                if found:
                    call = json.load(open(call_path))
                    assert sar.distance_m(person[0], person[1], call["lat"],
                                          call["lon"]) < 1.0
                    assert len(cells) < 49        # search stopped early
                else:
                    assert not os.path.exists(call_path)
                    assert len(cells) == 49       # whole grid covered once
                    by = [sum(c["instance"] == i for c in claims)
                          for i in swarm]
                    assert max(by) - min(by) <= 3, by   # work is shared


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print(f"PASS {t.__name__}")
    print(f"{len(tests)} tests passed")
