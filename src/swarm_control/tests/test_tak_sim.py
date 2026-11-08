#!/usr/bin/env python3
"""Tests for the no-Gazebo TAK simulation path (no ROS, no Gazebo needed):

  * synthetic_camera: rendered frames go through the REAL HSV detectors and
    georeference back onto the ground truth (weeds + missing person).
  * tak_cot.PytakSender / make_sender: PyTAK transport over UDP and TCP
    (skipped when pytak isn't installed).
  * tak_viewer.Picture: CoT parsing, tracks, coverage footprints, deletes,
    CASEVAC + GeoChat feed.

Run directly:  python3 src/swarm_control/tests/test_tak_sim.py
"""
import json
import math
import os
import socket
import sys
import tempfile
import time

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, '..', 'scripts'))
MODELS = os.path.join(HERE, '..', '..', 'custom_gz', 'models')
if not os.path.isdir(MODELS):  # inside the container: compose mount
    MODELS = os.environ.get("SIM_MODELS_DIR", "/root/sim_models")

import camera_processor as cp  # noqa: E402
import synthetic_camera as sc  # noqa: E402
import tak_cot  # noqa: E402
import tak_viewer  # noqa: E402
from sar_coordination import distance_m, best_detection  # noqa: E402
import geo  # noqa: E402

SKIP = object()


def _offset(lat, lon, n, e):
    return geo.offset_latlon(lat, lon, n, e)


# ------------------------------------------------------ synthetic camera

def test_projection_inverts_georef():
    lat0, lon0 = 40.0728, -105.2306
    for hd in (0.0, 0.7, math.pi, -2.0):
        lat, lon = _offset(lat0, lon0, 7.0, -4.0)
        x, y = sc.project(lat0, lon0, 30.0, hd, lat, lon)
        f_px = (sc.FRAME_W / 2) / math.tan(cp.CAMERA_HFOV_RAD / 2)
        right = (x - sc.FRAME_W / 2) / f_px * 30.0
        fwd = -(y - sc.FRAME_H / 2) / f_px * 30.0
        n = fwd * math.cos(hd) - right * math.sin(hd)
        e = fwd * math.sin(hd) + right * math.cos(hd)
        assert abs(n - 7.0) < 1e-6 and abs(e + 4.0) < 1e-6


def test_weeds_detected_and_georeferenced():
    if not cp._CV2_OK:
        return SKIP
    weeds, _ = sc.load_ground_truth(MODELS)
    assert len(weeds) >= 5
    for hd in (0.0, 1.1, math.pi):
        lat0, lon0 = _offset(weeds[0]["lat"], weeds[0]["lon"], 5.0, -3.0)
        f = sc.render_frame(lat0, lon0, 30.0, hd, weeds, None)
        dets = cp.detect_color_blobs(f, *cp.DEFAULT_DETECTORS["weed"],
                                     lat=lat0, lon=lon0, alt=30.0, heading=hd)
        assert dets, "weed under the drone not detected"
        for d in dets:
            err = min(distance_m(d["lat"], d["lon"], w["lat"], w["lon"])
                      for w in weeds)
            assert err < 0.5, f"weed georef error {err:.2f} m"


def test_person_detected_at_search_and_confirm_alt():
    if not cp._CV2_OK:
        return SKIP
    _, person = sc.load_ground_truth(MODELS)
    assert person
    areas = {}
    for alt in (30.0, 15.0):
        lat0, lon0 = _offset(person["lat"], person["lon"], -8.0, 6.0)
        f = sc.render_frame(lat0, lon0, alt, 0.4, [], person)
        dets = cp.detect_color_blobs(f, *cp.PERSON_DETECTORS["person"],
                                     lat=lat0, lon=lon0, alt=alt, heading=0.4)
        best = best_detection(dets)
        assert best, f"person not detected at {alt} m"
        assert distance_m(best["lat"], best["lon"],
                          person["lat"], person["lon"]) < 0.5
        areas[alt] = best["area_px"]
    assert areas[15.0] > 3 * areas[30.0]   # closer -> bigger blob


def test_bands_do_not_cross_trigger():
    if not cp._CV2_OK:
        return SKIP
    weeds, person = sc.load_ground_truth(MODELS)
    f = sc.render_frame(person["lat"], person["lon"], 30.0, 0.0, [], person)
    assert not cp.detect_color_blobs(f, *cp.DEFAULT_DETECTORS["weed"])
    f = sc.render_frame(weeds[0]["lat"], weeds[0]["lon"], 30.0, 0.0, weeds)
    assert not cp.detect_color_blobs(f, *cp.PERSON_DETECTORS["person"])


def test_synthetic_camera_writes_sidecar_and_factory():
    if not cp._CV2_OK:
        return SKIP
    tmp = tempfile.mkdtemp()
    _, person = sc.load_ground_truth(MODELS)
    cam = sc.SyntheticCamera(2, out_dir=tmp, models_dir=MODELS,
                             detectors=cp.PERSON_DETECTORS)
    assert cam.active
    img = cam.capture(3, lat=person["lat"], lon=person["lon"], alt=15.0,
                      heading=0.0, prefix="confirm")
    assert img and os.path.isfile(img)
    with open(os.path.join(tmp, "drone_2", "confirm_003.json")) as f:
        meta = json.load(f)
    assert meta["person_detections"]
    assert cam.capture(4, lat=None, lon=None, alt=None) is None  # no pose
    old = os.environ.get("SIM_SENSORS")
    try:
        os.environ["SIM_SENSORS"] = "synthetic"
        assert isinstance(sc.make_camera(0, 5600, out_dir=tmp),
                          sc.SyntheticCamera)
        os.environ["SIM_SENSORS"] = ""
        c = sc.make_camera(0, 0, out_dir=tmp)
        assert type(c) is cp.CameraProcessor and not c.active
    finally:
        if old is None:
            os.environ.pop("SIM_SENSORS", None)
        else:
            os.environ["SIM_SENSORS"] = old


# ---------------------------------------------------------- pytak sender

def test_make_sender_selection():
    assert type(tak_cot.make_sender("udp://127.0.0.1:9", "stdlib",
                                    logger=lambda m: None)) is tak_cot.CotSender
    f = tak_cot.make_sender(f"file://{tempfile.mkdtemp()}/c.log", "pytak",
                            logger=lambda m: None)
    assert type(f) is tak_cot.CotSender     # file:// never goes to pytak
    try:
        tak_cot.make_sender("udp://127.0.0.1:9", "bogus")
        assert False
    except ValueError:
        pass


def _wait(pred, timeout=3.0):
    end = time.time() + timeout
    while time.time() < end:
        if pred():
            return True
        time.sleep(0.05)
    return False


def test_pytak_udp_and_tcp():
    if not tak_cot.pytak_available():
        return SKIP
    xml = tak_cot.drone_event("u1", "SEARCH-1", 40.07, -105.23, 1600.0)
    r = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    r.bind(("127.0.0.1", 0))
    r.settimeout(3)
    s = tak_cot.make_sender(f"udp://127.0.0.1:{r.getsockname()[1]}",
                            "pytak", logger=lambda m: None)
    assert type(s) is tak_cot.PytakSender
    assert _wait(lambda: s.send(xml))
    assert b'callsign="SEARCH-1"' in r.recv(65535)
    s.close()
    r.close()

    srv = socket.socket()
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)
    srv.settimeout(3)
    t = tak_cot.make_sender(f"tcp://127.0.0.1:{srv.getsockname()[1]}",
                            "pytak", logger=lambda m: None)
    c, _ = srv.accept()
    c.settimeout(3)
    assert _wait(lambda: t.send(xml))
    assert b"<event" in c.recv(65535)
    t.close()
    c.close()
    srv.close()


def test_pytak_unreachable_never_raises():
    if not tak_cot.pytak_available():
        return SKIP
    s = tak_cot.make_sender("tcp://127.0.0.1:1", "pytak",
                            logger=lambda m: None)
    time.sleep(0.3)
    assert s.send("<event/>") is False
    s.close()


# ----------------------------------------------------------------- viewer

def test_viewer_picture_sar_flow():
    clock = [1000.0]
    pic = tak_viewer.Picture(now=lambda: clock[0])
    uid = "ardupilot-swarm-drone-0"
    for k in range(4):
        lat, lon = _offset(40.0728, -105.2306, 15.0 * k, 0.0)
        pic.ingest(tak_cot.drone_event(
            uid, "SEARCH-1", lat, lon, 1600.0, course=0.0, speed=5.0,
            battery=88, remarks=f"search | GUIDED | ARMED | 30.0 m AGL | "
                                f"5.0 m/s | SAR: searching wp {k + 1}/12",
            now=clock[0]))
        clock[0] += 1
    snap = pic.snapshot()
    d = next(i for i in snap["items"] if i["uid"] == uid)
    assert d["agl"] == 30.0 and d["mode"] == "GUIDED"
    assert d["sar"] == "searching wp 4/12" and d["battery"] == 88
    assert len(d["trail"]) == 4
    assert len(d["footprints"]) == 4        # 15 m apart > FOOTPRINT_STEP_M

    corners = [_offset(40.0728, -105.2306, n, e)
               for n, e in ((60, -60), (60, 60), (-60, 60), (-60, -60))]
    pic.ingest(tak_cot.rectangle_event("f", "SEARCH AREA", corners,
                                       remarks="120x120 m | searching"))
    pic.ingest(tak_cot.casevac_event("cas", "SURVIVOR", 40.0725, -105.2308,
                                     remarks="Image: drone_1/confirm_003.jpg"))
    pic.ingest(tak_cot.chat_event(uid, "SEARCH-1", "SURVIVOR FOUND",
                                  40.0725, -105.2308))
    pic.ingest(tak_cot.drone_event(
        uid, "SEARCH-1", 40.0725, -105.2308, 1600.0,
        remarks="search | GUIDED | ARMED | 15.0 m AGL | SAR: ON SCENE (finder)"))
    snap = pic.snapshot()
    types = {i["type"] for i in snap["items"]}
    assert {"u-d-r", "b-r-f-h-c", "b-t-f"} <= types
    area = next(i for i in snap["items"] if i["type"] == "u-d-r")
    assert len(area["polygon"]) == 4
    kinds = [f["kind"] for f in snap["feed"]]
    assert "alert" in kinds and "chat" in kinds
    assert any("ON SCENE" in f["text"] for f in snap["feed"])

    pic.ingest(tak_cot.delete_event("cas", tak_cot.TYPE_CASEVAC))
    assert not any(i["uid"] == "cas" for i in pic.snapshot()["items"])


def test_viewer_parse_robust():
    assert tak_viewer.parse_cot(b"garbage") is None
    assert tak_viewer.parse_cot(b"<event uid='x'") is None
    two = (tak_cot.spot_event("a", "A", 1, 2) + tak_cot.spot_event("b", "B",
                                                                   3, 4))
    assert tak_viewer.parse_cot(two)["uid"] == "a"


if __name__ == "__main__":
    tests = [(n, f) for n, f in sorted(globals().items())
             if n.startswith("test_") and callable(f)]
    failed = skipped = 0
    for name, fn in tests:
        try:
            res = fn()
            if res is SKIP:
                skipped += 1
                print(f"SKIP {name}")
            else:
                print(f"ok   {name}")
        except Exception as e:  # noqa: BLE001
            failed += 1
            print(f"FAIL {name}: {type(e).__name__}: {e}")
    print(f"\n{len(tests) - failed - skipped} passed, {skipped} skipped, "
          f"{failed} failed")
    sys.exit(1 if failed else 0)
