#!/usr/bin/env python3
"""Tests for the TAK/CoT integration (no ROS, no TAK install needed).

Run directly:  python3 src/swarm_control/tests/test_tak_bridge.py
"""
import json
import os
import socket
import sys
import tempfile
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))

import tak_cot  # noqa: E402
from tak_bridge import SwarmTakBridge, FIELD_ORIGIN_LAT, FIELD_ORIGIN_LON  # noqa: E402


def parse(xml):
    root = ET.fromstring(xml.split("?>", 1)[1])
    assert root.tag == "event" and root.get("version") == "2.0"
    for attr in ("uid", "type", "how", "time", "start", "stale"):
        assert root.get(attr), f"missing event@{attr}"
    for attr in ("time", "start", "stale"):
        assert root.get(attr).endswith("Z") and len(root.get(attr)) == 24
    assert root.get("stale") > root.get("time")
    pt = root.find("point")
    for attr in ("lat", "lon", "hae", "ce", "le"):
        float(pt.get(attr))
    assert root.find("detail") is not None
    return root


def test_argb_signed():
    assert tak_cot.MAGENTA == -65281          # TAK's encoding of 0xFFFF00FF
    assert tak_cot.WHITE == -1


def test_drone_event():
    xml = tak_cot.drone_event("u1", "MAPPER-1", 40.07, -105.23, hae=1600.0,
                              course=370.0, speed=4.2, battery=87.4,
                              remarks="a<b & c", video_url="rtsp://x/1",
                              now=1_700_000_000.0)
    root = parse(xml)
    assert root.get("type") == "a-f-A-M-F-Q-r"
    d = root.find("detail")
    assert d.find("contact").get("callsign") == "MAPPER-1"
    assert d.find("track").get("course") == "10.0"   # wrapped
    assert d.find("status").get("battery") == "87"
    assert d.find("__video").get("url") == "rtsp://x/1"
    assert d.find("remarks").text == "a<b & c"       # escaped round-trip


def test_shapes_and_delete():
    corners = [(1, 1), (1, 2), (0, 2), (0, 1)]
    r = parse(tak_cot.rectangle_event("f", "FIELD", corners))
    assert r.get("type") == "u-d-r"
    assert len(r.find("detail").findall("link")) == 4
    dl = parse(tak_cot.delete_event("victim"))
    assert dl.get("type") == "t-x-d-d"
    assert dl.find("detail/link").get("uid") == "victim"
    assert dl.find("detail/__forcedelete") is not None


def test_course_speed():
    lat2 = 40.0 + 10.0 / tak_cot.M_PER_DEG_LAT        # 10 m north in 2 s
    course, speed = tak_cot.course_speed(40.0, -105.0, 0.0, lat2, -105.0, 2.0)
    assert abs(course) < 1e-6 or abs(course - 360) < 1e-6
    assert abs(speed - 5.0) < 1e-6


def _write_mission(logs):
    os.makedirs(os.path.join(logs, "drone_1"))
    side = {"drone_id": 1, "wp_index": 3, "lat": 40.0728, "lon": -105.2305,
            "alt": 30.0, "weed_detections": [
                {"area_px": 900, "lat": 40.07290, "lon": -105.23060},
                {"area_px": 50, "lat": None, "lon": None}]}  # ungeoreferenced
    with open(os.path.join(logs, "drone_1", "wp_003.json"), "w") as f:
        json.dump(side, f)


def _write_plan(logs, inst, n):
    os.makedirs(os.path.join(logs, "prescriptions"), exist_ok=True)
    plan = {"sprayer_instance": inst, "clusters": [
        {"lat": 40.0729 + k * 1e-5, "lon": -105.2306, "num_detections": 2}
        for k in range(n)]}
    with open(os.path.join(logs, "prescriptions", f"sprayer_{inst}.json"),
              "w") as f:
        json.dump(plan, f)


def test_bridge_pipeline():
    with tempfile.TemporaryDirectory() as logs:
        b = SwarmTakBridge([0, 1, 2], sprayers=[2], logs_dir=logs,
                           video_url="rtsp://h:8554/drone{instance}")

        # Telemetry -> tracks with course/speed/AGL; sprayer has no video
        b.update_home(0, 1586.0)
        b.update_status(0, armed=True, mode=4)
        b.update_battery(0, 0.9)
        b.update_pose(0, FIELD_ORIGIN_LAT, FIELD_ORIGIN_LON, 1616.0, t=100.0)
        b.update_pose(0, FIELD_ORIGIN_LAT, FIELD_ORIGIN_LON + 5 / 85000.0,
                      1616.0, t=101.0)
        b.update_pose(2, FIELD_ORIGIN_LAT, FIELD_ORIGIN_LON, 1601.0, t=101.0)
        evs = [parse(x) for x in b.drone_events(now=101.5)]
        assert len(evs) == 2                      # instance 1 has no fix
        m0 = next(e for e in evs if e.get("uid").endswith("drone-0"))
        s2 = next(e for e in evs if e.get("uid").endswith("drone-2"))
        assert m0.find("detail/contact").get("callsign") == "MAPPER-1"
        assert s2.find("detail/contact").get("callsign") == "SPRAYER-3"
        assert abs(float(m0.find("point").get("hae")) - (1616 - 17)) < 1e-6
        assert 80 < float(m0.find("detail/track").get("course")) < 100  # east
        assert "GUIDED" in m0.find("detail/remarks").text
        assert "30.0 m AGL" in m0.find("detail/remarks").text
        assert m0.find("detail/status").get("battery") == "90"
        assert m0.find("detail/__video").get("url") == "rtsp://h:8554/drone0"
        assert s2.find("detail/__video") is None
        assert b.drone_events(now=120.0) == []    # fixes stopped -> go stale

        # Survey: one georeferenced detection -> magenta spot + field rect
        _write_mission(logs)
        evs = [parse(x) for x in b.scan_artifacts()]
        spots = [e for e in evs if e.get("type") == "b-m-p-s-m"]
        assert len(spots) == 1
        assert spots[0].find("detail/color").get("argb") == str(tak_cot.MAGENTA)
        field = [e for e in evs if e.get("type") == "u-d-r"]
        assert "1 weed detections" in field[0].find("detail/remarks").text
        assert b.scan_artifacts() == []           # unchanged -> nothing new

        # Plan lands -> detections deleted, targets published
        _write_plan(logs, 2, 3)
        evs = [parse(x) for x in b.scan_artifacts()]
        types = [e.get("type") for e in evs]
        assert types.count("t-x-d-d") == 1
        assert types.count("b-m-p-s-m") == 3
        t0 = next(e for e in evs if e.get("type") == "b-m-p-s-m")
        assert t0.find("detail/link").get("uid").endswith("drone-2")
        statics = [parse(x) for x in b.static_events()]
        assert sum(e.get("type") == "b-m-p-s-m" for e in statics) == 3

        # Regenerated smaller plan -> stale targets deleted
        os.utime(os.path.join(logs, "prescriptions", "sprayer_2.json"),
                 (0, 0))
        _write_plan(logs, 2, 1)
        evs = [parse(x) for x in b.scan_artifacts()]
        assert [e.get("type") for e in evs].count("t-x-d-d") == 2


def test_udp_and_file_senders():
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", 0))
    rx.settimeout(2)
    port = rx.getsockname()[1]
    s = tak_cot.CotSender(f"udp://127.0.0.1:{port}", logger=lambda m: None)
    xml = tak_cot.spot_event("x", "X", 1.0, 2.0)
    assert s.send(xml)
    data, _ = rx.recvfrom(65535)
    assert data.decode() == xml
    s.close()
    rx.close()

    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "sub", "cot.log")
        f = tak_cot.CotSender(f"file://{path}", logger=lambda m: None)
        assert f.send(xml) and f.send(xml)
        f.close()
        assert open(path).read().count("<event ") == 2


def test_unreachable_tcp_is_harmless():
    s = tak_cot.CotSender("tcp://127.0.0.1:1", logger=lambda m: None)
    assert s.send(tak_cot.spot_event("x", "X", 1.0, 2.0)) is False
    assert s.send(tak_cot.spot_event("x", "X", 1.0, 2.0)) is False  # backoff


def test_sar_page_survives_link_outage():
    """The SURVIVOR FOUND GeoChat is sent once. If the TAK link is down at
    that moment (send() returns False), it must be retried, not lost."""
    from tak_bridge import SinkFanout

    class FlakySink:
        url = "tls://flaky:8089"

        def __init__(self):
            self.up, self.got = False, []

        def send(self, xml):
            if self.up:
                self.got.append(xml)
            return self.up

    logs = tempfile.mkdtemp()
    os.makedirs(os.path.join(logs, "sar"))
    with open(os.path.join(logs, "sar", "call.json"), "w") as f:
        json.dump({"found_by_instance": 2, "found_by_id": 3,
                   "lat": FIELD_ORIGIN_LAT, "lon": FIELD_ORIGIN_LON}, f)
    bridge = SwarmTakBridge([0, 1, 2, 3], [], logs_dir=logs, mission="sar")
    sink = FlakySink()
    fan = SinkFanout([sink], log=lambda m: None)

    fan.retry(bridge.one_shot)
    fan.send(bridge.scan_artifacts(), bridge.one_shot)   # link down: page lost?
    assert len(bridge.one_shot) == 1 and len(fan.pending[0]) == 1
    fan.send(bridge.scan_artifacts(), bridge.one_shot)   # rescan: no re-page
    assert len(fan.pending[0]) == 1

    sink.up = True                                       # link restored
    fan.retry(bridge.one_shot)
    pages = [x for x in sink.got if "SURVIVOR FOUND" in x]
    assert len(pages) == 1, pages
    assert not fan.pending[0] and not bridge.one_shot
    fan.retry(bridge.one_shot)                           # never sent twice
    assert len([x for x in sink.got if "SURVIVOR FOUND" in x]) == 1


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print(f"PASS {t.__name__}")
    print(f"{len(tests)} tests passed")
