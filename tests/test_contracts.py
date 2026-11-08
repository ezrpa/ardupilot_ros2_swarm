#!/usr/bin/env python3
"""Cross-component contracts: core/config.py is the single owner, and
everything that cannot import it (shell, SDF assets) is checked against it.

Before the refactor these values were copied into ~14 files with "must match"
comments and nothing enforcing them; one copy (check_swarm_status.sh) and a
dead launch parameter had already drifted to a wrong port formula.

Run directly:  python3 tests/test_contracts.py
"""
import json
import os
import re
import sys
import xml.etree.ElementTree as ET

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "src", "swarm_control"))
from swarm_control.core import config  # noqa: E402

MODELS = os.path.join(REPO, "src", "custom_gz", "models")
CONFIG_PY = os.path.join("src", "swarm_control", "swarm_control", "core", "config.py")
CODE_DIRS = ("src", "tests", "tools", "start_swarm.sh", "start_gazebo.sh",
             "start_tak_sim.sh", "stop_swarm.sh")


def _code_lines():
    """(relpath, lineno, line) for code lines (comments stripped) of .py/.sh."""
    for top in CODE_DIRS:
        root = os.path.join(REPO, top)
        paths = [root] if os.path.isfile(root) else [
            os.path.join(dp, f) for dp, dn, fs in os.walk(root)
            if "__pycache__" not in dp for f in fs]
        for p in paths:
            rel = os.path.relpath(p, REPO)
            if not p.endswith((".py", ".sh")) or rel in (CONFIG_PY, "tests/test_contracts.py"):
                continue
            in_doc = False
            for i, line in enumerate(open(p, errors="replace"), 1):
                s = line.strip()
                if s.count('"""') % 2 == 1:
                    in_doc = not in_doc
                    continue
                if in_doc or s.startswith("#"):
                    continue
                yield rel, i, line.split(" #")[0]


def test_field_origin_has_one_owner():
    hits = [f"{r}:{i}" for r, i, l in _code_lines()
            if re.search(r"40\.072842|-105\.230575", l)]
    assert not hits, "field origin literal outside core/config.py:\n" + "\n".join(hits)


def test_port_and_domain_formulas_have_one_owner():
    """No code re-derives a per-instance port: base + k*instance arithmetic
    on the contract's base ports appears only in config.py."""
    bases = r"(14550|5760|5762|5763|5501|9002|2019|5600)"
    pat = re.compile(rf"\b{bases}\s*\+|\+\s*{bases}\b")
    hits = [f"{r}:{i}: {l.strip()}" for r, i, l in _code_lines() if pat.search(l)]
    assert not hits, "port formula outside core/config.py:\n" + "\n".join(hits)


def test_shell_env_matches_recorded_bash_behaviour():
    """config.instance_env reproduces, string for string, what
    start_drone_container.sh computed in bash before it read config.py."""
    golden = json.load(open(os.path.join(REPO, "tests", "golden", "drone_env.json")))
    for case, exp in golden.items():
        d = dict(kv.split("=") for kv in case.split())
        env = config.instance_env(int(d["instance"]), d["gazebo"] == "1",
                                  "" if d["layout"] == "default" else d["layout"])
        got = {k: str(v) for k, v in env.items()}
        for k, v in exp.items():
            assert got[k] == v, (case, k, v, got[k])


def _sdf(name):
    return ET.parse(os.path.join(MODELS, name, "model.sdf")).getroot()


def test_gazebo_fdm_ports_match_config():
    for inst in range(6):
        name = "base_drone" if inst == 0 else f"base_drone_{inst}"
        ports = [int(e.text) for e in _sdf(name).iter("fdm_port_in")]
        assert ports == [config.gazebo_fdm_port(inst)], (name, ports)


def test_camera_models_match_config():
    for inst in range(4):
        name = "base_drone_cam" if inst == 0 else f"base_drone_cam_{inst}"
        root = _sdf(name)
        cam = next(root.iter("camera"))
        assert float(cam.find("horizontal_fov").text) == config.CAMERA_HFOV_RAD, name
        img = cam.find("image")
        assert int(img.find("width").text) == config.CAMERA_WIDTH_PX, name
        assert int(img.find("height").text) == config.CAMERA_HEIGHT_PX, name
        ports = [int(e.text) for e in root.iter("udp_port")]
        assert ports == [config.camera_udp_port(inst)], (name, ports)
        base = "base_drone" if inst == 0 else f"base_drone_{inst}"
        assert any(u.text.strip() == f"model://{base}" for u in root.iter("uri")), name


def test_ros_domain_and_sysid_are_instance_plus_one():
    for i in range(8):
        assert config.ros_domain_id(i) == i + 1 and config.sysid(i) == i + 1


if __name__ == "__main__":
    failed = 0
    tests = sorted(n for n in dir() if n.startswith("test_"))
    for name in tests:
        try:
            globals()[name]()
            print(f"PASS {name}")
        except Exception as e:  # noqa: BLE001
            failed += 1
            print(f"FAIL {name}: {type(e).__name__}: {e}")
    print(f"\n{len(tests) - failed}/{len(tests)} passed")
    sys.exit(1 if failed else 0)
