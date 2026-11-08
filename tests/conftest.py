"""Shared pytest setup: put the swarm_control package on the path, provide
repo paths, and gate the `ros` / `sim` markers (see pytest.ini)."""
import importlib.util
import os
import sys

import pytest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG_ROOT = os.path.join(REPO, "src", "swarm_control")
if PKG_ROOT not in sys.path:
    sys.path.insert(0, PKG_ROOT)

HAVE_ROS = all(importlib.util.find_spec(m) is not None
               for m in ("rclpy", "ardupilot_msgs"))


def pytest_addoption(parser):
    parser.addoption("--sim", action="store_true",
                     help="run tests that need a live swarm / TAK viewer")


def pytest_collection_modifyitems(config, items):
    skip_ros = pytest.mark.skip(reason="needs ROS: run inside the swarm_controller image")
    skip_sim = pytest.mark.skip(reason="needs a live swarm: pass --sim")
    for item in items:
        if "ros" in item.keywords and not HAVE_ROS:
            item.add_marker(skip_ros)
        if "sim" in item.keywords and not config.getoption("--sim"):
            item.add_marker(skip_sim)
