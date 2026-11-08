#!/usr/bin/env python3
"""Dev demo: plot the waypoint split.

Executable entry point only (installed as lib/swarm_control/points_distributor.py); the code
lives in swarm_control.mission.points_distributor.
"""
import os
import sys

# From a source checkout (or a colcon --symlink-install, whose lib/ entries
# symlink back here) put src/swarm_control on the path so the package
# imports. A plain colcon install already has it in site-packages.
_SRC = os.path.join(os.path.dirname(os.path.realpath(__file__)), "..")
if os.path.isfile(os.path.join(_SRC, "swarm_control", "__init__.py")):
    sys.path.insert(0, os.path.abspath(_SRC))

import runpy  # noqa: E402

if __name__ == "__main__":
    runpy.run_module("swarm_control.mission.points_distributor", run_name="__main__")
