#!/usr/bin/env python3
"""SearchDrone node: SAR search, confirm, call, rally.

Executable entry point only (installed as lib/swarm_control/search_drone_dds.py); the code
lives in swarm_control.nodes.search_drone.
"""
import os
import sys

# From a source checkout (or a colcon --symlink-install, whose lib/ entries
# symlink back here) put src/swarm_control on the path so the package
# imports. A plain colcon install already has it in site-packages.
_SRC = os.path.join(os.path.dirname(os.path.realpath(__file__)), "..")
if os.path.isfile(os.path.join(_SRC, "swarm_control", "__init__.py")):
    sys.path.insert(0, os.path.abspath(_SRC))

from swarm_control.nodes.search_drone import main  # noqa: E402

if __name__ == "__main__":
    main()
