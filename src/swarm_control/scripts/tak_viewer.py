#!/usr/bin/env python3
"""Browser map of the CoT feed (localhost:8088).

Executable entry point only (installed as lib/swarm_control/tak_viewer.py); the code
lives in swarm_control.tak.viewer.
"""
import os
import sys

# From a source checkout (or a colcon --symlink-install, whose lib/ entries
# symlink back here) put src/swarm_control on the path so the package
# imports. A plain colcon install already has it in site-packages.
_SRC = os.path.join(os.path.dirname(os.path.realpath(__file__)), "..")
if os.path.isfile(os.path.join(_SRC, "swarm_control", "__init__.py")):
    sys.path.insert(0, os.path.abspath(_SRC))

from swarm_control.tak.viewer import main  # noqa: E402

if __name__ == "__main__":
    main()
