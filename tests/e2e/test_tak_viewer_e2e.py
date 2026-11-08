"""End-to-end: drive the live TAK viewer through a SAR mission (4 stages).

Needs a running SAR swarm and viewer (./start_tak_sim.sh 4) plus Playwright
with Chromium. Skipped unless pytest is given --sim:

    python3 -m pytest tests/e2e --sim
    TAK_VIEWER_URL=http://VM:8088 python3 -m pytest tests/e2e --sim

The checks themselves live in check_tak_viewer.py (also runnable directly).
"""
import os
import subprocess
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))


@pytest.mark.sim
def test_tak_viewer_sar_mission():
    pytest.importorskip("playwright")
    url = os.environ.get("TAK_VIEWER_URL", "http://localhost:8088")
    drones = os.environ.get("TAK_VIEWER_DRONES", "4")
    r = subprocess.run([sys.executable, os.path.join(HERE, "check_tak_viewer.py"),
                        "--url", url, "--drones", drones],
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stdout[-3000:] + r.stderr[-2000:]
