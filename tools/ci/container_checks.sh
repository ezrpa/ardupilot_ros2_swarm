#!/bin/bash
# Checks that need the swarm_controller image (ROS 2 Humble + ardupilot_msgs).
# Used by CI and runnable locally:
#   docker run --rm -v "$PWD":/repo:ro --entrypoint bash \
#       ardupilot_ros2_swarm-swarm_controller /repo/tools/ci/container_checks.sh
# Pass --regen-install-layout to rewrite tests/golden/install_layout.txt
# (prints it; the repo mount is read-only).
set -euo pipefail
set +u  # ROS setup scripts reference unset variables
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
set -u
REPO=${REPO:-/repo}

echo "== colcon build (plain install, from a copy)"
WS=$(mktemp -d)
mkdir -p "$WS/src" && cp -r "$REPO/src/swarm_control" "$WS/src/"
find "$WS/src" -name __pycache__ -prune -exec rm -rf {} +
(cd "$WS" && colcon build --packages-select swarm_control > "$WS/build.log" 2>&1) \
    || { tail -40 "$WS/build.log"; exit 1; }
set +u; source "$WS/install/setup.bash"; set -u

echo "== install layout"
LAYOUT=$(cd "$WS/install/swarm_control" && find . -type f \
    | grep -v -E "__pycache__|\.pyc$|colcon|hook|package\.(dsv|sh|bash|zsh|ps1|xml)$|local_setup" | sort)
if [ "${1:-}" = "--regen-install-layout" ]; then echo "$LAYOUT"; exit 0; fi
diff <(echo "$LAYOUT") "$REPO/tests/golden/install_layout.txt" \
    || { echo "install layout changed (regen with --regen-install-layout if intended)"; exit 1; }

echo "== installed executables"
L="$WS/install/swarm_control/lib/swarm_control"
cd /
for x in prescription_planner tak_bridge tak_viewer; do
    python3 "$L/$x.py" --help > /dev/null
done
for x in base_drone_dds camera_drone_dds sprayer_drone_dds search_drone_dds; do
    python3 -c "import runpy; runpy.run_path('$L/$x.py', run_name='smoke')"
done
echo "ok"

echo "== pytest (ros tests included)"
cd "$REPO"
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider
