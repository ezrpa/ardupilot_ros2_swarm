#!/bin/bash
# TAK simulation: run a mission WITHOUT Gazebo and watch it as a TAK picture.
#
#   ./start_tak_sim.sh 4                      # search & rescue, 4 drones
#   SWARM_MISSION=survey ./start_tak_sim.sh 4 # weed survey instead
#   then open http://localhost:8088
#
# What replaces Gazebo:
#   physics  -> SITL's built-in multicopter model (USE_GAZEBO=0)
#   camera   -> synthetic_camera.py renders nadir frames from the worlds'
#               ground truth (SIM_SENSORS=synthetic); the real HSV detectors,
#               georeferencing, planner and SAR call logic run unchanged
#   visuals  -> tak_bridge.py sends CoT (PyTAK) to ATAK multicast, and
#               tak_viewer.py (PyTAK receiver + Leaflet) maps it in a browser
#
# Any ATAK/WinTAK/iTAK on the LAN sees the same picture on 239.2.3.1:6969.
# Stop with ./stop_swarm.sh.
set -e
cd "$(dirname "$0")"

NUM_DRONES=${1:-4}
if ! [[ "$NUM_DRONES" =~ ^[1-9][0-9]*$ ]]; then
    echo "error: drone count must be a positive integer, got '$NUM_DRONES'" >&2
    echo "usage: $0 <num_drones>   e.g. $0 4" >&2
    exit 2
fi
export SWARM_MISSION=${SWARM_MISSION:-sar}
export TAK_URL=${TAK_URL:-udp://239.2.3.1:6969}
export TAK_TRANSPORT=${TAK_TRANSPORT:-auto}
export SIM_SENSORS=synthetic
export USE_GAZEBO=0
# SAR: drones launch from separate corners and run a coordinated grid
# search (bfs | dfs | strips)
if [ "${SWARM_MISSION}" = "sar" ]; then
    export SAR_SEARCH=${SAR_SEARCH:-bfs}
    export SPAWN_LAYOUT=${SPAWN_LAYOUT:-corners}
    export SIM_SCENARIO=${SIM_SCENARIO:-sar_scenario}
fi
VIEWER_PORT=${TAK_VIEWER_PORT:-8088}

echo "TAK sim: mission=${SWARM_MISSION} drones=${NUM_DRONES} search=${SAR_SEARCH:-n/a} cot=${TAK_URL}"

# Rebuild only if the PyTAK layer is missing (first run after this change).
if ! docker run --rm --entrypoint python3 ardupilot_ros2_swarm-swarm_controller \
        -c "import pytak" >/dev/null 2>&1; then
    echo "Building images (adds PyTAK)..."
    docker compose build swarm_controller
fi
docker compose --profile tak build tak_viewer >/dev/null

docker compose --profile tak up -d tak_viewer
# Fresh controller every run: `compose up` keeps an unchanged container, which
# would skip the log archiving (a stale logs/sar/call.json makes every new
# SearchDrone rally on the OLD survivor) and keep the finished launch alive.
docker compose rm -sf swarm_controller >/dev/null 2>&1 || true
./start_swarm.sh "${NUM_DRONES}"

echo
echo "TAK picture:  http://localhost:${VIEWER_PORT}"
echo "Mission log:  docker logs -f swarm_controller"
echo "Stop:         ./stop_swarm.sh"
