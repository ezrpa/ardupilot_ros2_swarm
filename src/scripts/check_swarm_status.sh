#!/bin/bash
# Swarm status check, run from the host:  ./src/scripts/check_swarm_status.sh [NUM_DRONES]
#
# Checks what actually indicates health (instance i = 0..N-1, per
# start_drone_container.sh):
#   * MAVLink: a HEARTBEAT on MAVProxy's UDP output 14550+i (mode, armed).
#     MAVProxy pushes UDP; a TCP probe of that port is always "closed", which
#     is why the old version of this script reported healthy drones as failed.
#   * ROS 2: nodes in the drone's own domain, ROS_DOMAIN_ID = i+1.
#     The default domain 0 is empty by design.

NUM_DRONES=${1:-4}
if ! [[ "$NUM_DRONES" =~ ^[1-9][0-9]*$ ]]; then
    echo "usage: $0 [NUM_DRONES]   (positive integer, default 4)" >&2
    exit 2
fi

echo "=== ArduPilot Swarm Status: $NUM_DRONES drones ==="
echo

echo "=== Containers ==="
if command -v docker &>/dev/null; then
    docker ps --format "table {{.Names}}\t{{.Status}}" | grep -E "NAMES|drone|swarm|discovery|micro-ros"
else
    echo "docker not available"
fi
echo

echo "=== MAVLink heartbeat (UDP 14550+i, 3 s each) ==="
for i in $(seq 0 $((NUM_DRONES - 1))); do
    port=$((14550 + i))
    python3 - "$i" "$port" <<'PY'
import sys
inst, port = int(sys.argv[1]), int(sys.argv[2])
label = f"instance {inst} (UDP {port})"
try:
    from pymavlink import mavutil
except ImportError:
    print(f"{label}: SKIP - pymavlink not installed (pip install pymavlink)")
    sys.exit(0)
try:
    m = mavutil.mavlink_connection(f"udpin:0.0.0.0:{port}")
except OSError as e:
    print(f"{label}: BUSY - port held by another listener (QGroundControl?): {e.strerror}")
    sys.exit(0)
hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=3)
if hb is None:
    print(f"{label}: NO HEARTBEAT")
else:
    armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    mode = mavutil.mode_string_v10(hb)
    print(f"{label}: OK  sysid={m.target_system} mode={mode} armed={armed}")
PY
done
echo

echo "=== ROS 2 nodes per drone domain (ROS_DOMAIN_ID = i+1) ==="
if docker ps --format '{{.Names}}' 2>/dev/null | grep -qx swarm_controller; then
    for i in $(seq 0 $((NUM_DRONES - 1))); do
        nodes=$(docker exec swarm_controller bash -c \
            "source /opt/ros/humble/setup.bash && ROS_DOMAIN_ID=$((i + 1)) timeout 6 ros2 node list" 2>/dev/null \
            | tr '\n' ' ')
        echo "instance $i (domain $((i + 1))): ${nodes:-NO NODES}"
    done
else
    echo "swarm_controller not running"
fi
echo

echo "=== Recent errors ==="
docker logs swarm_controller --tail=50 2>&1 | grep -i error | tail -3 || true
for i in $(seq 0 $((NUM_DRONES - 1))); do
    docker logs "drone-ardu-$i" --tail=50 2>&1 | grep -i error | tail -2 | sed "s/^/drone-ardu-$i: /"
done
echo

echo "=== Useful commands ==="
echo "Controller logs:  docker logs -f swarm_controller"
echo "Drone logs:       docker logs -f drone-ardu-<i>"
echo "Enter controller: docker exec -it swarm_controller bash"
echo "Stop swarm:       ./stop_swarm.sh"
echo "Restart swarm:    ./start_swarm.sh $NUM_DRONES"
