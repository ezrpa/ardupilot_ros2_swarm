#!/bin/bash

set -e

# Spawn heading in degrees. Overridable so a vehicle can be started facing a
# known direction to validate heading-dependent code (e.g. the body-frame ->
# north/east rotation in camera_processor.py).
YAW=${YAW:-0}

# Environment variables from Docker Compose / start_swarm.sh
INSTANCE=${INSTANCE:-0}
NUM_DRONES=${NUM_DRONES:-2}

# Ports, IDs and the SITL home (LAT_OFFSET/LON_OFFSET/ALT_BASE) come from the
# swarm's one contract module, mounted read-only at /root/swarm_core (see
# src/swarm_control/swarm_control/core/config.py for the Gazebo / corners /
# built-in spacing rules). Assigned first so a failure stops the script.
INSTANCE_ENV=$(python3 /root/swarm_core/config.py env --instance "${INSTANCE}" \
    --gazebo "${USE_GAZEBO:-0}" --layout "${SPAWN_LAYOUT:-}" --area "${SPAWN_AREA_M:-120}")
eval "${INSTANCE_ENV}"

# Create instance directory for parameter isolation
mkdir -p /sitl/instance_"${INSTANCE}"
cd /sitl/instance_"${INSTANCE}"

# UPDATE parameter file to set correct DDS port, domain ID, and sys_mav_id:
touch instance_dds.parm
echo -e "DDS_UDP_PORT=${MICRO_ROS_AGENT_PORT}\nSYSID_THISMAV=${SYSID_THISMAV}\nDDS_DOMAIN_ID=${ROS_DOMAIN_ID}\n" > instance_dds.parm
cat /config/dds_swarm.parm >> instance_dds.parm

# Gazebo mode: sim_vehicle passes our file as the ONLY --defaults (frame defaults
# are not merged), so append gazebo-iris frame params AFTER dds_swarm.parm — the
# Gazebo iris model is an X-frame quad (FRAME_TYPE 1); dds_swarm.parm's
# FRAME_TYPE 0 (plus) would break motor mixing and the drones never lift off.
if [ "${USE_GAZEBO:-0}" = "1" ]; then
    cat /root/ardu_ws/src/ardupilot/Tools/autotest/default_params/gazebo-iris.parm >> instance_dds.parm
fi

# Sprayer role: enable ArduPilot's AC_Sprayer (pump/spinner outputs, speed-
# proportional flow). Appended last so it wins over earlier defaults.
if [ "${DRONE_ROLE:-mapper}" = "sprayer" ] && [ -f /config/sprayer.parm ]; then
    echo "Instance ${INSTANCE}: sprayer role - applying sprayer.parm"
    cat /config/sprayer.parm >> instance_dds.parm
fi
cat instance_dds.parm

# Start ArduPilot SITL
# USE_GAZEBO=1 switches physics from SITL's built-in model to Gazebo via the
# JSON FDM backend: SITL instance N exchanges FDM data with the ArduPilotPlugin
# listening on host port 9002 + 10*N (see src/custom_gz/worlds/swarm_drone.sdf).
# Gazebo must be running on the host BEFORE the drones start, else SITL blocks.
if [ "${USE_GAZEBO:-0}" = "1" ]; then
    FRAME_ARGS="--frame gazebo-iris --model JSON"
else
    FRAME_ARGS=""
fi

cd -
sim_vehicle.py \
    --vehicle ArduCopter \
    --no-rebuild \
    ${FRAME_ARGS} \
    --out 127.0.0.1:${MAVPROXY_UDP_PORT} \
    --out 127.0.0.1:${DDS_UDP_PORT} \
    --out 127.0.0.1:${MAVLINK_TCP_PORT} \
    --custom-location="${LAT_OFFSET}","${LON_OFFSET}",${ALT_BASE},${YAW} \
    --sysid ${SYSID_THISMAV} \
    --wipe False \
    --instance "${INSTANCE}" \
    --add-param-file=/sitl/instance_"${INSTANCE}"/instance_dds.parm \
    --enable-DDS

