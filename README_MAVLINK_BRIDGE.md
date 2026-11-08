# MAVLink Bridge — Hylio-Style Companion-Computer Simulation (Plan)

Status: **planned, not yet implemented.**

## Why

Real Hylio spray drones run ArduPilot-derived firmware (their documented mode list —
RTL, Auto, Guided, Land, Brake, Loiter, Altitude Hold — is ArduCopter's) but expose
**no DDS**. Hylio publishes no SDK or developer API; AgroSol GCS is proprietary. The
practical integration path is the standard ArduPilot companion-computer pattern:

```
Hylio drone (ArduPilot fw) ── RFD900x serial radio ── Jetson (OBC) ── LAN ── QGC / operators
                                    MAVLink                │
                                                 DDS→MAVLink wrapper
                                                 (our existing ROS2 swarm nodes, unchanged)
```

- Hylio sells the RFD900x as their fleet telemetry radio — a plain MAVLink serial
  modem (57600 baud default, TTL serial). No Hylio-specific protocol in the way.
- Control is MAVLink **GUIDED mode** from the Jetson; the radio serial is attached to
  the Jetson and can be exposed over LAN (ser2net-style) for QGC via the drone's IP.
- Relevant guides: ArduPilot companion computers
  (https://ardupilot.org/dev/docs/companion-computers.html), MAVLink developer guide
  (https://mavlink.io/en/). Anything firmware-locked (e.g. whether Hylio's build
  accepts `SET_POSITION_TARGET_GLOBAL_INT` unmodified) needs confirmation from Hylio
  support. Power management for the Jetson + custom mounting are hardware-side tasks
  outside this simulation.

To de-risk that deployment, the simulation will control selected **sprayer** drones
purely over MAVLink — no micro-ROS agent, firmware DDS disabled — via a new
`mavlink_bridge.py` ROS2 node that re-exposes the exact `/ap/*` surface the swarm
nodes already use. The bridge is the simulated Jetson-side wrapper; `BaseDrone` /
`SprayerDrone` mission logic runs unchanged. Mappers stay on DDS. Selection is
per-drone via `MAVLINK_INSTANCES` (must be a subset of `SPRAYER_INSTANCES`).

## Verified facts the design rests on

- BaseDrone's full DDS surface: services `/ap/arm_motors` (ArmMotors),
  `/ap/mode_switch` (ModeSwitch), `/ap/experimental/takeoff` (Takeoff, hardcoded
  topic); pub `/ap/cmd_gps_pose` (GlobalPosition, **raw degrees/metres**, frame 6,
  type_mask `0b0000111111111000`); subs `/ap/status`, `/ap/pose/filtered`,
  `/ap/gps_global_origin/filtered` (QoS BEST_EFFORT / VOLATILE / depth 10).
- `Status.msg` fields: header, vehicle_type (APM_ARDUCOPTER=2), armed, mode, flying,
  external_control, failsafe[]. BaseDrone reads only `.armed` / `.mode`; first
  receipt drives CONNECTING→CONNECTED.
- SprayerDrone currently opens its own pymavlink TCP to SERIAL1 `5762+10*i` solely
  for `MAV_CMD_DO_AUX_FUNCTION(218, param1=15, param2=2/0)`. SITL allows **one TCP
  client per serial port**, so in bridge mode the pump command must route through
  the bridge (faithful anyway — on real hardware everything shares one radio link).
  `spray_visualizer.py` uses SERIAL2 `5763+10*i` — untouched.
- All containers are `network_mode: host`; all per-drone ROS2 nodes run inside
  `swarm_controller`, spawned by `compose_decentralized.launch.py` with per-Node
  `additional_env ROS_DOMAIN_ID=i+1`, staggered 5 s, `respawn=True`. pymavlink is in
  the image. `dds_swarm.parm` already sets `SERIAL1_PROTOCOL=2` and `FS_GCS_ENABLE=0`.
- `CMakeLists.txt` installs `scripts/*.py` via GLOB — a new executable script needs
  no build-config change.

## Phase 1 — `src/swarm_control/scripts/mavlink_bridge.py` (new)

`MavlinkBridge(Node)`; params: `instance` (int), `mavlink_tcp_port` (default 0 →
`5762+10*instance`), `stream_rate_hz` (10.0).

Startup (before creating any ROS services — their existence is BaseDrone's readiness
gate):
1. `mavutil.mavlink_connection('tcp:127.0.0.1:<port>', source_system=255,
   source_component=191 /* MAV_COMP_ID_ONBOARD_COMPUTER */)`; on failure
   `sys.exit(1)` and let launch respawn (this is the boot-ordering mechanism).
2. `wait_heartbeat(timeout=30)` (exit(1) on timeout); warn if
   `target_system != instance+1`.
3. `MAV_CMD_SET_MESSAGE_INTERVAL` (511) for GLOBAL_POSITION_INT(33),
   LOCAL_POSITION_NED(32), EXTENDED_SYS_STATE(245) at `stream_rate_hz`; **also**
   `REQUEST_DATA_STREAM` (POSITION + EXTENDED_STATUS) as fallback — a raw serial
   link streams nothing until asked.
4. `MAV_CMD_REQUEST_MESSAGE` (512) for GPS_GLOBAL_ORIGIN(49); re-request every 5 s
   until first received; accept HOME_POSITION(242) as fallback; republish the cached
   origin at 1 Hz (AP_DDS behavior; BaseDrone ignores the (0,0) sentinel).
5. Start the reader thread, then create services/subs/pubs.

Threading: one daemon reader thread is the sole `recv_match` caller; it updates
state (heartbeat, LOCAL_POSITION_NED, origin, EXTENDED_SYS_STATE) under a state
lock, records COMMAND_ACKs keyed by `msg.command` (not arrival order) and sets an
event. All sends go through a send lock. `_send_and_wait_ack(cmd, *params,
timeout=3.0, retries=2)`. Publishing to ROS from the reader thread is fine. A 1 Hz
timer sends a companion HEARTBEAT.

ROS surface (publisher QoS: BEST_EFFORT / VOLATILE / 10 — exact AP_DDS mirror):

| ROS entity | MAVLink mapping |
|---|---|
| srv `/ap/arm_motors` | `MAV_CMD_COMPONENT_ARM_DISARM(400)`, param1=1/0; `result = (ack==ACCEPTED)`. No force flag; no bridge-side retries — BaseDrone's TimeoutRetry owns retrying (EKF-warmup denials map to `result=False`, same as DDS). |
| srv `/ap/mode_switch` | `MAV_CMD_DO_SET_MODE(176)`, param1=1 (custom-mode-enabled), param2=float(mode); `status = ack accepted`, `curr_mode = latest heartbeat custom_mode & 0xFF`. |
| srv `/ap/experimental/takeoff` | `MAV_CMD_NAV_TAKEOFF(22)`, param7=alt; `status = ack accepted`. |
| sub `/ap/cmd_gps_pose` | `set_position_target_global_int_send(..., frame=msg.coordinate_frame, type_mask passthrough, int(lat*1e7), int(lon*1e7), alt_m, ...)`. **The degrees→degE7 ×1e7 scaling lives HERE** — the exact inverse of the AP_DDS degE7 bug documented in CLAUDE.md; loud comment required. |
| pub `/ap/status` | On HEARTBEAT: `armed = base_mode & 0x80`, `mode = custom_mode & 0xFF`, `flying = EXTENDED_SYS_STATE.landed_state == IN_AIR` (False if unseen), `external_control=True`, `vehicle_type=2`. |
| pub `/ap/pose/filtered` | On LOCAL_POSITION_NED, NED→ENU: `x=msg.y, y=msg.x, z=-msg.z`; identity orientation. |
| pub `/ap/gps_global_origin/filtered` | `lat/1e7, lon/1e7, alt_mm/1000`. |
| sub `/ap/sprayer_cmd` (std_msgs/Bool, default QoS) | `MAV_CMD_DO_AUX_FUNCTION(218)`, param1=15, param2=2/0; fire-and-forget (matches current behavior). |

Verify standalone (before any wiring): `./start_swarm.sh 1`, then
`docker stop micro-ros-agent-drone-0`, then inside swarm_controller
(`ROS_DOMAIN_ID=1`): run the bridge with `instance:=0`, check
`ros2 topic hz /ap/status /ap/pose/filtered`, echo the origin, service-call
arm / mode 4 / takeoff — **confirm real `rel_alt` gain via pymavlink on UDP 14550**.
Then run `base_drone_dds.py` against it and compare the GPS track against commanded
waypoints.

## Phase 2 — Sprayer bridge mode + launch/orchestration wiring

- `sprayer_drone_dds.py`: new param `use_mavlink_bridge` (False). When true: publish
  `Bool` on `/ap/sprayer_cmd` from `set_spray` instead of opening pymavlink (keep
  `mavlink_tcp_port=0` as defense-in-depth). `from std_msgs.msg import Bool`.
- `compose_decentralized.launch.py`: new `mavlink_instances` launch arg parsed like
  `sprayer_instances`; validate subset (warn + drop non-sprayer members). For each
  bridged sprayer: sprayer Node gets `mavlink_tcp_port: 0, use_mavlink_bridge: True`;
  add a `mavlink_bridge.py` Node (`respawn=True, respawn_delay=5.0`, params
  `instance`, port `5762+10*i`) in the **same `ROS_DOMAIN_ID` additional_env** and
  same TimerAction, listed before the drone node.
- `compose.yaml`: add `- MAVLINK_INSTANCES=${MAVLINK_INSTANCES:-}` to
  swarm_controller env (next to SPRAYER_INSTANCES).
- `src/scripts/start_swarm_controller.sh`: pass
  `mavlink_instances:="${MAVLINK_INSTANCES:-}"` to `ros2 launch`.
- `start_swarm.sh`: per-instance `IS_MAVLINK` via the same case-pattern idiom as
  `DRONE_ROLE`; skip the micro-ros-agent `docker compose run` when set; pass
  `-e MAVLINK_BRIDGE=$IS_MAVLINK` to the drone_ardu run; update the usage comment.

Verify: `SPRAYER_INSTANCES="1" MAVLINK_INSTANCES="1" ./start_swarm.sh 2` — no
`micro-ros-agent-drone-1` container, bridge node up in domain 2, both drones gain
real rel_alt (UDP 14550/14551), mapper unaffected.

## Phase 3 — Firmware fidelity

`src/scripts/start_drone_container.sh`: after the sprayer.parm append,

```sh
if [ "${MAVLINK_BRIDGE:-0}" = "1" ]; then echo "DDS_ENABLE 0" >> instance_dds.parm; fi
```

Last-append wins; the parm file is regenerated each boot so reverting is automatic.
Keeps the sim honest (zero DDS traffic for these drones, matching real Hylio
firmware) and silences agent-connection retries.

## Phase 4 — Docs

CLAUDE.md: new subsection under "Sprayer Drones" — "MAVLink-bridged sprayers
(Hylio-style companion mode)": `MAVLINK_INSTANCES` semantics (subset of
`SPRAYER_INSTANCES`), bridge = simulated Jetson on SERIAL1 TCP `5762+10*i`, the
×1e7 scaling-inversion note, one-TCP-client rule (`/ap/sprayer_cmd` routing;
spray_visualizer keeps SERIAL2), no micro-ros-agent for these instances.

## Phase 5 — A/B swarm verification (success criterion)

```bash
SWARM_WORLD=swarm_drone_2m4s.sdf ./start_gazebo.sh &
SPRAYER_INSTANCES="2 3 4 5" MAVLINK_INSTANCES="4 5" USE_GAZEBO=1 ./start_swarm.sh 6
python3 src/scripts/spray_visualizer.py --instances 2 3 4 5 &
```

2 DDS mappers + 2 DDS sprayers (control group) + 2 MAVLink sprayers. Check:

1. Agents exist only for instances 0–3 (`docker ps`).
2. All 6 gain real rel_alt (pymavlink UDP 14550..14555).
3. GPS tracks of instance 3 (DDS sprayer) vs 4/5 (MAVLink) both match commanded
   prescription waypoints within a few metres.
4. Prescription pipeline end-to-end: `logs/prescriptions/sprayer_{4,5}.json`
   written, bridged sprayers launch only after loading them, plumes appear
   (visualizer on SERIAL2 unaffected).
5. QGC still connects on `5760+10*i` for a MAVLink instance.
6. Mapper mosaic still stitches (DDS side untouched).

## Risks / gotchas

- **One TCP client on `5762+10*i`**: the bridge is sole owner; on crash, SITL frees
  the socket after close — the 5 s respawn delay covers it (an occasional extra
  respawn cycle is normal).
- **Empty-stream trap**: without SET_MESSAGE_INTERVAL / REQUEST_DATA_STREAM the link
  is silent and BaseDrone hangs in `wait_for_position_estimate` — check
  `ros2 topic hz /ap/pose/filtered` first in Phase 1.
- **degE7 scaling** is the most likely correctness bug (missed ×1e7 → Null Island;
  doubled → int32 overflow). The Phase 1 track-vs-commanded comparison catches both.
- **Domain mismatch** (bridge not in its drone's ROS_DOMAIN_ID) manifests as endless
  "arm service not available".
- Ack matching keys on `msg.command`; heartbeat `custom_mode` is uint32 → mask to
  uint8 for `curr_mode` / `Status.mode`.

## Files to be touched

- NEW `src/swarm_control/scripts/mavlink_bridge.py`
- `src/swarm_control/scripts/sprayer_drone_dds.py`
- `src/swarm_control/launch/compose_decentralized.launch.py`
- `compose.yaml`, `src/scripts/start_swarm_controller.sh`, `start_swarm.sh`
- `src/scripts/start_drone_container.sh`
- `CLAUDE.md`

No CMakeLists / setup.py changes (GLOB install of `scripts/*.py` verified).