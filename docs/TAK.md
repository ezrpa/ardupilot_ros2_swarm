# TAK Integration (and how it fits with AFSIM)

The swarm publishes its live picture to the **Team Awareness Kit** (ATAK,
WinTAK, iTAK, TAK Server, FreeTAKServer) as **Cursor-on-Target (CoT)** XML.
Anyone on a TAK network sees the drones fly, sees weeds appear as the mappers
detect them, and sees the per-sprayer targets once the prescription is planned.

```
 ArduPilot SITL ──DDS──► micro-ROS agent ──► ROS2 domain i+1 ─┐
   (x N drones)                                                │ /ap/geopose/filtered
                                                               │ /ap/status, /ap/battery
 CameraDrone sidecars  logs/drone_<id>/wp_*.json ──────────────┤
 prescription_planner  logs/prescriptions/sprayer_<i>.json ────┤
                                                               ▼
                                         tak_bridge.py (swarm_controller)
                                                               │ CoT XML
                    ┌──────────────────────┬──────────────────┼───────────────┐
                    ▼                      ▼                  ▼               ▼
           udp://239.2.3.1:6969    tcp://server:8087   tls://server:8089   file://...
           (ATAK mesh SA, LAN)     (TAK Server plain)  (TAK Server mTLS)   (debug/replay)
```

## TAK simulation without Gazebo (PyTAK + browser map)

```bash
./start_tak_sim.sh 4                       # search & rescue, 4 drones
SWARM_MISSION=survey ./start_tak_sim.sh 4  # weed survey (wired, not yet run end to end)
# open http://localhost:8088               (stop: ./stop_swarm.sh)
# Verified 2026-11-08, SAR, 4 drones: call raised 50 s into the search,
# 0.03 m from ground truth, all 4 on scene ~30 s later.
```

![TAK sim: SAR mission, all units on scene](images/tak_sim_sar.jpg)

**SAR in the TAK sim** (defaults set by `start_tak_sim.sh`):

- *Separate launch points* (`SPAWN_LAYOUT=corners`): drone i homes at a corner
  of the 120 m area (inset 8 m), then edge midpoints
  (`start_drone_container.sh`, matching `sar_search.launch_points`).
- *Coordinated grid search* (`SAR_SEARCH=bfs|dfs|strips`, `sar_search.py`):
  7x7 cells of ~17 m. **bfs**: from its current cell each drone takes the
  nearest unclaimed cell (breadth-first over the 4-connected grid, ties keep
  heading), so the fronts sweep inward from the corners. **dfs**: depth-first
  from the launch cell, backtracking over its own stack. Cells are claimed
  with O_EXCL files in `logs/sar/claims/`, so no cell is searched twice. The
  bridge sends each claimed cell as a CoT rectangle in the drone's colour.
- *Computer vision* (`camera_processor.detect_person`): a wide hi-vis HSV mask
  finds candidates. Each blob is then measured in metres (GSD = AGL / focal px)
  and scored as a person lying down (length about 1.6 m, width about 0.75 m
  with arms, solidity). Blobs cut by the frame edge are rejected, because a
  clipped kayak can look exactly person-sized. Detections carry
  `confidence`, `length_m` and `width_m`. Rejections go to the sidecar's
  `person_rejected` with a reason. A sighting must be re-detected from 15 m
  before the call.
- *Harder imagery* (`SIM_SCENARIO=sar_scenario`, `SIM_REALISM=1`):
  `models/sar_scenario/ground_truth.json` holds the casualty (seed 33,
  mid-field) and orange decoys (3 cones, a tarp, a kayak, a backpack). The
  renderer adds ground texture, tree shadows, an articulated casualty,
  exposure changes, blur and sensor noise. The Gazebo worlds are untouched.
  Bench: person found in 80/80 fully-in-view frames at 30 and 15 m, and
  0 decoy hits in 480 frames.
- *Browser check*: `python3 tests/e2e/check_tak_viewer.py [--url ...]`
  (Playwright, headless Chromium). It passes through four stages (picture ->
  searching -> found -> on scene) and saves screenshots to
  `test-results/tak_viewer/`.

No Gazebo, no ATAK install needed. What stands in for Gazebo:

| Gazebo provided | Replaced by |
|---|---|
| physics | SITL's built-in multicopter model (`USE_GAZEBO=0`) |
| camera images | `synthetic_camera.py` (`SIM_SENSORS=synthetic`): renders a nadir frame from the drone's measured pose + heading and the worlds' ground truth (`weed_field/` and `missing_person/ground_truth.json`, mounted at `/root/sim_models`). The frame goes through the **unchanged** `CameraProcessor` HSV detectors and georeferencing, so the planner, the SAR confirm/call/rally logic and the bridge behave exactly as with Gazebo. Rendering is the exact inverse of the georef model: no lens or GPS noise. |
| 3D view | `tak_viewer.py` + `src/swarm_control/viewer/index.html`: a PyTAK receiver on the CoT feed serving a Leaflet satellite map: search area, per-drone tracks, accumulated camera-coverage footprints and % searched, SAR state per unit, the CASEVAC survivor marker with its confirmation frame, responder lines, and an event/GeoChat feed |

The viewer runs as compose service `tak_viewer` (profile `tak`, host
networking, port `TAK_VIEWER_PORT`=8088) from the swarm_controller image.
It only listens: any ATAK/WinTAK/iTAK on the LAN sees the same
`239.2.3.1:6969` feed. On the host without Docker (Ubuntu's Python refuses system-wide pip, so use
a venv): `python3 -m venv --system-site-packages .venv && .venv/bin/pip install pytak==7.6.1 && .venv/bin/python src/swarm_control/scripts/tak_viewer.py --logs ./logs`.
Without PyTAK it falls back to a plain socket.

**PyTAK transport.** The swarm_controller image pins `pytak==7.6.1`.
`TAK_TRANSPORT` (compose env / `--transport`) selects the bridge's sender:
`auto` (default: PyTAK when importable), `pytak`, or `stdlib` (the original
`CotSender`). `PytakSender` runs PyTAK's asyncio `TXWorker` in a daemon
thread behind the same never-raises `send()` contract. It sends UDP as
`udp+wo://` so PyTAK doesn't also bind a reader on 6969, uses multicast TTL
32 like `CotSender`, and reconnects TCP/TLS every 5 s. `file://` always
uses `CotSender`, because PyTAK opens files `wb+`, which truncates them.

Tests (no ROS/Gazebo): `python3 tests/test_tak_sim.py`.
The OpenCV cases need cv2 and the PyTAK cases need pytak; each is skipped
when its module is missing.

Gotchas:
- The viewer and test feeds share ATAK's multicast group with any other
  swarm on the LAN, and the default UID prefix (`ardupilot-swarm`) is shared
  too. A test feed sent while a real mission is running mixes into that
  mission's picture. Use `TAK_UID_PREFIX` or a unicast `TAK_URL` for
  experiments.
- Map tiles (Esri World Imagery) and Leaflet load from the internet; the
  CoT picture itself is local.

## Quick start

```bash
# LAN multicast — any ATAK/WinTAK on the same network sees the swarm, no server
TAK_URL="udp://239.2.3.1:6969" SPRAYER_INSTANCES="2 3 4 5" USE_GAZEBO=1 ./start_swarm.sh 6
```

```bash
# TAK Server over mutual TLS (put PEMs in ./tak first, see tak/README.md)
TAK_URL="tls://tak.example.com:8089" \
TAK_CERT=/root/tak/swarm.pem TAK_KEY=/root/tak/swarm.key TAK_CA=/root/tak/ca.pem \
./start_swarm.sh 4
```

```bash
# Several sinks at once, plus a CoT log you can replay later
TAK_URL="udp://239.2.3.1:6969 tcp://10.0.0.5:8087 file:///root/logs/tak/cot.log" ./start_swarm.sh 4
```

`TAK_URL` empty (the default) = bridge not started; nothing else changes.
The bridge is read-only with respect to the swarm and every send is
best-effort, so a missing/unreachable TAK server never affects a mission.

### What shows up in TAK

| Item | CoT type | Source | Notes |
|---|---|---|---|
| `MAPPER-<id>` / `SPRAYER-<id>` | `a-f-A-M-F-Q-r` (friendly rotary UAV) | `/ap/geopose/filtered` per drone domain | 1 Hz; course/speed from fixes; remarks = role, mode, armed, AGL, speed; battery in `<status>`; stale after 10 s without fixes |
| Weed detections (magenta) | `b-m-p-s-m` spot map | mapper sidecars `weed_detections[]` | appear live during the survey |
| Spray targets `S<id>-T<k>` | `b-m-p-s-m`, one colour per sprayer | `logs/prescriptions/sprayer_<i>.json` | replace the raw detections (sent `t-x-d-d` deletes) once the plan lands; linked to their sprayer |
| `SURVEY FIELD` | `u-d-r` rectangle | field origin ± 40 m | remarks carry progress: frames, detections, targets |

In the search-and-rescue mission (`SWARM_MISSION=sar`, see
[`SAR.md`](SAR.md)) the drones are `SEARCH-<id>`, and the call is relayed as a
`SURVIVOR` CASEVAC point (`b-r-f-h-c`) plus one GeoChat broadcast (`b-t-f`).

Static items are re-broadcast every 30 s so EUDs that join late (multicast has
no server-side history) still get them.

### Configuration (env vars → `swarm_controller`)

| Variable | Meaning |
|---|---|
| `TAK_URL` | space-separated sinks: `udp://`, `tcp://`, `tls://`, `file://` |
| `TAK_CERT` / `TAK_KEY` / `TAK_KEY_PASSWORD` | client cert + key PEM (container paths, `/root/tak/...`) |
| `TAK_CA` | TAK Server CA PEM. Omitted = encrypted but **server not authenticated** |
| `TAK_VIDEO_URL` | template for a video button on mapper markers, e.g. `rtsp://10.0.0.2:8554/drone{instance}` |
| `TAK_GEOID_OFFSET` | geoid undulation N in m (default −17 for the Boulder field). CoT `hae` is ellipsoidal: `hae = AMSL + N` |
| `TAK_UID_PREFIX` | CoT uid prefix (default `ardupilot-swarm`) — change it if two swarms share a TAK network |

### Camera video in ATAK

The Gazebo cameras stream raw H.264/RTP to `127.0.0.1:5600+i`, which an EUD
cannot reach. Restream each one as RTSP (e.g. [MediaMTX](https://github.com/bluenviron/mediamtx)
with a GStreamer/ffmpeg source per port), then set
`TAK_VIDEO_URL=rtsp://<host-ip>:8554/drone{instance}`. The mapper markers get
a video button. Mind the QGC port-5600 clash noted in `CLAUDE.md`.

### Testing without a TAK client

```bash
python3 tests/test_tak_bridge.py      # unit + socket tests
```

```bash
# Replay a finished mission's detections / plan from the host (no ROS needed)
python3 src/swarm_control/scripts/tak_bridge.py --no-telemetry --logs ./logs \
    --sprayers 2 3 4 5 --url udp://239.2.3.1:6969
```

```bash
# Watch the multicast feed on the host
python3 -c "import socket,struct;s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1);s.bind(('',6969));s.setsockopt(socket.IPPROTO_IP,socket.IP_ADD_MEMBERSHIP,struct.pack('4sl',socket.inet_aton('239.2.3.1'),socket.INADDR_ANY));[print(s.recv(65535).decode()[:200]) for _ in iter(int,1)]"
```

### Gotchas

- **Multicast interface**: on a multi-homed host the kernel picks the
  default-route NIC. If EUDs are on another interface, add a route
  (`ip route add 239.0.0.0/8 dev <if>`) or use a TAK Server.
- **Altitude**: TAK shows `hae` (ellipsoid). Forgetting the geoid offset puts
  the drones ~17 m off in 3D views / elevation readouts.
- **Port 8087/8089 are TAK Server defaults**; FreeTAKServer uses 8087 (TCP)
  and 8089 (TLS) too.

### Next steps (not implemented)

1. **Inbound tasking from TAK**: listen for CoT from operators — e.g. a spot
   marker of a reserved type or a `u-d-f` polygon — and feed it to
   `prescription_planner.py` as extra targets / no-spray zones. The sprayers'
   `prepare_mission` hook already consumes a file, so the planner is the
   natural place to merge operator input.
2. **Sensor footprint in ATAK**: add `<sensor fov=".." azimuth=".." elevation="-90"/>`
   to mapper markers so TAK itself draws the camera footprint. The browser
   viewer already draws the footprints from track + AGL.
3. **TAK Server data sync**: upload `field_map.jpg` (stitched mosaic) and
   `plan.geojson` as a Data Package / KML overlay after the mission.

---

## How this would work with AFSIM

**AFSIM** (Advanced Framework for Simulation, Integration and Modeling, AFRL)
is a C++ engagement/mission-level simulation: platforms with movers, sensors,
comms, weapons and *processors* (behaviour/decision logic), scripted in WSF
input files, with Warlock (operator UI) and Mystic (replay) on top. It is
strong exactly where this repo is weak — sensors, EW, comms degradation,
threats, many-vs-many analysis — and weak where this repo is strong — real
autopilot code, 6-DoF physics, cameras. So the natural pairing is an
**LVC** split:

| Layer | Owner | What it provides |
|---|---|---|
| Live | real ATAK users (and later real drones) | humans in the loop via TAK |
| Virtual | this repo: ArduPilot SITL + Gazebo | flight-accurate drones, real firmware (AC_Sprayer, GUIDED), camera + detection pipeline |
| Constructive | AFSIM | environment, sensors, comms/EW, threats, other forces, tasking logic |

> AFSIM distribution is controlled by AFRL (license / information transfer
> agreement); confirm the export classification of your release before
> putting it on the same network as unclassified tooling.

### Glue: DIS as the simulation bus, TAK as the human picture

AFSIM natively speaks **DIS** (and HLA via its plugin); it does not speak CoT.
So the recommended topology is:

```
                 DIS (UDP 3000, Entity State / Fire / Detonation / Signal PDUs)
  AFSIM  ◄───────────────────────────────────────────────────►  dis_bridge (new)
  (WSF dis_interface,                                              │  shares the
   realtime mode)                                                  │  SwarmTakBridge
                                                                   │  state core
                                                                   ▼
  ATAK / WinTAK ◄──── CoT ──── tak_bridge.py ◄── swarm state ── ROS2 / DDS ── ArduPilot SITL
```

1. **Swarm → AFSIM (drones become external platforms).** Add a DIS sink next
   to the CoT sinks: from the same per-drone state `tak_bridge.py` already
   keeps (lat/lon/alt, course/speed), emit Entity State PDUs (WGS84 → ECEF
   position, ECEF velocity, Euler angles; force ID friendly; an entity type
   you map to a small-UAS platform type in AFSIM's `dis_interface`
   `entity_type` block). AFSIM then treats each SITL drone as an *external*
   platform — its radars/EO-IR sensors can detect them, its comms models can
   decide whether a drone's link survives, its threats can engage them. The
   Python [`opendis`](https://github.com/open-dis/open-dis-python) package is
   enough; no AFSIM code changes.
2. **AFSIM → TAK (constructive forces on the operator's map).** Listen for
   AFSIM's outgoing Entity State PDUs and re-emit them as CoT, mapping DIS
   force ID → CoT affiliation (`1→a-f`, `2→a-h`, `3→a-n`) and DIS entity
   domain → CoT dimension (air `A`, ground `G`, surface `S`). Threat rings /
   sensor coverage from AFSIM can become `u-d-c-c` circles. Now ATAK shows
   the real-firmware drones *and* the simulated environment on one map.
3. **AFSIM → swarm (effects and tasking), the closed loop.** Map AFSIM
   outputs onto the hooks this repo already has:
   - **Comms / EW effects**: when AFSIM says a drone's datalink is jammed,
     have the gateway drop that drone's DDS commands or trigger a GCS
     failsafe via MAVLink (`FS_GCS_ENABLE`) — exercises the real firmware's
     failsafe behaviour.
   - **No-fly / threat zones**: AFSIM threat envelopes → ArduPilot polygon
     fences (MAVLink `FENCE_POINT` / mission-protocol fence upload), and →
     exclusion zones for `prescription_planner.py`.
   - **Tasking**: an AFSIM task processor (`WSF_TASK_PROCESSOR` / quantum
     tasker) assigns targets; the gateway turns assignments into the same
     prescription JSON the sprayers already poll for, or directly into GUIDED
     targets on `/ap/cmd_gps_pose` (degrees + metres — see `CLAUDE.md`).
   - **Sensor truth vs. perception**: AFSIM's simulated EO detections can be
     compared against the OpenCV weed/ArUco detections from the real camera
     pipeline — a ready-made validation harness for sensor models.

### Timing and coordinates

- Run AFSIM in **real-time** mode (`realtime` in the scenario) — SITL and
  Gazebo run at wall-clock speed (speedup 1), and DIS assumes shared time.
  Use DIS dead-reckoning (algorithm 2/DRM FPW) so 1 Hz updates look smooth.
- Use one geodetic origin everywhere: AFSIM scenario positions, the DIS
  gateway, `FIELD_ORIGIN_LAT/LON` in the launch file and
  `LAT_BASE/LON_BASE` in `start_drone_container.sh` must all agree.
- DIS is ECEF (metres), CoT is lat/lon + ellipsoidal `hae`, ArduPilot is
  AMSL — keep the geoid offset in one place (`TAK_GEOID_OFFSET`).

### Two modes worth separating

- **Interactive LVC** (above): few drones, wall-clock, humans in ATAK.
  Fidelity is in SITL; AFSIM provides the world.
- **Batch analysis**: thousands of Monte-Carlo runs (swarm size, sensor
  quality, threat density) are AFSIM's home turf and too slow with SITL in the
  loop. Port the *decision logic* (balanced serpentine split, clustering,
  NN-ordered prescription shares) into an AFSIM script processor and
  validate the AFSIM model against this repo's SITL runs before trusting the
  batch results.

### Suggested build order

1. CoT outbound (done — this document).
2. DIS outbound ESPDUs from the bridge state → AFSIM sees the swarm.
3. DIS inbound → CoT, so AFSIM entities appear in ATAK.
4. One closed-loop effect (comms jam → GCS failsafe) end-to-end, verified the
   way `CLAUDE.md` requires: by actual vehicle behaviour, not by ACKs.
5. Tasking from AFSIM / ATAK into the prescription planner.
