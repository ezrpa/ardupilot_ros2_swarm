# Search & Rescue Mission (with TAK)

A second mission for the same swarm: the drones search an area for a missing
person. The first drone to **confirm** a sighting **calls the others**, who
abort their search strips and converge on the survivor. TAK users see the
whole thing and are paged when the call goes out.

```bash
SWARM_WORLD=swarm_drone_sar.sdf ./start_gazebo.sh &
SWARM_MISSION=sar TAK_URL="udp://239.2.3.1:6969" USE_GAZEBO=1 ./start_swarm.sh 4
```

`SWARM_MISSION=sar` makes `start_swarm_controller.sh` launch
`sar_mission.launch.py` instead of the mapping/spraying launch. Without
`TAK_URL` the mission runs the same, just without the TAK feed.

## What happens

```
 SEARCH ──sighting──► CONFIRM ──confirmed──► CALL ─────────► RALLY ─► hold ─► RTL
   │  (each drone flies   (descend to 15 m,   (logs/sar/call.json,   finder overhead @15 m,
   │   its 1/n strip of    look again; no      O_EXCL: first caller   others on a 15 m ring,
   │   the 120 m square    match = false       wins)                  one altitude each
   │   at 30 m, captures   alarm, resume)        │                    (35/40/45 m), arrival
   │   at each waypoint)                         │                    judged by real GPS
   │                                             ▼
   └── every 0.5 s: "has anyone called?" ── yes ──► RALLY (abort strip)
       strip done, nothing found ──► hover + listen until the whole swarm is done ──► RTL
```

| Piece | File |
|---|---|
| Missing person (life-size, hi-vis orange coverall, seeded random spot/heading) | `src/custom_gz/worlds/generate_missing_person.py` → `models/missing_person/` (+ `ground_truth.json`) |
| World (swarm_drone.sdf with the person instead of weeds) | `src/custom_gz/worlds/swarm_drone_sar.sdf` |
| Person detector (HSV band H 0–14, high saturation; no overlap with the farm palette or magenta weeds) | `perception/camera_processor.py` `PERSON_*`, `detect_color_blobs()`, `detect_person()` |
| Drone node | `nodes/search_drone.py` (executable `search_drone_dds.py`) → `SearchDrone(BaseDrone)` |
| Call channel + rally geometry (ROS-free) | `mission/sar_coordination.py` |
| Launch | `launch/sar_mission.launch.py` |
| Tests (incl. 4-drone end-to-end mission with ROS stubbed) | `tests/test_sar.py` |

A person is ~160 px at 30 m (6.4 cm/px), so the 30 m search grid (20 m
spacing, ~41×31 m camera footprint) covers the area with overlap.

Move the person: `python3 src/custom_gz/worlds/generate_missing_person.py --seed N`
(restart Gazebo afterwards).

## The call

The drones' call channel is `logs/sar/` — a shared directory standing in for
the drones' mesh datalink (all drone nodes run in `swarm_controller`):

- `call.json` — created exactly once (`O_CREAT|O_EXCL`). Two drones
  confirming at the same moment cannot both call; the loser rallies.
- `status_<instance>.json` — each drone's state: `searching` (wp k/n),
  `confirming`, `calling`, `converging` (distance), `on_scene`,
  `search_complete`, `returning`.

Rally slots are computed by every drone from the call alone (no
negotiation), so a drone that misses status updates still goes to the right
place.

`start_swarm_controller.sh` archives `logs/sar/` with the other run artifacts
at startup — a stale `call.json` would otherwise send the whole swarm to the
previous run's survivor.

## In TAK

| Item | CoT |
|---|---|
| `SEARCH-<id>` drones, remarks show SAR state (`searching wp 5/13`, `converging, 23 m out`, `ON SCENE (finder)`) | `a-f-A-M-F-Q-r` |
| `SEARCH AREA` rectangle: coverage → `PERSON FOUND by SEARCH-2 at … \| 3/4 drones on scene` or `search complete - person NOT found` | `u-d-r` |
| `SURVIVOR` CASEVAC point at the confirmed position, linked to the finder; open it in ATAK to fill the 9-line | `b-r-f-h-c` |
| One GeoChat broadcast to **All Chat Rooms**: "SURVIVOR FOUND by SEARCH-2 at … All search drones converging." | `b-t-f` |

Add `TAK_VIDEO_URL=rtsp://<host>:8554/drone{instance}` (see `docs/TAK.md`)
to put a video button on every search drone, so the operator can look
through the finder's camera.

## Verify a real run

As with the other missions (see `CLAUDE.md`), check the vehicles, not the
messages:

1. `logs/sar/call.json` position vs `models/missing_person/ground_truth.json`
   (expect a few metres: nadir georeferencing, yaw not compensated).
2. `logs/drone_<finder>/confirm_*.jpg` shows the person.
3. Every drone's actual GPS track (pymavlink on UDP `14550+i`) ends at its
   rally slot before RTL — `status_*.json` reports `on_scene` only after a GPS
   check, but confirm it.

## Limits / next steps

- **Open-loop search legs**: like `BaseDrone`, each waypoint is a fixed dwell
  (10 s) rather than an arrival check; only the rally uses real GPS arrival.
- **Detection at waypoints only**: a person between captures is found from
  the overlapping neighbour frames; continuous detection while moving would
  shorten time-to-find.
- **Single casualty, colour-based detector**: swap `detect_color_blobs` for a
  learned person detector (e.g. YOLO on the same frames) for realism; the
  CONFIRM step already guards against one-frame false positives.
- **Operator tasking from TAK**: let an ATAK user drop a marker that raises
  the call (or cancels it). The bridge would write `call.json` exactly as a
  drone does — the drones don't care who called.
- **Real datalink**: replace the shared directory with a DDS topic or
  MAVLink broadcast between vehicles; `SarBus` is the only class to change.
