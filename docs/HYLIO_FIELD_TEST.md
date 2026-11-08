# Testing This System on an Actual Hylio Drone

Status: **plan, blocked on unbuilt software and unanswered vendor questions.**

This is the deployment target described in `MAVLINK_BRIDGE.md`. That
document specifies the *simulation* of this path; this one covers taking it to a
real aircraft. Read that file first — everything here assumes its design.

---

## 1. Why a Hylio is not like the test quads

`HARDWARE_TESTING.md` covers the two custom quads, where you control the
whole stack: swap the flight controller, flash a custom ArduPilot build with
AP_DDS compiled in, and run the repo unchanged.

**None of that is available on a Hylio.** The established facts:

- The firmware is ArduPilot-derived — its documented mode list (RTL, Auto,
  Guided, Land, Brake, Loiter, Altitude Hold) is ArduCopter's.
- It exposes **no DDS**, and you cannot reflash it with a DDS-enabled build
  without destroying the aircraft's certification, warranty, and support.
- Hylio publishes no SDK or developer API; AgroSol GCS is proprietary.
- The documented integration surface is the **RFD900x** telemetry radio Hylio
  sells as their fleet radio — a plain MAVLink serial modem, 57600 baud default.

So the only viable path is the standard ArduPilot companion-computer pattern
over MAVLink:

```
Hylio drone (ArduPilot fw) ── RFD900x serial radio ── companion ── LAN ── QGC / operators
                                    MAVLink              │
                                                MAVLink→ROS2 bridge
                                                (swarm nodes unchanged above it)
```

---

## 2. Hard prerequisites

### 2.1 Software that does not exist yet

`mavlink_bridge.py` is **specified but not implemented**. Until it exists,
`base_drone_dds.py` cannot talk to a Hylio at all — it speaks only to the `/ap/*`
DDS surface. Build and verify it in SITL first, following
`MAVLINK_BRIDGE.md` Phases 1–3, and pass its Phase 5 A/B verification
(DDS sprayers as control group vs. MAVLink sprayers) before any real hardware is
involved.

Do not treat the simulator step as optional. The bridge's most likely
correctness bug is the degrees→degE7 `×1e7` scaling — the exact inverse of the
firmware bug documented in `CLAUDE.md`. Missing it puts the aircraft at Null
Island; doubling it overflows int32. On a 180 kg aircraft over a field, that is
not a bug you debug in flight.

### 2.2 Regulatory

Hylio aircraft are far over the 55 lb threshold — the AG-272 is
[181 kg / 399 lb MTOW](https://agtecher.com/en/drones/hylio-ag-272/), the
HYL-150 ARES carries a
[110 lb payload](https://dronexl.co/2025/01/30/hylios-new-hyl-150-ares-agricultural-drone/).
Dispensing agricultural chemicals from them in the US requires, at minimum:

- **Part 107** remote pilot certificate;
- **[Part 137](https://www.faa.gov/uas/advanced_operations/dispensing_chemicals)**
  Agricultural Aircraft Operator certificate;
- for aircraft ≥55 lb, **Part 91 certification** plus **§44807 exemption**, and
  exemptions from numerous Part 61/91/137 sections.

Any modification to the control path — which is exactly what attaching a
companion computer is — has to be reconciled with whatever exemption the
aircraft operates under. **Settle this before scheduling a flight, not after.**
Non-spraying test flights (water, or dry runs with the pump inhibited) sidestep
the chemical-dispensing rules but not the airworthiness ones.

### 2.3 Vendor

Confirm with Hylio support, in writing, before touching an aircraft:

1. Does their firmware build accept `SET_POSITION_TARGET_GLOBAL_INT` unmodified,
   and is **GUIDED** mode enabled and reachable over the telemetry link?
2. Are parameters locked or read-only over MAVLink? The bridge needs to
   `SET_MESSAGE_INTERVAL`; some fleet builds restrict this.
3. What `SYSID_THISMAV` does each aircraft use, and is it configurable?
4. Is attaching a companion computer to the telemetry link permitted, and what
   does it do to warranty and support?
5. Does `MAV_CMD_DO_SPRAYER (216)` reach `AC_Sprayer::run()` in their build, or
   have they replaced the sprayer path with proprietary logic?

Questions 1 and 5 are the ones that can invalidate the whole approach. Get them
answered first.

---

## 3. The bandwidth constraint

The RFD900x default is **57600 baud** — roughly 5.7 kB/s raw, less after air-rate
and duty-cycle overhead, and **shared across every aircraft on the channel**.

`MAVLINK_BRIDGE.md` Phase 1 requests `GLOBAL_POSITION_INT`,
`LOCAL_POSITION_NED` and `EXTENDED_SYS_STATE` at 10 Hz. At ~40 bytes per
message that is ~1.2 kB/s per aircraft before heartbeats, acks, and whatever
AgroSol is already streaming. Workable for one aircraft; it will not scale to a
fleet on one radio.

Plan for it explicitly:

- **One aircraft first.** Do not attempt swarm operation on a shared channel
  until single-aircraft telemetry is proven stable.
- Drop `stream_rate_hz` to 4–5 Hz. `BaseDrone`'s arrival check
  (`wait_until_arrived`, `arrival_radius` 3 m) polls at 5 Hz and tolerates this;
  a 3 m radius at Hylio spray speeds still needs enough position updates not to
  fly through the acceptance circle between samples — check this in SITL by
  lowering the bridge's stream rate before you rely on it in the field.
- Give each aircraft its own radio and channel if you go beyond one.
- Watch for the **empty-stream trap**: without `SET_MESSAGE_INTERVAL` /
  `REQUEST_DATA_STREAM` the link is silent and `BaseDrone` hangs in
  `wait_for_position_estimate()`. Check `ros2 topic hz /ap/pose/filtered` before
  anything else.

---

## 4. Test ladder

Every rung's acceptance is a **dataflash `.bin` log review** — commanded vs.
actual track, and real `rel_alt` gain. Command ACKs prove nothing; `CLAUDE.md`
documents this twice, and it is doubly true here because an ACCEPTED
`DO_SPRAYER` ack is returned whenever `AP::sprayer()` is non-null, **whatever
`SPRAY_ENABLE` is**.

### Rung 0 — SITL

`MAVLINK_BRIDGE.md` Phase 5, in full. Bridge-controlled sprayers must
match DDS-controlled sprayers on track accuracy and prescription execution.

### Rung 1 — Bench, aircraft powered, **props removed, tank empty**

Aircraft on stands, RFD900x paired, companion connected to the ground-side
radio serial.

1. `mavlink_bridge.py` connects, `wait_heartbeat` succeeds, `target_system`
   matches the aircraft's `SYSID_THISMAV`.
2. `ros2 topic list` shows the `/ap/*` surface; `ros2 topic hz` confirms all of
   `/ap/status`, `/ap/pose/filtered`, `/ap/gps_global_origin/filtered` flowing.
3. Origin is a real lat/lon, not the (0,0) sentinel.
4. Service-call `/ap/arm_motors` — **expect and verify a refusal or a clean
   arm-then-disarm**; do not leave a large multirotor armed on a bench.
5. Mode switch to GUIDED accepted, `Status.mode` reflects it.
6. Sprayer command path: send the `Bool` on `/ap/sprayer_cmd` and read the pump
   servo PWM back. Same principle as
   `src/scripts/spray_command_test.py --ground-test`: `AC_Sprayer` will not pump
   below `SPRAY_SPEED_MIN`, so a parked aircraft stays at idle PWM no matter
   what you command. **Verify with pump PWM, never with the ack.**

**Gate:** full `/ap/*` surface live over the real radio at the real baud rate.

### Rung 2 — Ground, engines idle, GPS fix

Outdoors, aircraft on the ground, props on, pilot with the AgroSol controller in
hand and able to take the aircraft in an instant.

1. Set `field_origin_lat/lon` to the **actual field**. This is the highest-risk
   configuration item on the whole page: the repo ships
   `FIELD_ORIGIN_LAT/LON = 40.072842, -105.230575`, ArduPilot's SITL default in
   Boulder, Colorado. A stale origin commands waypoints a continent away.
2. Load the mission and confirm, on the GCS map, that every commanded waypoint
   lands inside the intended field boundary **before arming**.
3. Confirm the geofence on the aircraft is enabled and sized to the field.

**Gate:** commanded waypoints plotted on a map over the real field. Do not skip
this by trusting the numbers.

### Rung 3 — First GUIDED flight, water or dry

One aircraft, reduced mission: a handful of waypoints, low altitude, well inside
the fence, tank empty or water only.

**The pilot must know that recovering control means switching mode — not moving
the sticks.** GUIDED ignores stick input.

**Gate:** logged GPS track matches commanded waypoints within the arrival
radius; no failsafe events in the log; telemetry link held throughout.

### Rung 4 — Prescription-driven spray

Only after Rung 3 is repeatable. Mappers survey, `prescription_planner.py`
writes `logs/prescriptions/sprayer_<i>.json`, the bridged Hylio waits on the
ground in `prepare_mission()` (600 s timeout → blanket-strip fallback), then
flies only the weed targets.

Verify against the same standard the simulation was held to: coverage of
ground-truth targets and median targeting error, measured from the flight log —
not from the plan file.

**Note the fallback is not benign here.** If the planner never writes a file,
`prepare_mission()` falls back to a blanket strip: a full-field spray with real
chemical. Decide deliberately whether that fallback should be disabled on a real
aircraft, and if so, disable it before Rung 4.

---

## 5. Risks specific to real hardware

| Risk | Mitigation |
|---|---|
| Hylio firmware rejects `SET_POSITION_TARGET_GLOBAL_INT` or has GUIDED disabled | Vendor question #1. Kills the approach; find out before building anything |
| degE7 scaling bug in the bridge | Rung 0 track-vs-commanded comparison in SITL catches both the missing and doubled cases |
| Stale `field_origin_lat/lon` | Rung 2 map check + aircraft geofence as backstop |
| Radio link loss mid-mission | Aircraft-side GCS/telemetry failsafe configured to RTL. The bridge running on a ground companion means link loss stops the mission — an onboard companion would not have this property, but it is the right trade for a first campaign |
| Node crash mid-flight | `respawn=False`. The launch file's `respawn=True` would re-run arm/takeoff/mission on a flying aircraft — the code's own comment flags this |
| Spray triggered unintentionally | `RC9_OPTION 15` means any RC9 position change clobbers a `DO_SPRAYER` enable — and can equally enable it. Know the aux-switch state before arming |
| Blanket-spray fallback fires with chemical aboard | Disable the fallback for real-aircraft operation (§4 Rung 4) |

---

## 6. Recommended order of work

1. Implement and verify `mavlink_bridge.py` in SITL (`MAVLINK_BRIDGE.md`
   Phases 1–5). No hardware, no risk, and it is the long pole.
2. In parallel, send Hylio the five vendor questions and start the regulatory
   conversation.
3. In parallel, run the custom quads through `HARDWARE_TESTING.md`. They
   are the cheap place to learn how this stack behaves on a real airframe with
   real GPS, real EKF warm-up, and real wind — all lessons that transfer, at a
   fraction of the consequence.
4. Only then book time on a Hylio, and start at Rung 1.

The quads are the rehearsal. Do not make a 180 kg sprayer the first aircraft
this code has ever flown.
