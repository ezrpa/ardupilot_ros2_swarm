# Hardware Testing — Flying This System on the Two Custom Quads

Status: **plan, not yet executed.** Scope: **one drone flies its GUIDED waypoint
strip.** Camera, sprayer, prescription planner and mosaic stay out; the second
airframe is a spare / second test article.

---

## 0. The blocker you must clear first

The airframes were built as the **NXP HoverGames** kit — `RDDRONE-FMUK66` flight
controller + NavQPlus companion, running **PX4**
([project page](https://www.hackster.io/tayaran-drones/tayaran-drones-16c45a)).
This repository is ArduPilot end to end. Two facts decide the plan:

1. **ArduPilot has no build for the FMUK66.** All 636 Copter stable targets on
   `firmware.ardupilot.org` were checked: zero matches for `fmuk`, `nxp`, `k66`.
   The FMUK66 is an NXP Kinetis K66; ArduPilot's ChibiOS HAL is STM32-only.
   PX4's own docs list the board as
   [discontinued](https://docs.px4.io/main/en/flight_controller/nxp_rddrone_fmuk66.html).
2. **PX4 OFFBOARD is structurally incompatible with how `BaseDrone` flies.** PX4
   [requires a continuous ≥2 Hz setpoint stream](https://docs.px4.io/main/en/flight_modes/offboard.html)
   or it triggers `COM_OBL_RC_ACT`. `base_drone_dds.py:420` publishes **one**
   `GlobalPosition` per waypoint and then blocks in `wait_until_arrived()` —
   ArduPilot's latched-target GUIDED semantics. Porting is a control-loop
   rewrite, not a transport swap.

**Decision taken: replace the flight controller with an ArduPilot-capable H7
board, keep a Raspberry Pi as the companion.** The repo then runs unchanged and
what you test on hardware is the code you already debugged in SITL.

### Before buying anything

Connect one FC to QGroundControl over USB and record what it reports: board
name, firmware type, firmware version. That confirms board identity and whether
these are still the 2022 FMUK66 boards or something you have since swapped. Do
not skip this — the whole section above assumes the Hackster BOM is current.

---

## 1. Target hardware

### Flight controller — must be H7

ArduPilot's own guidance is explicit:

> "For STM32 based flight controllers, the DDS interface should be used on the
> H7-based boards. The F4-based and F7-based boards do not have enough CPU to
> run DDS."
> — [ROS 2 on Raspberry Pi](https://ardupilot.org/dev/docs/ros2-pi.html)

| Board | MCU | Notes |
|---|---|---|
| Pixhawk 6C | STM32H743 | Cheapest credible option (~$200), standard JST-GH telemetry ports |
| CubeOrange / Orange+ | STM32H757 | Triple-redundant IMU, best vibration tolerance, ~$350 |
| Matek H743-SLIM | STM32H743 | Lightest, cheapest, no vibration isolation |

Any of these carries the existing GPS/compass on its standard GPS1 port.

### Firmware — DDS is NOT in the stable build

> "ROS2 (via DDS) is not part of the standard ArduPilot build. Use the
> [Custom Build Server](https://custom.ardupilot.org/add_build) and ensure
> 'MicroXRCE DDS support for ROS 2' is checked."

So: build ArduCopter for your board on `custom.ardupilot.org` with the
**MicroXRCE DDS** feature ticked, and flash that `.apj`. Verify by loading the
full parameter list and confirming `DDS_ENABLE` exists. If it does not, the
build did not include AP_DDS and nothing downstream will work.

### Companion — Raspberry Pi

The ArduPilot guide uses a Pi Zero 2W, but that is for running the **agent
only**. This repo also runs `base_drone_dds.py` with `pyproj` and `numpy` on the
companion, so use a **Pi 4 or Pi 5 with ≥4 GB**. OS: **Ubuntu Server 22.04
LTS (64-bit)** — the only Ubuntu that carries ROS 2 Humble, which is the only
distro ArduPilot's DDS supports.

**Power the Pi from its own 5 V BEC, not from the FC's telemetry port.** Per the
ArduPilot wiki: a problem on the Pi must not be able to bring down the vehicle.

### Wiring

FC `TELEM2` (JST-GH) ↔ Pi GPIO14 (TXD) / GPIO15 (RXD), **crossed** (FC TX → Pi
RX), plus a common ground. Both sides are 3.3 V; do not connect the telemetry
port's 5 V pin.

On Ubuntu 22.04 for Pi, the GPIO UART needs freeing from the serial console:

- `/boot/firmware/config.txt`: add `enable_uart=1` and `dtoverlay=disable-bt`
  (moves the good PL011 UART onto the GPIO header as `/dev/ttyAMA0`).
- `/boot/firmware/cmdline.txt`: remove any `console=serial0,115200` /
  `console=ttyAMA0,115200` entry.
- `sudo systemctl disable --now serial-getty@ttyS0.service`

Confirm which device you ended up with (`/dev/ttyAMA0` after `disable-bt`,
`/dev/ttyS0` without it) and use that name consistently below.

---

## 2. Phases

**What you can usefully do before the new FC arrives:** mechanical checks (prop
balance, motor/ESC health, frame and wiring quality), site selection and
permission, and pilot practice on another aircraft. Everything else waits.

Calibration and tuning do **not** transfer across an FC swap — accel, compass,
RC and ESC calibration, `BATT_MONITOR` calibration, motor test configuration,
failsafe parameters and the entire autotune result all live in the flight
controller. Doing Phases 1–2 on the current board means doing them twice. The
parameter names below are ArduPilot's and assume the new board is already
mounted.

**Acceptance for every phase is a dataflash `.bin` log review, not observation.**
This is the hardware form of the rule `CLAUDE.md` already states twice: verify
by actual `rel_alt` gain and by comparing the real track against commanded
waypoints, never by ACKs.

### Phase 1 — Bench bring-up (props OFF)

These have never flown. Nothing here is optional.

1. Frame class/type: `FRAME_CLASS 1` (quad), `FRAME_TYPE 1` (X) — check against
   your actual motor layout. Note `dds_swarm.parm` sets `FRAME_TYPE 0` (plus);
   that is a SITL value and wrong for an X frame.
2. Accelerometer calibration (6-position), compass calibration, RC calibration.
3. Motor test in Mission Planner / QGC: confirm motor **order** and **direction**
   against the ArduCopter motor diagram for your frame type. A reversed motor is
   the single most common first-flight crash.
4. ESC calibration.
5. Battery monitor: set `BATT_MONITOR`, calibrate voltage and current against a
   multimeter. The SITL file sets `BATT_MONITOR 4` with `BATT_LOW_VOLT 0` — the
   zero disables the low-voltage failsafe and must not survive to hardware.
6. Failsafes: RC failsafe configured on the receiver AND `FS_THR_ENABLE`;
   battery failsafe thresholds set for your cell count.
7. Flight-mode switch set so the pilot can reach **Stabilize, AltHold, Loiter,
   Land, RTL** instantly.
8. Prop balance and vibration check.

**Gate:** motor test correct, all calibrations pass, arming checks pass on the
bench with `ARMING_CHECK 1`.

### Phase 2 — Manual airworthiness

Open field, competent pilot on the sticks, props on.

1. Stabilize: brief hover, check for oscillation and drift.
2. AltHold: confirm `MOT_THST_HOVER` learns to a sane value.
3. Loiter: must hold position within a couple of metres without pilot input.
4. Autotune (Copter's `AUTOTUNE` mode), then re-verify Loiter.

**Gate:** Loiter holds position cleanly for 60 s, hands off. Review the `.bin`
log for vibration (`VIBE`), clipping, and EKF innovation spikes.

ArduPilot's own prerequisite says the same thing: make the vehicle work well in
manual and autonomous modes *before* trying to use ROS.

### Phase 3 — GUIDED from a GCS, no ROS 2

Prove the vehicle accepts external position targets at all, with nothing of ours
in the loop. In QGC or Mission Planner: take off in GUIDED, click a "Go to
location" a short distance away, confirm it flies there and holds.

**Gate:** commanded position vs. logged position match in the `.bin` log.

This is the last phase with none of our code in the loop. If GUIDED misbehaves
here, the problem is the vehicle or its tune — fix it before adding ROS 2, or
you will be debugging two systems at once.

### Phase 4 — Companion integration on the ground (props OFF)

Highest-value cheap test in the whole plan. It separates *"our software works on
real hardware"* from *"this airframe flies."* It must happen **outdoors** —
`wait_for_position_estimate()` and the EKF origin need a real 3D GPS fix.

Props off, battery in, vehicle on the ground, Pi powered:

1. Micro-ROS agent running on the Pi (§3), autopilot connected.
2. `ros2 topic list` shows the `/ap/*` surface.
3. `ros2 service list` shows **all three** of `/ap/arm_motors`,
   `/ap/mode_switch`, `/ap/experimental/takeoff`. `base_drone_dds.py:64-66`
   blocks forever in `wait_for_service()` on that last one, and it is a
   **hardcoded** topic with no parameter override. `AP_DDS_EXPERIMENTAL_ENABLED`
   defaults to 1 in source, but whether your custom-build-server feature set
   ships the experimental interfaces is unverified — check it here, not in the
   field.
4. `ros2 topic hz /ap/status /ap/pose/filtered /ap/geopose/filtered` — and read
   the numbers, don't just confirm traffic. **`/ap/geopose/filtered` must sustain
   ≳2 Hz.** `wait_until_arrived()` polls at 5 Hz and `is_airborne()` gates on
   the same data; below that the mission loop starts throwing phantom
   `'not_flying'` aborts. Default AP_DDS publication rates are not sized for a
   115200 serial link — ArduPilot's own guide lists *"applying a custom hwdef to
   tune rates"* as still-to-come. If the rate is short, reduce the published
   topic set or raise the link baud before going further.
5. `ros2 topic echo /ap/gps_global_origin/filtered` — real lat/lon, not (0,0).
   `_gps_origin_callback_impl` explicitly ignores the (0,0) sentinel.
6. Service-call `/ap/mode_switch` to GUIDED and confirm the mode changes.
7. Run the drone node with **props off** and watch it attempt arm → GUIDED →
   takeoff. Expect it to arm (motors spin, no props) and then fail the climb
   check — that is correct behaviour, `main()` raises rather than flying a
   mission from a grounded aircraft.

**Gate:** all three services present, every `/ap/*` topic verified live at usable
rates, arm and mode switch work from ROS 2.

### Phase 5 — First GUIDED strip

Props on, pilot on the sticks, RC transmitter live, in a mode the pilot can
override from.

**The pilot must know: recovering control means flipping the mode switch to
Loiter or Land. Moving the sticks does not take control back from GUIDED.**

First flight uses the reduced mission from §4: small box, low altitude.

**Gate:** the logged GPS track matches the commanded waypoints within the
arrival radius, and `rel_alt` shows a real climb.

---

## 3. Pi software setup

### Install

```bash
# Ubuntu Server 22.04 arm64 on the Pi
sudo apt install ros-humble-ros-base ros-dev-tools
sudo apt install ros-humble-geographic-msgs ros-humble-geometry-msgs
sudo apt install python3-pyproj python3-numpy
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
```

`geographic_msgs` is imported by `base_drone_dds.py` (`GeoPoseStamped`,
`GeoPointStamped`) but is **missing from `src/swarm_control/package.xml`** — add
it there as well as installing it.

### Micro-ROS agent

Build it per the [ArduPilot guide](https://ardupilot.org/dev/docs/ros2-pi.html)
(cross-compile with Docker on your laptop; the Pi's memory makes on-target
builds painful), then run it against the **serial** device — not UDP:

```bash
ros2 run micro_ros_agent micro_ros_agent serial -v4 -b 115200 -D /dev/ttyAMA0
```

Note the transport change from SITL. `AP_DDS_config.h` gates UDP behind
`AP_NETWORKING_ENABLED` with the comment *"UDP only on SITL for now"*, so on a
non-Ethernet FC the `DDS_UDP_PORT` / `DDS_IP` parameters do not even exist.
Serial is the only path. `src/scripts/start_micro_ros_agent_drone.sh` (which
uses `udp4 --port 2019+i`) does not apply on hardware.

Use the systemd user service from the ArduPilot guide so the agent comes up on
boot, and `loginctl enable-linger $USER`.

### ardupilot_msgs

`swarm_control` needs `ardupilot_msgs` (the `Status`, `GlobalPosition`,
`ArmMotors`, `ModeSwitch`, `Takeoff` interfaces). Build only that package from
the ArduPilot source tree — do **not** try to satisfy `package.xml`'s
`ardupilot_sitl` dependency on the Pi; it drags in the whole SITL build and is
irrelevant to a real vehicle. Drop it from `package.xml` for the hardware build.

---

## 4. Code changes required before the first flight

None of these are optional, and all of them are values that only make sense in
SITL.

| # | Where | Problem | Change |
|---|---|---|---|
| 1 | `base_drone_dds.py:29` | `TAKEOFF_ALT_M = 50.0` — hardcoded 50 m first climb | Make it a ROS parameter; set **15 m**, with waypoints at the same altitude. **Do not go below 10 m.** `rel_alt()` is `geopose alt_amsl - home_alt` — the difference of two GPS-derived altitudes, several metres of noise on a non-RTK M8N — checked against `MIN_FLIGHT_ALT_M = 2.0`. Fly low enough and `is_airborne()` flickers false, which `wait_until_arrived()` turns into a mission abort |
| 2 | `compose_decentralized.launch.py:14-15` | `FIELD_ORIGIN_LAT/LON = 40.072842, -105.230575` — ArduPilot's SITL default location (Boulder, CO) | Set to your **actual** test site. A stale origin commands waypoints kilometres away — the same failure class as the degE7 bug documented in `CLAUDE.md`, arriving by a different route |
| 3 | `compose_decentralized.launch.py` | `generate_grid_waypoints(field_size=80.0, grid_points=5, height=30.0)` — 25 waypoints over 80×80 m | First flight: ~20 m box, 4 waypoints, 15 m altitude (matching the takeoff altitude above), well inside the fence |
| 4 | `compose_decentralized.launch.py` | `respawn=True, respawn_delay=5.0` | **`respawn=False` on hardware.** The code's own comment notes a respawn "re-runs this whole sequence mid-air" — on a real aircraft that means a second arm/takeoff/mission attempt on a flying vehicle |
| 5 | `src/swarm_control/parameters/dds_swarm.parm` | `FS_GCS_ENABLE 0`, `BATT_LOW_VOLT 0`, `ARMING_CHECK 65470`, `DISARM_DELAY 0`, `FRAME_TYPE 0`, fake `COMPASS_OFS`/`INS_ACCOFFS` values | **Never load this on hardware.** Write a separate `hw_safety.parm` (§5) |
| 6 | Launch path | Everything runs inside `swarm_controller` via docker compose; there is no single-drone, non-docker launch | Add a `hw_single_drone.launch.py`, or run directly: `ros2 run swarm_control base_drone_dds.py --ros-args -p drone_id:=1 -p field_origin_lat:=<site> -p field_origin_lon:=<site> -p assigned_waypoints:='<json>'`. This is the actual software deliverable of this phase |
| 7 | `package.xml` | `geographic_msgs` missing; `ardupilot_sitl` unusable on the Pi | Add the former, drop the latter for hardware builds |
| 8 | `base_drone_dds.py` `arrival_radius` default `3.0` | Fine for SITL's perfect GPS; a non-RTK M8N is ~1–2.5 m CEP | Keep 3 m but widen to 5 m if the vehicle circles waypoints without latching |

---

## 5. `hw_safety.parm` — the hardware parameter set

Do not reuse `dds_swarm.parm`. Start from your board's defaults after Phase 1
calibration and add:

```
# --- DDS / companion link ---
DDS_ENABLE       1
DDS_DOMAIN_ID    1        # must equal ROS_DOMAIN_ID on the Pi (drone_id)
DDS_USE_NS       0        # 1 would prefix topics with /vN and break our /ap/* names
SERIAL2_PROTOCOL 45       # SerialProtocol_DDS_XRCE
SERIAL2_BAUD     115      # 115200 — the agent is only reliable at this rate

# --- Geofence: the backstop against a bad field origin ---
FENCE_ENABLE     1
FENCE_TYPE       3        # altitude + circle
FENCE_RADIUS     100      # metres, sized to your test site
FENCE_ALT_MAX    30
FENCE_ACTION     1        # RTL

# --- Failsafes: all ON, unlike SITL ---
ARMING_CHECK     1        # all checks
FS_THR_ENABLE    1        # RTL on RC loss
DISARM_DELAY     10
BATT_LOW_VOLT    <set for your pack>
BATT_CRT_VOLT    <set for your pack>
BATT_FS_LOW_ACT  2        # RTL
BATT_FS_CRT_ACT  1        # Land

# --- Conservative speeds for early flights ---
WPNAV_SPEED      300      # 3 m/s
WPNAV_SPEED_UP   150
WPNAV_SPEED_DN   100
RTL_ALT          2000     # 20 m, above the survey altitude
```

`FS_GCS_ENABLE`: set to `1` **only if** you fly with a GCS continuously
connected over a telemetry radio. The Pi's DDS link is not a GCS heartbeat
source, so enabling it without a GCS will trigger a failsafe. `dds_swarm.parm`
sets it to `0`; that is a SITL convenience, not a hardware default.

---

## 6. Assumptions to confirm

- An open flying site with permission, well clear of people and structures.
- A competent pilot on an RC transmitter for every flight from Phase 2 onward.
- Applicable airspace rules for your site (in the US: Part 107 and a check of
  the airspace class before flying).
- Sufficient battery for a 25-waypoint survey once you scale past the reduced
  first mission — the 80×80 m grid at 3 m/s is several minutes of transit on top
  of climb and RTL.

## 7. Explicitly out of scope

Camera capture, weed detection, the sprayer role, the prescription planner, the
field mosaic, and multi-drone operation. Bring the second airframe up through
Phase 2 as a spare, but do not fly two aircraft under ROS 2 control until a
single one has completed Phase 5 repeatably.
