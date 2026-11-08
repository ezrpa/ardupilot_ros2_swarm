#!/usr/bin/env python3
"""The swarm's TAK picture, ROS-free: SwarmTakBridge turns drone poses and
the mission artifacts in logs/ into Cursor-on-Target events; SinkFanout
delivers them to one or more TAK sinks (retrying one-shot events).

Fed by swarm_control.nodes.tak_bridge (ROS 2 sources + CLI); see that module
for what the feed contains and how it is started.
"""
import glob
import json
import os
import threading
import time

from swarm_control.core import config
from swarm_control.tak.cot import (MAGENTA, SPRAYER_PALETTE, argb,
                                   casevac_event, chat_event, course_speed,
                                   delete_event, drone_event, ne_to_latlon,
                                   rectangle_event, spot_event)

# Searched-cell colours per drone instance (alpha, r, g, b); the viewer's
# drone palette uses the same order.
CELL_PALETTE = [(70, 56, 214, 245), (70, 245, 184, 61), (70, 163, 230, 53),
                (70, 192, 132, 252), (70, 244, 114, 182), (70, 96, 165, 250)]

FIELD_ORIGIN_LAT = config.FIELD_ORIGIN_LAT
FIELD_ORIGIN_LON = config.FIELD_ORIGIN_LON
# Geoid undulation N at the field origin (EGM96, Boulder CO ~ -17 m).
# CoT hae = AMSL + N. Site-specific: change it with the field origin.
DEFAULT_GEOID_OFFSET = -17.0

MODE_NAMES = {0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO",
              4: "GUIDED", 5: "LOITER", 6: "RTL", 7: "CIRCLE", 9: "LAND",
              11: "DRIFT", 13: "SPORT", 14: "FLIP", 15: "AUTOTUNE",
              16: "POSHOLD", 17: "BRAKE", 20: "GUIDED_NOGPS", 21: "SMART_RTL"}

TRACK_STALE_S = 10.0     # drone marker greys out this long after last fix
FIX_TIMEOUT_S = 5.0      # stop reporting a drone whose fixes stopped


class SwarmTakBridge:
    """ROS-free core: holds swarm state, renders CoT. Unit-testable."""

    def __init__(self, instances, sprayers=(), logs_dir="/root/logs",
                 uid_prefix="ardupilot-swarm", geoid_offset=DEFAULT_GEOID_OFFSET,
                 field_half_m=40.0, video_url=None, replace_detections=True,
                 origin=(FIELD_ORIGIN_LAT, FIELD_ORIGIN_LON),
                 mission="survey"):
        self.mission = mission
        self.instances = list(instances)
        self.sprayers = set(sprayers)
        self.logs_dir = logs_dir
        self.prefix = uid_prefix
        self.geoid = geoid_offset
        self.field_half = field_half_m
        self.video_url = video_url
        self.replace_detections = replace_detections
        self.origin = origin
        self._lock = threading.Lock()
        self.drones = {i: {} for i in self.instances}
        self.detections = {}       # uid -> xml-builder kwargs
        self.targets = {}          # uid -> kwargs
        self._sidecar_mtime = {}   # path -> mtime
        self._plan_mtime = {}      # path -> mtime
        self._frames = 0
        self._plan_ready = False
        self._field_dirty = True
        # SAR state (mission == "sar")
        self.sar_call = None
        self.sar_status = {}       # instance -> status dict
        self._sar_mtime = {}
        self.sar_cells = {}        # "rRcC" -> claim record (grid search)
        # Events that are sent ONCE, not re-sent every cycle (the SAR call's
        # GeoChat page): SinkFanout retries these until a sink accepts them.
        self.one_shot = set()

    # --------------------------------------------------------- identity
    def role(self, inst):
        if self.mission == "sar":
            return "search"
        return "sprayer" if inst in self.sprayers else "mapper"

    def callsign(self, inst):
        return f"{self.role(inst).upper()}-{inst + 1}"

    def drone_uid(self, inst):
        return f"{self.prefix}-drone-{inst}"

    # ---------------------------------------------------- telemetry in
    def update_pose(self, inst, lat, lon, alt_amsl, t=None):
        t = time.time() if t is None else t
        with self._lock:
            d = self.drones.setdefault(inst, {})
            d["fix"] = (lat, lon, alt_amsl, t)
            # Course/speed from fixes >= 0.5 s apart (DDS pose arrives at
            # tens of Hz; tiny deltas would make course jitter wildly).
            anchor = d.get("anchor")
            if anchor is None:
                d["anchor"] = (lat, lon, t)
            elif t - anchor[2] >= 0.5:
                cs = course_speed(anchor[0], anchor[1], anchor[2], lat, lon, t)
                if cs is not None:
                    course, speed = cs
                    # hold the last course while hovering (< 0.3 m/s)
                    if speed < 0.3:
                        course = d.get("course", 0.0)
                    d["course"], d["speed"] = course, speed
                d["anchor"] = (lat, lon, t)

    def update_status(self, inst, armed=None, mode=None, flying=None):
        with self._lock:
            d = self.drones.setdefault(inst, {})
            if armed is not None:
                d["armed"] = bool(armed)
            if mode is not None:
                d["mode"] = int(mode)
            if flying is not None:
                d["flying"] = bool(flying)

    def update_battery(self, inst, percentage):
        if percentage is None or percentage != percentage or percentage < 0:
            return  # NaN / unknown
        pct = percentage * 100.0 if percentage <= 1.0 else percentage
        with self._lock:
            self.drones.setdefault(inst, {})["battery"] = pct

    def update_home(self, inst, alt_amsl):
        with self._lock:
            self.drones.setdefault(inst, {})["home_amsl"] = alt_amsl

    # ------------------------------------------------------- CoT out
    def drone_events(self, now=None):
        now = time.time() if now is None else now
        out = []
        with self._lock:
            for inst in self.instances:
                d = self.drones.get(inst, {})
                fix = d.get("fix")
                if fix is None or now - fix[3] > FIX_TIMEOUT_S:
                    continue  # let the last marker go stale on the EUD
                lat, lon, amsl, _ = fix
                parts = [self.role(inst)]
                if "mode" in d:
                    parts.append(MODE_NAMES.get(d["mode"], f"mode {d['mode']}"))
                if "armed" in d:
                    parts.append("ARMED" if d["armed"] else "disarmed")
                if "home_amsl" in d:
                    parts.append(f"{amsl - d['home_amsl']:.1f} m AGL")
                if "speed" in d:
                    parts.append(f"{d['speed']:.1f} m/s")
                st = self.sar_status.get(inst)
                if st:
                    parts.append(self._sar_state_text(st))
                video = None
                if self.video_url and self.role(inst) in ("mapper", "search"):
                    video = self.video_url.format(instance=inst, id=inst + 1)
                out.append(drone_event(
                    self.drone_uid(inst), self.callsign(inst), lat, lon,
                    hae=amsl + self.geoid, course=d.get("course"),
                    speed=d.get("speed"), battery=d.get("battery"),
                    remarks=" | ".join(parts), video_url=video,
                    stale_s=TRACK_STALE_S, now=now))
        return out

    def field_corners(self):
        h = self.field_half
        lat0, lon0 = self.origin
        return [ne_to_latlon(lat0, lon0, n, e)
                for n, e in ((h, -h), (h, h), (-h, h), (-h, -h))]

    @staticmethod
    def _sar_state_text(st):
        state = st.get("state", "?")
        if state == "searching" and "wp" in st:
            return f"SAR: searching wp {st['wp']}/{st.get('of', '?')}"
        if state == "converging" and "distance_m" in st:
            return f"SAR: converging, {st['distance_m']:.0f} m out"
        if state == "on_scene":
            return "SAR: ON SCENE" + (" (finder)" if st.get("finder") else "")
        return f"SAR: {state.replace('_', ' ')}"

    def _grid_info(self):
        """(ALGO, total cells) of a BFS/DFS grid search, remembered once
        seen: rally/on-scene statuses no longer carry 'algo'."""
        for st in self.sar_status.values():
            if st.get("algo") in ("bfs", "dfs") and st.get("of"):
                self._grid = (st["algo"].upper(), int(st["of"]))
        return getattr(self, "_grid", None)

    def _sar_field_status(self):
        grid = self._grid_info()
        grid_total = grid[1] if grid else 0
        grid_txt = (f"{grid[0]} grid search: {len(self.sar_cells)}/"
                    f"{grid_total} cells searched") if grid else ""
        if grid_total and not self.sar_call:
            if self.sar_status and all(
                    st.get("state") in ("search_complete", "returning")
                    for st in self.sar_status.values()):
                return "search complete - person NOT found"
            return grid_txt
        done = sum(min(st.get("wp", 0), st.get("of", 0))
                   for st in self.sar_status.values()
                   if st.get("state") in ("searching", "search_complete"))
        total = sum(st.get("of", 0) for st in self.sar_status.values())
        if not self.sar_call:
            if self.sar_status and all(
                    st.get("state") in ("search_complete", "returning")
                    for st in self.sar_status.values()):
                return "search complete - person NOT found"
            return f"searching: {done}/{total or '?'} waypoints"
        on_scene = sum(st.get("state") == "on_scene"
                       for st in self.sar_status.values())
        c = self.sar_call
        return (f"PERSON FOUND by SEARCH-{c.get('found_by_id')} at "
                f"{c['lat']:.6f}, {c['lon']:.6f} | {on_scene}/"
                f"{len(self.instances)} drones on scene"
                + (f" | {grid_txt}" if grid_txt else ""))

    def field_event(self, now=None):
        if self.mission == "sar":
            return rectangle_event(
                f"{self.prefix}-field", "SEARCH AREA", self.field_corners(),
                remarks=f"{2 * self.field_half:.0f}x{2 * self.field_half:.0f}"
                        f" m search area | {self._sar_field_status()}",
                now=now)
        if self._plan_ready:
            status = f"plan ready: {len(self.targets)} spray targets"
        else:
            status = (f"surveying: {self._frames} frames, "
                      f"{len(self.detections)} weed detections")
        return rectangle_event(
            f"{self.prefix}-field", "SURVEY FIELD", self.field_corners(),
            remarks=f"{2 * self.field_half:.0f}x{2 * self.field_half:.0f} m "
                    f"survey area | {status}", now=now)

    def static_events(self, now=None):
        """Everything that must survive late EUD joins (re-broadcast)."""
        out = [self.field_event(now)]
        if self.sar_call:
            out.append(self._casevac(now))
        out += [self._cell_event(c, now) for c in self.sar_cells.values()]
        with self._lock:
            items = (list(self.targets.items()) if self._plan_ready
                     else list(self.detections.items()))
        for uid, kw in items:
            out.append(spot_event(uid, now=now, **kw))
        return out

    # ------------------------------------------------ mission artifacts
    def scan_artifacts(self, now=None):
        """Pick up new sidecars / prescriptions. Returns CoT to send now."""
        out = []
        if self.mission == "sar":
            out += self._scan_sar(now)
        else:
            out += self._scan_sidecars(now)
            out += self._scan_prescriptions(now)
        if self._field_dirty:
            out.append(self.field_event(now))
            self._field_dirty = False
        return out

    # ------------------------------------------------------------- SAR
    def _casevac(self, now=None):
        c = self.sar_call
        finder = c.get("found_by_instance")
        return casevac_event(
            f"{self.prefix}-sar-casualty", "SURVIVOR", c["lat"], c["lon"],
            parent_uid=self.drone_uid(finder) if finder is not None else None,
            remarks=f"Missing person CONFIRMED by SEARCH-{c.get('found_by_id')}"
                    f" from {c.get('confirm_alt', '?')} m AGL "
                    f"({c.get('area_px', 0):.0f} px, nadir georef). "
                    f"Image: {c.get('image', 'n/a')}", now=now)

    def _read_json_if_changed(self, path):
        try:
            m = os.path.getmtime(path)
        except OSError:
            return None
        if self._sar_mtime.get(path) == m:
            return None
        try:
            with open(path) as f:
                data = json.load(f)
        except (json.JSONDecodeError, OSError):
            return None  # mid-write; retry next scan
        self._sar_mtime[path] = m
        return data

    def _cell_event(self, claim, now=None):
        """Searched grid cell (claimed by one drone), drawn in that drone's
        colour so TAK shows how the BFS/DFS fronts sweep the area."""
        inst = int(claim.get("instance", 0))
        a, r, g, b = CELL_PALETTE[inst % len(CELL_PALETTE)]
        return rectangle_event(
            f"{self.prefix}-cell-r{claim['row']}c{claim['col']}",
            f"{self.callsign(inst)} r{claim['row']}c{claim['col']}",
            [tuple(p) for p in claim["corners"]],
            stroke=argb(160, r, g, b), fill=argb(a, r, g, b),
            remarks=f"searched by {self.callsign(inst)} "
                    f"({claim.get('algo', '?').upper()})", now=now)

    def _scan_claims(self, now):
        out = []
        cdir = os.path.join(self.logs_dir, "sar", "claims")
        for path in glob.glob(os.path.join(cdir, "r*c*.json")):
            key = os.path.basename(path)[:-5]
            if key in self.sar_cells:
                continue
            try:
                with open(path) as f:
                    claim = json.load(f)
            except (OSError, json.JSONDecodeError):
                continue  # mid-write
            if "corners" in claim:
                self.sar_cells[key] = claim
                self._field_dirty = True
                out.append(self._cell_event(claim, now))
        return out

    def _scan_sar(self, now):
        out = self._scan_claims(now)
        sar_dir = os.path.join(self.logs_dir, "sar")
        for path in glob.glob(os.path.join(sar_dir, "status_*.json")):
            st = self._read_json_if_changed(path)
            if st is not None and "instance" in st:
                with self._lock:
                    self.sar_status[int(st["instance"])] = st
                self._field_dirty = True
        call = self._read_json_if_changed(os.path.join(sar_dir, "call.json"))
        if call is not None and "lat" in call and self.sar_call is None:
            self.sar_call = call
            self._field_dirty = True
            out.append(self._casevac(now))
            finder = call.get("found_by_instance", 0)
            page = chat_event(
                self.drone_uid(finder), f"SEARCH-{call.get('found_by_id')}",
                f"SURVIVOR FOUND by SEARCH-{call.get('found_by_id')} at "
                f"{call['lat']:.6f}, {call['lon']:.6f}. All search drones "
                f"converging. CASEVAC point 'SURVIVOR' dropped.",
                call["lat"], call["lon"], now=now)
            self.one_shot.add(page)
            out.append(page)
        return out

    def _scan_sidecars(self, now):
        out = []
        for path in sorted(glob.glob(
                os.path.join(self.logs_dir, "drone_*", "wp_*.json"))):
            try:
                mtime = os.path.getmtime(path)
            except OSError:
                continue
            if self._sidecar_mtime.get(path) == mtime:
                continue
            try:
                with open(path) as f:
                    meta = json.load(f)
            except (json.JSONDecodeError, OSError):
                continue  # mid-write; next scan gets it
            if path not in self._sidecar_mtime:
                self._frames += 1
                self._field_dirty = True
            self._sidecar_mtime[path] = mtime
            drone_id = meta.get("drone_id")
            wp = meta.get("wp_index")
            for k, det in enumerate(meta.get("weed_detections") or []):
                if det.get("lat") is None or det.get("lon") is None:
                    continue
                uid = f"{self.prefix}-weed-d{drone_id}-wp{wp}-{k}"
                kw = dict(callsign=f"weed {drone_id}.{wp}.{k}",
                          lat=det["lat"], lon=det["lon"], color=MAGENTA,
                          remarks=f"Weed detected by MAPPER-{drone_id} at "
                                  f"WP {wp} (area {det.get('area_px', 0):.0f} "
                                  f"px, nadir georef)")
                with self._lock:
                    is_new = uid not in self.detections
                    self.detections[uid] = kw
                if is_new:
                    self._field_dirty = True
                if not self._plan_ready:
                    out.append(spot_event(uid, now=now, **kw))
        return out

    def _scan_prescriptions(self, now):
        paths = sorted(glob.glob(os.path.join(
            self.logs_dir, "prescriptions", "sprayer_*.json")))
        changed = False
        for p in paths:
            try:
                m = os.path.getmtime(p)
            except OSError:
                continue
            if self._plan_mtime.get(p) != m:
                self._plan_mtime[p] = m
                changed = True
        if not changed:
            return []

        new_targets = {}
        for p in paths:
            try:
                with open(p) as f:
                    plan = json.load(f)
            except (json.JSONDecodeError, OSError):
                continue
            inst = int(plan.get("sprayer_instance", -1))
            order = sorted(self.sprayers) or [inst]
            color = SPRAYER_PALETTE[
                (order.index(inst) if inst in order else 0)
                % len(SPRAYER_PALETTE)]
            clusters = plan.get("clusters") or []
            for k, c in enumerate(clusters):
                if c.get("lat") is None or c.get("lon") is None:
                    continue
                uid = f"{self.prefix}-target-s{inst}-{k}"
                new_targets[uid] = dict(
                    callsign=f"S{inst + 1}-T{k + 1}", lat=c["lat"],
                    lon=c["lon"], color=color,
                    parent_uid=self.drone_uid(inst),
                    remarks=f"Spray target {k + 1}/{len(clusters)} for "
                            f"SPRAYER-{inst + 1} | "
                            f"{c.get('num_detections', '?')} detections "
                            f"clustered")

        out = []
        with self._lock:
            # Remove superseded targets (plan regenerated with fewer items)
            for uid in set(self.targets) - set(new_targets):
                out.append(delete_event(uid, now=now))
            first_plan = not self._plan_ready
            self.targets = new_targets
            self._plan_ready = True
            dets = list(self.detections)
        if first_plan and self.replace_detections:
            out += [delete_event(uid, now=now) for uid in dets]
        out += [spot_event(uid, now=now, **kw)
                for uid, kw in new_targets.items()]
        self._field_dirty = True
        return out


# ------------------------------------------------------------- ROS input

class SinkFanout:
    """Sends every event to every sink. Most CoT is a live picture that the
    bridge re-sends each cycle, so a dropped event is harmless. A one-shot
    event (the SAR GeoChat page) is NOT re-sent: send() returns False while
    a TCP/TLS link is down or reconnecting, and the page would be lost
    silently. Such events are kept per sink and retried until accepted."""

    def __init__(self, sinks, log=print, max_pending=50):
        self.sinks = list(sinks)
        self.log = log
        self.max_pending = max_pending
        self.pending = [[] for _ in self.sinks]

    def send(self, events, must_deliver=()):
        for xml in events:
            for i, s in enumerate(self.sinks):
                if not s.send(xml) and xml in must_deliver:
                    q = self.pending[i]
                    if len(q) < self.max_pending:
                        q.append(xml)
                        self.log(f"[tak] one-shot event not delivered to "
                                 f"{getattr(s, 'url', s)}; will retry")

    def retry(self, must_deliver=None):
        for i, s in enumerate(self.sinks):
            if self.pending[i]:
                left = [x for x in self.pending[i] if not s.send(x)]
                if len(left) < len(self.pending[i]):
                    self.log(f"[tak] delivered "
                             f"{len(self.pending[i]) - len(left)} retried "
                             f"one-shot event(s) to {getattr(s, 'url', s)}")
                self.pending[i] = left
        if must_deliver is not None and not any(self.pending):
            must_deliver.clear()  # all delivered: stop tracking them
