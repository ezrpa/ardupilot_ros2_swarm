#!/usr/bin/env python3
"""Search-and-rescue coordination: the drones' "call" channel + rally slots.

ROS-free so it can be unit-tested and reused by the TAK bridge.

The call channel is a shared directory, logs/sar/ (all drone nodes run in the
swarm_controller container) — the stand-in for the drones' mesh datalink:

  call.json          THE call. Created exactly once with O_CREAT|O_EXCL, so
                     if two drones confirm a sighting at the same moment only
                     one call wins; the loser simply becomes a responder.
  status_<inst>.json each drone's SAR state (searching / confirming /
                     converging / on_scene / ...), atomically replaced.

The TAK bridge watches the same files and relays the call to people
(CASEVAC marker + GeoChat broadcast), so operators and drones hear the call
at the same time.

Rally geometry (decentralised: every drone computes its own slot from the
call alone, no negotiation): the finder holds overhead at the confirm
altitude; responders take evenly spaced points on a ring around the survivor,
each at its own altitude so transit paths never share a level.
"""
import json
import math
import os
import time

try:
    import geo
except ImportError:  # installed as the `scripts` python package
    from scripts import geo

M_PER_DEG_LAT = geo.M_PER_DEG_LAT

# Drone SAR states (written to status_<inst>.json, shown in TAK remarks)
SEARCHING = "searching"
CONFIRMING = "confirming"
CALLING = "calling"
CONVERGING = "converging"
ON_SCENE = "on_scene"
RETURNING = "returning"
SEARCH_COMPLETE = "search_complete"


distance_m = geo.distance_m
offset_latlon = geo.offset_latlon


def best_detection(dets):
    """Most reliable georeferenced detection, or None: highest shape
    confidence when the detector provides one, else the largest blob."""
    geo = [d for d in (dets or [])
           if d.get("lat") is not None and d.get("lon") is not None]
    return max(geo, key=lambda d: (d.get("confidence", 0.0),
                                   d.get("area_px", 0))) if geo else None


def match_detection(dets, lat, lon, radius_m):
    """Georeferenced detection nearest (lat, lon) within radius_m, or None."""
    best = None
    for d in dets or []:
        if d.get("lat") is None or d.get("lon") is None:
            continue
        r = distance_m(lat, lon, d["lat"], d["lon"])
        if r <= radius_m and (best is None or r < best[0]):
            best = (r, d)
    return best[1] if best else None


def rally_slot(call, instance, instances, search_alt=30.0, ring_radius=15.0,
               alt_step=5.0):
    """(lat, lon, alt_rel_home) this drone should hold for a given call."""
    if instance == call["found_by_instance"]:
        return call["lat"], call["lon"], call["confirm_alt"]
    responders = sorted(i for i in instances
                        if i != call["found_by_instance"])
    if instance not in responders:
        responders.append(instance)
        responders.sort()
    k = responders.index(instance)
    angle = 2.0 * math.pi * k / len(responders)
    lat, lon = offset_latlon(call["lat"], call["lon"],
                             ring_radius * math.cos(angle),
                             ring_radius * math.sin(angle))
    return lat, lon, search_alt + alt_step * (k + 1)


class SarBus:
    """File-based call channel under <logs_dir>/sar/."""

    def __init__(self, logs_dir="/root/logs"):
        self.dir = os.path.join(logs_dir, "sar")
        os.makedirs(self.dir, exist_ok=True)
        self.call_path = os.path.join(self.dir, "call.json")

    def read_call(self):
        try:
            with open(self.call_path) as f:
                return json.load(f)
        except (OSError, json.JSONDecodeError):
            return None  # absent, or the winner is mid-write

    def try_call(self, instance, drone_id, lat, lon, confirm_alt, **extra):
        """Raise the call. Returns (won, call). First writer wins."""
        call = {"found_by_instance": instance, "found_by_id": drone_id,
                "lat": lat, "lon": lon, "confirm_alt": confirm_alt,
                "unix_time": time.time(), **extra}
        try:
            fd = os.open(self.call_path,
                         os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)
        except FileExistsError:
            for _ in range(20):          # let the winner finish writing
                existing = self.read_call()
                if existing:
                    return False, existing
                time.sleep(0.05)
            return False, None
        with os.fdopen(fd, "w") as f:
            json.dump(call, f, indent=2)
        return True, call

    def write_status(self, instance, state, **fields):
        path = os.path.join(self.dir, f"status_{instance}.json")
        tmp = path + ".tmp"
        with open(tmp, "w") as f:
            json.dump({"instance": instance, "state": state,
                       "unix_time": time.time(), **fields}, f)
        os.replace(tmp, path)

    def read_statuses(self):
        out = {}
        for name in os.listdir(self.dir):
            if name.startswith("status_") and name.endswith(".json"):
                try:
                    with open(os.path.join(self.dir, name)) as f:
                        st = json.load(f)
                    out[int(st["instance"])] = st
                except (OSError, json.JSONDecodeError, KeyError, ValueError):
                    continue
        return out
