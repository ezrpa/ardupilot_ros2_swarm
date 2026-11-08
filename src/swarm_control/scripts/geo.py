#!/usr/bin/env python3
"""Local metres <-> GPS, the ONE conversion every module uses.

The model is ArduPilot's own, so positions agree with the autopilot and with
SITL's Gazebo ground truth (SITL reports GPS as ``home.offset(north, east)``,
libraries/SITL/SIM_Aircraft.cpp). From libraries/AP_Common/Location.cpp and
AP_Math/definitions.h:

  * LATLON_TO_M = 0.011131884502145034 m per 1e-7 deg
    -> 111318.84502145034 m per degree of latitude;
  * equirectangular: longitude metres scale by cos(latitude), taken at the
    mid-latitude of the two points (``longitude_scale``), floored at 0.01.

Earlier code mixed three models (111111 and 111320 m/deg, and a pyproj
transverse Mercator for waypoints), so the same point read up to ~0.26%
differently depending on which module asked.
"""
import math

M_PER_DEG_LAT = 0.011131884502145034 * 1e7  # = 111318.84502145034


def longitude_scale(lat):
    """cos(lat), floored like ArduPilot's Location::longitude_scale."""
    return max(math.cos(math.radians(lat)), 0.01)


def offset_latlon(lat, lon, north, east):
    """Point `north`/`east` metres from (lat, lon) (Location::offset)."""
    dlat = north / M_PER_DEG_LAT
    dlon = east / (M_PER_DEG_LAT * longitude_scale(lat + dlat / 2.0))
    return lat + dlat, lon + dlon


def ne_between(lat1, lon1, lat2, lon2):
    """(north, east) metres from point 1 to point 2 (Location::get_distance_NE)."""
    return ((lat2 - lat1) * M_PER_DEG_LAT,
            (lon2 - lon1) * M_PER_DEG_LAT * longitude_scale((lat1 + lat2) / 2.0))


def distance_m(lat1, lon1, lat2, lon2):
    """Ground distance in metres (Location::get_distance)."""
    return math.hypot(*ne_between(lat1, lon1, lat2, lon2))


class LocalFrame:
    """North/east metres relative to a fixed GPS origin (e.g. the field origin)."""

    def __init__(self, lat0, lon0):
        self.lat0, self.lon0 = float(lat0), float(lon0)

    def to_latlon(self, north, east):
        return offset_latlon(self.lat0, self.lon0, north, east)

    def to_ne(self, lat, lon):
        return ne_between(self.lat0, self.lon0, lat, lon)

    def __repr__(self):
        return f"LocalFrame({self.lat0:.7f}, {self.lon0:.7f})"
