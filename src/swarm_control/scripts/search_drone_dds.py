#!/usr/bin/env python3
"""SearchDrone: search-and-rescue extension of BaseDrone.

Pairs with the base_drone_cam* models in swarm_drone_sar.sdf. Each drone flies
its balanced 1/n serpentine strip of the search area (same split as mapping)
and, at every waypoint, looks for the missing person (hi-vis orange, see
camera_processor.PERSON_*). Behaviour:

  SEARCH    fly the strip; capture + detect at each waypoint.
  CONFIRM   on a sighting, descend to `confirm_alt` over the georeferenced
            position and look again (halves the pixel footprint and the
            georeferencing error, and rejects one-frame false positives).
            Not confirmed -> resume the search at the next waypoint.
  CALL      confirmed -> raise THE call on the swarm's call channel
            (sar_coordination.SarBus, first caller wins). The TAK bridge
            relays it to people as a CASEVAC marker + GeoChat broadcast.
  RALLY     every drone polls the channel between waypoints (0.5 s). On a
            call, the finder holds overhead at confirm_alt; the others abort
            their strips and fly to their own ring slot around the survivor
            (distinct altitudes), report on_scene when they actually arrive
            (GPS distance, not command ACKs), hold `rally_hold_s`, then RTL.
  NOT FOUND strip finished without a call -> hold at the last waypoint,
            still listening, until every other drone has also finished its
            strip (their status files) or `standby_timeout` expires — a
            slower drone may yet call. Then RTL.

Camera / channel failures are non-fatal: the drone still flies its strip.
"""
import math
import time

import rclpy
from geographic_msgs.msg import GeoPoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from base_drone_dds import BaseDrone, main
from scripts.camera_processor import PERSON_DETECTORS
from scripts.synthetic_camera import make_camera
from scripts.drone_state import DroneState, FlightMode
from scripts import sar_coordination as sar


class SearchDrone(BaseDrone):
    def __init__(self, node_name):
        super().__init__(node_name)

        p = {
            'camera_port': 0, 'instance': 0, 'swarm_instances': '',
            'logs_dir': '/root/logs', 'search_alt': 30.0,
            'confirm_alt': 15.0, 'confirm_radius': 8.0,
            'ring_radius': 15.0, 'alt_step': 5.0, 'dwell_s': 2.0,
            'leg_timeout': 90.0, 'level_tilt_deg': 3.0,
            'level_timeout': 10.0,
            'rally_hold_s': 180.0, 'arrive_radius': 4.0,
            'converge_timeout': 120.0, 'standby_timeout': 600.0,
        }
        for name, default in p.items():
            self.declare_parameter(name, default)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.instance = int(g('instance'))
        self.swarm = [int(s) for s in str(g('swarm_instances')).split()] \
            or [self.instance]
        self.search_alt = float(g('search_alt'))
        self.confirm_alt = float(g('confirm_alt'))
        self.confirm_radius = float(g('confirm_radius'))
        self.ring_radius = float(g('ring_radius'))
        self.alt_step = float(g('alt_step'))
        # dwell_s: settle time AFTER GPS-confirmed arrival, before capture.
        self.dwell_s = float(g('dwell_s'))
        self.leg_timeout = float(g('leg_timeout'))
        self.level_tilt = math.radians(float(g('level_tilt_deg')))
        self.level_timeout = float(g('level_timeout'))
        self.rally_hold_s = float(g('rally_hold_s'))
        self.arrive_radius = float(g('arrive_radius'))
        self.converge_timeout = float(g('converge_timeout'))
        self.standby_timeout = float(g('standby_timeout'))
        self.poll_s = 0.5           # call-channel poll interval
        self.converge_poll_s = 1.0  # arrival check / target re-send

        self.camera = make_camera(
            self.drone_id, int(g('camera_port')), logger=self.get_logger(),
            detectors=PERSON_DETECTORS)
        self.bus = sar.SarBus(str(g('logs_dir')))
        self.gps = None  # (lat, lon, alt_amsl) from /ap/geopose/filtered
        self.create_subscription(
            GeoPoseStamped, '/ap/geopose/filtered', self._geopose_cb,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE))
        self._status(sar.SEARCHING, detail="pre-flight")

    # ----------------------------------------------------------- helpers
    def _geopose_cb(self, msg):
        pos = msg.pose.position
        self.gps = (pos.latitude, pos.longitude, pos.altitude)

    def _log(self, msg):
        self.get_logger().info(f"[sar] {msg}")

    def _heading(self):
        """Current compass heading (rad) for body-frame georeferencing."""
        gp = getattr(self, 'current_geopose', None)
        return None if gp is None else gp.get('heading')

    def _status(self, state, **fields):
        if self.gps:
            fields.setdefault('lat', self.gps[0])
            fields.setdefault('lon', self.gps[1])
        try:
            self.bus.write_status(self.instance, state,
                                  drone_id=self.drone_id, **fields)
        except OSError as e:
            self.get_logger().warning(f"[sar] status write failed: {e}")

    def _foreign_call(self):
        call = self.bus.read_call()
        if call and call.get('found_by_instance') != self.instance:
            return call
        return None

    def _wait(self, seconds, abort_on_call=True):
        """Sleep while servicing callbacks; returns a foreign call early."""
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.0)
            if abort_on_call:
                call = self._foreign_call()
                if call:
                    return call
            time.sleep(self.poll_s)
        return None

    def _goto(self, lat, lon, alt):
        self.send_position_command_dds(lat, lon, alt)
        rclpy.spin_once(self, timeout_sec=0.0)

    def _fly_to(self, lat, lon, alt):
        """Fly to a target and return (call, arrived) once the MEASURED GPS
        is within arrive_radius (then settle dwell_s), on leg_timeout, or as
        soon as another drone calls. A fixed wait is not enough: long legs
        outlast it and the frame is taken in transit."""
        self._goto(lat, lon, alt)
        start = time.time()
        arrived = False
        while time.time() - start < self.leg_timeout:
            rclpy.spin_once(self, timeout_sec=0.0)
            call = self._foreign_call()
            if call:
                return call, False
            if self.gps and sar.distance_m(
                    self.gps[0], self.gps[1], lat, lon) <= self.arrive_radius:
                arrived = True
                break
            time.sleep(self.poll_s)
            self._goto(lat, lon, alt)  # re-send: GUIDED targets are cheap
        if not arrived:
            self.get_logger().warning(
                f"[sar] target not reached in {self.leg_timeout:.0f}s")
        call = self._wait(self.dwell_s)
        if call:
            return call, arrived
        return self._wait_level(), arrived

    def _wait_level(self):
        """Hold until the airframe is level. The camera is body-fixed (no
        gimbal): braking pitch of ~15 deg at 30 m shifts the nadir point ~9 m
        (measured), enough to fail the confirm match. Returns a foreign call
        if one arrives meanwhile."""
        end = time.time() + self.level_timeout
        while time.time() < end:
            gp = getattr(self, 'current_geopose', None)
            if gp is None or gp.get('tilt') is None \
                    or gp['tilt'] <= self.level_tilt:
                return None
            call = self._wait(self.poll_s)
            if call:
                return call
        self.get_logger().warning(
            f"[sar] not level after {self.level_timeout:.0f}s - capturing")
        return None

    def _capture_here(self, index, alt, prefix="wp"):
        """Capture geotagged with the MEASURED position and heading, never
        the commanded target. alt is height above ground (the commanded one
        when the measured height is unavailable)."""
        lat, lon = (self.gps[0], self.gps[1]) if self.gps else (None, None)
        rel = getattr(self, 'rel_alt', None)
        h = rel() if callable(rel) else None
        self.camera.capture(index, lat=lat, lon=lon,
                            alt=alt if h is None else h, prefix=prefix,
                            heading=self._heading())
        return self.camera.last_meta or {}

    # ------------------------------------------------------------ mission
    def start_mission(self):
        if self.transformer is None:
            self._log("waiting for GPS origin to initialize transformer...")
            start = time.time()
            while self.transformer is None and time.time() - start < 30:
                rclpy.spin_once(self, timeout_sec=0.1)

        self.state = DroneState.FLYING
        n = len(self.waypoints)
        call = None
        for i, wp in enumerate(self.waypoints):
            call = self._foreign_call()
            if call:
                break
            lat, lon = self.xy_to_latlon(wp[0], wp[1])
            alt = wp[2]
            self._status(sar.SEARCHING, wp=i + 1, of=n)
            self._log(f"search wp {i + 1}/{n}: {lat:.7f}, {lon:.7f} @ {alt} m")
            call, _ = self._fly_to(lat, lon, alt)
            if call:
                break

            meta = self._capture_here(i + 1, alt)
            det = sar.best_detection(meta.get('person_detections'))
            if det is None:
                continue
            self._log(f"possible person at {det['lat']:.7f}, "
                      f"{det['lon']:.7f} ({det['area_px']:.0f} px) - confirming")
            call = self._confirm_and_call(det, i + 1)
            if call:
                break

        if call is None:
            self._status(sar.SEARCH_COMPLETE, wp=n, of=n)
            self._log("strip searched, person not found by this drone")
            call = self._standby()

        if call is not None:
            self._rally(call)
        self._return_home()

    def _standby(self):
        """Strip done, nothing found: hover and listen until the rest of the
        swarm is done too (or the timeout). Returns a late call, or None."""
        done = (sar.SEARCH_COMPLETE, sar.RETURNING)
        end = time.time() + self.standby_timeout
        while time.time() < end:
            call = self._wait(self.poll_s * 4)
            if call:
                return call
            st = self.bus.read_statuses()
            if all(st.get(i, {}).get('state') in done
                   for i in self.swarm if i != self.instance):
                return self._foreign_call()  # last-moment call, if any
        self.get_logger().warning("[sar] standby timeout - returning")
        return None

    def _confirm_and_call(self, det, wp_index):
        """Descend, re-detect, and raise the call. Returns the call (ours or
        a foreign one that arrived meanwhile), or None to resume searching."""
        self._status(sar.CONFIRMING, wp=wp_index,
                     target_lat=det['lat'], target_lon=det['lon'])
        call, _ = self._fly_to(det['lat'], det['lon'], self.confirm_alt)
        if call:
            return call

        meta = self._capture_here(wp_index, self.confirm_alt, prefix="confirm")
        confirmed = sar.match_detection(meta.get('person_detections'),
                                        det['lat'], det['lon'],
                                        self.confirm_radius)
        if confirmed is None:
            self._log("not confirmed at low altitude - resuming search")
            self._status(sar.SEARCHING, wp=wp_index, detail="false alarm")
            return None

        won, call = self.bus.try_call(
            self.instance, self.drone_id, confirmed['lat'], confirmed['lon'],
            self.confirm_alt, first_sighting=[det['lat'], det['lon']],
            area_px=confirmed['area_px'],
            image=f"drone_{self.drone_id}/confirm_{wp_index:03d}.jpg")
        if won:
            self._log(f"PERSON CONFIRMED at {call['lat']:.7f}, "
                      f"{call['lon']:.7f} - calling the swarm")
            self._status(sar.CALLING)
        return call

    def _rally(self, call):
        lat, lon, alt = sar.rally_slot(
            call, self.instance, self.swarm, search_alt=self.search_alt,
            ring_radius=self.ring_radius, alt_step=self.alt_step)
        finder = call['found_by_instance'] == self.instance
        who = "finder, holding overhead" if finder else \
            f"responding to call from drone {call['found_by_id']}"
        self._log(f"{who}: slot {lat:.7f}, {lon:.7f} @ {alt} m")
        self._goto(lat, lon, alt)

        # Arrival is judged by actual GPS, never by the command being sent.
        start = time.time()
        arrived = False
        while time.time() - start < self.converge_timeout:
            rclpy.spin_once(self, timeout_sec=0.0)
            if self.gps:
                d = sar.distance_m(self.gps[0], self.gps[1], lat, lon)
                if d <= self.arrive_radius:
                    arrived = True
                    break
                self._status(sar.CONVERGING, distance_m=round(d, 1),
                             call_from=call['found_by_id'])
            time.sleep(self.converge_poll_s)
            self._goto(lat, lon, alt)  # re-send: GUIDED targets are cheap
        if not arrived:
            self.get_logger().warning(
                f"[sar] slot not reached in {self.converge_timeout:.0f}s")

        self._status(sar.ON_SCENE if arrived else sar.CONVERGING,
                     call_from=call['found_by_id'], finder=finder,
                     slot=[lat, lon, alt])
        self._log(f"on scene ({'arrived' if arrived else 'timeout'}), "
                  f"holding {self.rally_hold_s:.0f}s")
        self._wait(self.rally_hold_s, abort_on_call=False)

    def _return_home(self):
        self._status(sar.RETURNING)
        self.state = DroneState.RETURNING
        if not self.switch_mode(FlightMode.RTL):
            self.get_logger().error("Unable to switch to RTL, sending home")
            self.send_position_command_dds(
                self.home_lat, self.home_lon, self.home_alt)

    def destroy_node(self):
        self.camera.close()
        super().destroy_node()


if __name__ == '__main__':
    main(node_class=SearchDrone)
