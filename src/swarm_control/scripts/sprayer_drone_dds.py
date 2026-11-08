#!/usr/bin/env python3
"""SprayerDrone: extension of BaseDrone for the spraying role.

Pairs with the base_drone_sprayer_* Gazebo models and sprayer.parm (which
enables ArduPilot's AC_Sprayer: pump on SERVO9/function 22, speed-proportional
flow, RC aux function 15 as the on/off switch).

The drone flies its assigned strip like any BaseDrone; this subclass simply
turns the REAL firmware sprayer on for the duration of the mission and off
afterwards, via MAV_CMD_DO_SPRAYER over the drone's SERIAL1 MAVLink TCP port
(5762 + 10*instance — free, since ground stations use the UDP outputs).
While flying, AC_Sprayer drives the pump proportionally to ground speed and
stops below SPRAY_SPEED_MIN; the host-side spray_visualizer.py mirrors the
pump servo output onto the Gazebo particle-emitter plumes.

MAVLink failures are non-fatal: without the side channel the drone still
flies its mission, just without spraying. Nothing in here may raise into
start_mission().
"""
import json
import os
import time

from base_drone_dds import BaseDrone, main

try:
    from pymavlink import mavutil
    _MAV_OK = True
except ImportError:  # pragma: no cover
    mavutil = None
    _MAV_OK = False

# ArduPilot's sprayer commands, as implemented in firmware:
#
# MAV_CMD_DO_SPRAYER (216, ardupilotmega dialect) — the dedicated command.
#   param1: 1 = enable, 0 = disable; params 2-7 unused. Firmware path is
#   GCS_MAVLINK::handle_command_do_sprayer() -> AC_Sprayer::run(bool).
#   Sent as COMMAND_LONG: ArduPilot converts every COMMAND_LONG to a
#   COMMAND_INT (try_command_long_as_command_int) before dispatch, and
#   DO_SPRAYER stores no location, so the conversion is lossless.
#   The same command is also a valid AUTO mission item
#   (AP_Mission::start_command_do_sprayer, cmd.p1 == 1) — unused here, we
#   fly GUIDED and command the sprayer directly.
#
# MAV_CMD_DO_AUX_FUNCTION (218) — generic aux-switch command; aux function 15
#   is the sprayer (hence RC9_OPTION 15 in sprayer.parm). Reaches the same
#   AC_Sprayer::run() through RC_Channel, and is the fallback for firmware
#   built without the DO_SPRAYER handler (HAL_SPRAYER_ENABLED off).
MAV_CMD_DO_SPRAYER = 216
MAV_CMD_DO_AUX_FUNCTION = 218
SPRAYER_ENABLE, SPRAYER_DISABLE = 1, 0
AUX_FUNC_SPRAYER = 15
AUX_POS_LOW, AUX_POS_HIGH = 0, 2
MAV_RESULT_ACCEPTED = 0
ACK_TIMEOUT = 3.0


class SprayerDrone(BaseDrone):
    def __init__(self, node_name):
        super().__init__(node_name)

        # SERIAL1 MAVLink TCP port of this instance (0 disables spraying)
        self.declare_parameter('mavlink_tcp_port', 0)
        self._mav_port = self.get_parameter(
            'mavlink_tcp_port').get_parameter_value().integer_value
        self._mav = None  # None = not tried, False = unavailable

        # Prescription plan produced by prescription_planner.py from the
        # mappers' weed detections. Empty path = fly assigned_waypoints
        # (blanket strip) as before.
        self.declare_parameter('prescription_file', '')
        self.declare_parameter('prescription_timeout', 600.0)
        self._rx_file = self.get_parameter(
            'prescription_file').get_parameter_value().string_value
        self._rx_timeout = self.get_parameter(
            'prescription_timeout').get_parameter_value().double_value

    def prepare_mission(self):
        """Wait ON THE GROUND for the prescription plan, then fly its weed
        targets instead of the blanket strip. On timeout, fall back to the
        assigned strip so the field still gets treated."""
        if not self._rx_file:
            return
        self.get_logger().info(
            f"[sprayer] waiting for prescription {self._rx_file} "
            f"(timeout {self._rx_timeout:.0f}s)")
        deadline = time.time() + self._rx_timeout
        while time.time() < deadline:
            if os.path.exists(self._rx_file):
                try:
                    with open(self._rx_file) as f:
                        plan = json.load(f)
                    self.waypoints = [tuple(wp) for wp in plan['waypoints']]
                    self.get_logger().info(
                        f"[sprayer] prescription loaded: "
                        f"{len(self.waypoints)} weed targets")
                    return
                except (json.JSONDecodeError, KeyError, OSError) as e:
                    self.get_logger().warning(
                        f"[sprayer] bad prescription file: {e}; retrying")
            time.sleep(5)
        self.get_logger().warning(
            "[sprayer] prescription timeout - falling back to "
            f"blanket strip ({len(self.waypoints)} waypoints)")

    def _mavlink(self):
        if self._mav is None:
            if not (_MAV_OK and self._mav_port):
                self._mav = False
            else:
                try:
                    m = mavutil.mavlink_connection(
                        f'tcp:127.0.0.1:{self._mav_port}')
                    m.wait_heartbeat(timeout=10)
                    self._mav = m
                    self.get_logger().info(
                        f"[sprayer] MAVLink up on tcp:{self._mav_port}")
                except Exception as e:
                    self.get_logger().warning(
                        f"[sprayer] MAVLink unavailable on "
                        f"tcp:{self._mav_port}: {e} - flying without spray")
                    self._mav = False
        return self._mav or None

    def _command(self, m, command, *params):
        """Send a COMMAND_LONG and return the MAV_RESULT for THAT command.

        Returns None if no matching ack arrives within ACK_TIMEOUT. The link
        has been idle since wait_heartbeat(), so the buffer may hold acks for
        other commands — match on ack.command and keep reading until the
        deadline rather than trusting the first ack off the wire.
        """
        args = list(params) + [0.0] * (7 - len(params))
        m.mav.command_long_send(
            m.target_system, m.target_component, command, 0, *args)
        deadline = time.time() + ACK_TIMEOUT
        while True:
            remaining = deadline - time.time()
            if remaining <= 0:
                return None
            ack = m.recv_match(type='COMMAND_ACK', blocking=True,
                               timeout=remaining)
            if ack is not None and ack.command == command:
                return ack.result

    def set_spray(self, on):
        """Turn ArduPilot's AC_Sprayer on or off.

        Tries MAV_CMD_DO_SPRAYER first and falls back to the RC aux-function
        command if the firmware rejects it or stays silent.

        An ACCEPTED ack only proves the firmware has a sprayer object — it is
        returned whenever AP::sprayer() is non-null, whatever SPRAY_ENABLE is
        set to. Proof that liquid moves is pump servo PWM on SERVO9; watch it
        with spray_visualizer.py, or with
        `src/scripts/spray_command_test.py --instance N`.
        """
        m = self._mavlink()
        if m is None:
            return False
        want = 'ON' if on else 'OFF'
        try:
            result = self._command(
                m, MAV_CMD_DO_SPRAYER,
                SPRAYER_ENABLE if on else SPRAYER_DISABLE)
            if result != MAV_RESULT_ACCEPTED:
                self.get_logger().warning(
                    f"[sprayer] DO_SPRAYER {want} returned {result} - "
                    "falling back to aux function 15")
                result = self._command(
                    m, MAV_CMD_DO_AUX_FUNCTION, AUX_FUNC_SPRAYER,
                    AUX_POS_HIGH if on else AUX_POS_LOW)
        except Exception as e:
            # Side channel died mid-mission: log and fly on, dry.
            self.get_logger().warning(f"[sprayer] spray {want} failed: {e}")
            self._mav = False
            return False
        if result == MAV_RESULT_ACCEPTED:
            self.get_logger().info(f"[sprayer] spray {want}")
            return True
        self.get_logger().error(
            f"[sprayer] spray {want} rejected (result={result}) - "
            "flying without spray")
        return False

    def start_mission(self):
        self.set_spray(True)
        try:
            super().start_mission()
        finally:
            self.set_spray(False)


if __name__ == '__main__':
    main(node_class=SprayerDrone)
