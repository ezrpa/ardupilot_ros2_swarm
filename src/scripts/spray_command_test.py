#!/usr/bin/env python3
"""Exercise ArduPilot's sprayer commands against a running SITL instance and
prove the pump actually responds.

Both commands the firmware offers are tested:

  MAV_CMD_DO_SPRAYER (216)      param1 1=enable / 0=disable
  MAV_CMD_DO_AUX_FUNCTION (218) param1 15 (sprayer), param2 2=high / 0=low

An ACCEPTED ack is NOT proof of spraying: handle_command_do_sprayer() returns
ACCEPTED whenever the firmware has a sprayer object at all, regardless of
SPRAY_ENABLE. The real evidence is pump servo PWM on SERVO9 (SERVO9_FUNCTION
22), which this tool reads back from SERVO_OUTPUT_RAW on the same link.

AC_Sprayer only pumps above SPRAY_SPEED_MIN, so a parked SITL vehicle stays at
idle PWM no matter what you command. --ground-test temporarily sets
SPRAY_SPEED_MIN=0 and SPRAY_PUMP_MIN so a stationary vehicle pumps, then
restores both values.

Usage (drone-ardu-<N> running; SERIAL1 is free unless the SprayerDrone node
has it, in which case pass --port 5763+10*N for SERIAL2):

    python3 src/scripts/spray_command_test.py --instance 2 --ground-test
"""
import argparse
import os
import sys
import time

from pymavlink import mavutil

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "swarm_control"))
from swarm_control.core import config  # noqa: E402

MAV_CMD_DO_SPRAYER = 216
MAV_CMD_DO_AUX_FUNCTION = 218
SPRAYER_ENABLE, SPRAYER_DISABLE = 1, 0
AUX_FUNC_SPRAYER = 15
AUX_POS_LOW, AUX_POS_HIGH = 0, 2

PUMP_ON_PWM = 1150   # same threshold spray_visualizer.py uses
ACK_TIMEOUT = 3.0
# AC_Sprayer holds the pump on for ~1 s after a stop request (shutoff delay),
# so give each transition longer than that to settle.
SETTLE = 3.0
GROUND_TEST_PUMP_MIN = 25  # pump % floor so a parked vehicle still pumps


def command(m, cmd, *params):
    """Send COMMAND_LONG, return MAV_RESULT for that command (None on timeout)."""
    args = list(params) + [0.0] * (7 - len(params))
    m.mav.command_long_send(m.target_system, m.target_component, cmd, 0, *args)
    deadline = time.time() + ACK_TIMEOUT
    while True:
        remaining = deadline - time.time()
        if remaining <= 0:
            return None
        ack = m.recv_match(type='COMMAND_ACK', blocking=True, timeout=remaining)
        if ack is not None and ack.command == cmd:
            return ack.result


def result_name(result):
    if result is None:
        return "NO ACK"
    return mavutil.mavlink.enums['MAV_RESULT'][result].name


def get_param(m, name):
    m.mav.param_request_read_send(
        m.target_system, m.target_component, name.encode(), -1)
    deadline = time.time() + ACK_TIMEOUT
    while True:
        remaining = deadline - time.time()
        if remaining <= 0:
            return None
        msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=remaining)
        if msg is not None and msg.param_id.strip('\x00') == name:
            return msg.param_value


def set_param(m, name, value):
    m.mav.param_set_send(
        m.target_system, m.target_component, name.encode(), float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    deadline = time.time() + ACK_TIMEOUT
    while True:
        remaining = deadline - time.time()
        if remaining <= 0:
            return None
        msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=remaining)
        if msg is not None and msg.param_id.strip('\x00') == name:
            return msg.param_value


def peak_pump_pwm(m, seconds):
    """Highest SERVO9 PWM seen over the next `seconds`."""
    peak = 0
    deadline = time.time() + seconds
    while time.time() < deadline:
        msg = m.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=1)
        if msg is not None:
            peak = max(peak, msg.servo9_raw)
    return peak


def run_cycle(m, label, on_args, off_args, cmd):
    """Command the sprayer on, then off, reporting acks and pump PWM."""
    on_result = command(m, cmd, *on_args)
    on_pwm = peak_pump_pwm(m, SETTLE)
    print(f"  {label} ON   ack={result_name(on_result)}  peak pump pwm={on_pwm}")

    off_result = command(m, cmd, *off_args)
    off_pwm = peak_pump_pwm(m, SETTLE)
    print(f"  {label} OFF  ack={result_name(off_result)}  peak pump pwm={off_pwm}")

    acked = on_result == mavutil.mavlink.MAV_RESULT_ACCEPTED
    pumped = on_pwm > PUMP_ON_PWM and off_pwm <= PUMP_ON_PWM
    return acked, pumped


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--instance', type=int, default=2,
                    help='drone instance (SERIAL1 TCP port from core/config.py)')
    ap.add_argument('--port', type=int,
                    help='override the MAVLink TCP port')
    ap.add_argument('--ground-test', action='store_true',
                    help='temporarily drop SPRAY_SPEED_MIN/SPRAY_PUMP_MIN so a '
                         'stationary vehicle pumps, then restore them')
    ap.add_argument('--aux', action='store_true',
                    help='also exercise the DO_AUX_FUNCTION fallback path')
    args = ap.parse_args()

    port = args.port or config.serial1_tcp_port(args.instance)
    print(f"connecting to tcp:127.0.0.1:{port}")
    m = mavutil.mavlink_connection(f'tcp:127.0.0.1:{port}')
    if m.wait_heartbeat(timeout=15) is None:
        sys.exit(f"no heartbeat on tcp:{port} - is drone-ardu-{args.instance} up?")
    print(f"heartbeat from system {m.target_system}")

    enabled = get_param(m, 'SPRAY_ENABLE')
    print(f"SPRAY_ENABLE = {enabled}")
    if enabled == 0:
        print("  warning: sprayer disabled in firmware; commands will ack but "
              "the pump will not move (check sprayer.parm / DRONE_ROLE)")

    # SERVO_OUTPUT_RAW rides the RC_CHANNELS stream, same as spray_visualizer.py
    m.mav.request_data_stream_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 4, 1)

    saved = {}
    try:
        if args.ground_test:
            for name, value in (('SPRAY_SPEED_MIN', 0),
                                ('SPRAY_PUMP_MIN', GROUND_TEST_PUMP_MIN)):
                saved[name] = get_param(m, name)
                set_param(m, name, value)
            print(f"ground test: {saved} -> "
                  f"SPRAY_SPEED_MIN=0, SPRAY_PUMP_MIN={GROUND_TEST_PUMP_MIN}")

        idle = peak_pump_pwm(m, 2)
        print(f"idle pump pwm = {idle}")

        print("DO_SPRAYER (216):")
        acked, pumped = run_cycle(m, 'DO_SPRAYER', (SPRAYER_ENABLE,),
                                  (SPRAYER_DISABLE,), MAV_CMD_DO_SPRAYER)

        aux_acked = aux_pumped = None
        if args.aux:
            print("DO_AUX_FUNCTION (218, function 15):")
            aux_acked, aux_pumped = run_cycle(
                m, 'AUX_FUNC', (AUX_FUNC_SPRAYER, AUX_POS_HIGH),
                (AUX_FUNC_SPRAYER, AUX_POS_LOW), MAV_CMD_DO_AUX_FUNCTION)
    finally:
        for name, value in saved.items():
            if value is not None:
                set_param(m, name, value)
                print(f"restored {name} = {value}")

    print()
    print(f"DO_SPRAYER accepted: {acked}")
    print(f"pump responded:      {pumped}")
    if args.aux:
        print(f"AUX_FUNCTION accepted: {aux_acked}")
        print(f"pump responded:        {aux_pumped}")
    if not acked:
        sys.exit("FAIL: firmware did not accept DO_SPRAYER")
    if not pumped:
        sys.exit("FAIL: pump PWM did not follow the command "
                 "(try --ground-test if the vehicle is parked; check "
                 "SPRAY_ENABLE and SERVO9_FUNCTION=22)")
    print("PASS: sprayer commands accepted and pump PWM followed them")


if __name__ == '__main__':
    main()
