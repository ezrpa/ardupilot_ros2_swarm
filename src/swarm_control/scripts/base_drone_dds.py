#!/usr/bin/env python3

import json
import math

import rclpy
import time
from ardupilot_msgs.msg import Status, GlobalPosition  # Use DDS messages
from ardupilot_msgs.srv import ArmMotors, ModeSwitch, Takeoff
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoseStamped, GeoPointStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from pyproj import Transformer

from points_distributor import generate_circular_waypoints
from scripts.drone_state import DroneState, FlightMode

from scripts.time_out_retry import TimeoutRetry


# A vehicle below this height above home is treated as not flying, whatever
# the mission loop thinks it is doing.
MIN_FLIGHT_ALT_M = 2.0

# Metres per degree of latitude; longitude is scaled by cos(lat) at use.
M_PER_DEG_LAT = 111111.0  # equirectangular, same as every other module

TAKEOFF_ALT_M = 50.0
TAKEOFF_ALT_FRACTION = 0.9   # "climb finished" threshold, fraction of target
TAKEOFF_TIMEOUT_S = 90.0
GUIDED_SWITCH_ATTEMPTS = 3


def mode_success_check_factory(mode: FlightMode):
    """Factory that creates success check for mode switching"""
    def check(result):
        return result and (result.status or result.curr_mode == mode.value)
    return check

class BaseDrone(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        # Initialize home position variables
        self.initialize_vars()
        self.get_parameter_values()
        self.sysid = self.drone_id
        self.handle_waypoints()
        self.state = DroneState.INITIALIZING

        self.get_logger().info(f"Initializing drone {self.drone_id} with DDS interface")
        self.get_logger().info(f"Using system ID: {self.sysid}")

        # Service clients for DDS - using drone-specific topics
        self._arm_topic = self.get_parameter("arm_topic").get_parameter_value().string_value
        self.get_logger().info(f"Arm service topic: {self._arm_topic}")
        self.arm_client = self.create_client(ArmMotors, self._arm_topic)
        while not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('arm service not available, waiting again...')

        self._mode_topic = self.get_parameter("mode_topic").get_parameter_value().string_value
        self.get_logger().info(f"Mode service topic: {self._mode_topic}")
        self.mode_client = self.create_client(ModeSwitch, self._mode_topic)
        while not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('mode switch service not available, waiting again...')
        self.takeoff_client = self.create_client(Takeoff, '/ap/experimental/takeoff')
        while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Takeoff service not available, waiting...')

        self._setup_subscriptions()

        # DDS subscribers and service clients
        self._global_pos_pub = self.create_publisher(GlobalPosition, self._global_pos_topic, 1)
        self.get_logger().info(f"QoS: {self.status_subscription.qos_profile}")
        self.get_logger().info(f"Subscribed to: {self.status_subscription.topic_name}")
        self.get_logger().info(f"Subscribed to: {self.subscription_pose.topic_name}")

    def initialize_vars(self):
        self.waypoints = []
        self.home_lat = None
        self.home_lon = None
        self.home_alt = None
        self.transformer = None
        self.current_position = None
        # Measured global pose from /ap/geopose/filtered: the only trustworthy
        # answer to "where is this vehicle". Commanded waypoints are not.
        self.current_geopose = None
        self.target_position = None
        self.armed = False

    def get_parameter_values(self):
        # Declare all parameters first
        self.declare_parameter('drone_id', 1)
        self.declare_parameter('assigned_waypoints', '')
        self.declare_parameter("arm_topic", "/ap/arm_motors")
        self.declare_parameter("mode_topic", "/ap/mode_switch")
        self.declare_parameter("pose_topic", "/ap/pose/filtered")
        self.declare_parameter("global_position_topic", "/ap/cmd_gps_pose")
        self.declare_parameter("takeoff_topic", "/ap/experimental/takeoff")
        # Shared field origin: all drones convert their local waypoints with
        # ONE common reference so the chunks tile a single field. Without it
        # each drone centres the field on its own home (homes are offset ~5 m
        # per instance), skewing the tiles. (0, 0) means "not set" and falls
        # back to the per-drone GPS origin.
        self.declare_parameter('field_origin_lat', 0.0)
        self.declare_parameter('field_origin_lon', 0.0)
        # Waypoint arrival is confirmed by measured position, not by a timer.
        self.declare_parameter('arrival_radius', 3.0)
        self.declare_parameter('waypoint_timeout', 60.0)
        # Short settle after arrival so the airframe damps out before the
        # camera grabs a frame; 2 s is ~20 frames at the model's 10 Hz.
        self.declare_parameter('arrival_settle', 2.0)
        # Then get parameter values
        self.drone_id = self.get_parameter('drone_id').get_parameter_value().integer_value
        self._arm_topic = self.get_parameter("arm_topic").get_parameter_value().string_value
        self._mode_topic = self.get_parameter("mode_topic").get_parameter_value().string_value
        self._global_pos_topic = self.get_parameter("global_position_topic").get_parameter_value().string_value
        self._pose_topic = self.get_parameter("pose_topic").get_parameter_value().string_value

        self.arrival_radius = self.get_parameter(
            'arrival_radius').get_parameter_value().double_value
        self.waypoint_timeout = self.get_parameter(
            'waypoint_timeout').get_parameter_value().double_value
        self.arrival_settle = self.get_parameter(
            'arrival_settle').get_parameter_value().double_value

        field_lat = self.get_parameter('field_origin_lat').get_parameter_value().double_value
        field_lon = self.get_parameter('field_origin_lon').get_parameter_value().double_value
        self._fixed_origin = abs(field_lat) > 1e-6 or abs(field_lon) > 1e-6
        if self._fixed_origin:
            self.setup_coordinate_transformer(field_lat, field_lon)
            self.get_logger().info(
                f"Using shared field origin: lat={field_lat:.8f}, lon={field_lon:.8f}")

    def _setup_subscriptions(self):
        """Set up subscriptions """
        ardupilot_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.status_callback = self._status_callback_impl
        self.pose_callback = self._pose_callback_impl

        # GPS origin callback
        self.gps_origin_callback = self._gps_origin_callback_impl

        # Create subscriptions
        self.status_subscription = self.create_subscription(
            Status, '/ap/status', self.status_callback, ardupilot_qos)
        self.subscription_pose = self.create_subscription(
            PoseStamped, '/ap/pose/filtered', self.pose_callback, ardupilot_qos)
        self.gps_origin_subscription = self.create_subscription(
            GeoPointStamped,
            '/ap/gps_global_origin/filtered',
            self.gps_origin_callback,
            ardupilot_qos
        )
        # Measured lat/lon/alt + attitude. /ap/pose/filtered is local ENU
        # relative to the EKF origin, which is NOT the shared field origin the
        # waypoint transformer is built on, so it cannot be compared against
        # waypoints directly. geopose is already global.
        self.geopose_subscription = self.create_subscription(
            GeoPoseStamped,
            '/ap/geopose/filtered',
            self._geopose_callback_impl,
            ardupilot_qos
        )

    @TimeoutRetry(timeout_sec=30.0, retry_interval=1.0,
                  success_check_factory=lambda: lambda result: result and result.result)
    def arm(self):
        req = ArmMotors.Request()
        req.arm = True
        future = self.arm_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    @TimeoutRetry(timeout_sec=60.0, retry_interval=2.0,
                  success_check_factory=lambda: lambda pos: pos is not None)
    def wait_for_position_estimate(self):
        """Wait for position with automatic retry"""
        rclpy.spin_once(self)
        return self.current_position

    def _status_callback_impl(self, msg):
        """Handle ArduPilot status messages - implementation"""
        self.armed = msg.armed
        self.current_mode = msg.mode
        self.get_logger().info(f"Status callback: {msg}")
        if self.state == DroneState.CONNECTING:
            self.state = DroneState.CONNECTED
            self.get_logger().info(f"DDS connection established for drone {self.drone_id}")

    def _pose_callback_impl(self, msg):
        """Process a GeoPose message - implementation"""
        self.current_position = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        )

    def _geopose_callback_impl(self, msg):
        """Measured global pose: true lat/lon/alt plus vehicle heading.

        AP_DDS publishes this orientation in ENU/REP-103 (x forward, y left,
        z up), so the quaternion yaw is measured counter-clockwise from EAST;
        convert to a compass heading (clockwise from north) for georeferencing.
        """
        q = msg.pose.orientation
        yaw_enu = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.current_geopose = {
            'lat': msg.pose.position.latitude,
            'lon': msg.pose.position.longitude,
            'alt_amsl': msg.pose.position.altitude,
            'heading': (math.pi / 2.0 - yaw_enu) % (2.0 * math.pi),
            # Total tilt from level (rad), sign-free: the body-fixed camera
            # is only nadir when this is ~0 (braking pitch skews georef).
            'tilt': math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (
                q.x * q.x + q.y * q.y)))),
        }

    def rel_alt(self):
        """Height above home in metres, or None if not yet known."""
        if self.current_geopose is None or self.home_alt is None:
            return None
        return self.current_geopose['alt_amsl'] - self.home_alt

    def is_airborne(self):
        """True only when the vehicle is armed AND actually off the ground.

        The mission loop must never advance on a vehicle that has landed,
        disarmed, or crashed: every capture taken in that state gets geotagged
        with a waypoint the aircraft never visited.
        """
        alt = self.rel_alt()
        return bool(self.armed) and alt is not None and alt > MIN_FLIGHT_ALT_M

    def distance_to(self, lat, lon):
        """Ground distance in metres from the measured position, or None."""
        gp = self.current_geopose
        if gp is None:
            return None
        m_per_deg_lon = M_PER_DEG_LAT * math.cos(math.radians(gp['lat']))
        return math.hypot((gp['lat'] - lat) * M_PER_DEG_LAT,
                          (gp['lon'] - lon) * m_per_deg_lon)

    def wait_until_arrived(self, lat, lon, timeout):
        """Spin until the vehicle actually reaches (lat, lon).

        Returns 'arrived', 'timeout', or 'not_flying'. Replaces a blind
        time.sleep() that could not distinguish flying from parked.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if not self.is_airborne():
                return 'not_flying'
            distance = self.distance_to(lat, lon)
            if distance is not None and distance <= self.arrival_radius:
                return 'arrived'
        return 'timeout'

    def arm_response_callback(self, future):
        """Handle arm service response"""
        try:
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            if response.result:
                self.get_logger().info(f"Drone {self.drone_id} armed successfully via DDS")
                self.set_guided_mode_dds()
            else:
                self.get_logger().error(f"Failed to arm drone {self.drone_id}")
        except Exception as e:
            self.get_logger().error(f'Arm service call failed: {e}')

    @TimeoutRetry(timeout_sec=60.0, retry_interval=2.0,
                  success_check_factory=mode_success_check_factory)
    def switch_mode(self, mode: FlightMode):
        req = ModeSwitch.Request()
        req.mode = mode.value
        future = self.mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def takeoff(self, altitude=10.0):
        req = Takeoff.Request()
        req.alt = altitude
        future = self.takeoff_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def _gps_origin_callback_impl(self, msg):
        """Capture the GPS origin (home position) when it's set - implementation"""
        lat = msg.position.latitude
        lon = msg.position.longitude

        # ArduPilot publishes (0, 0) on this topic until the EKF origin is
        # actually set. Building the transformer from that sentinel centres all
        # waypoints on Null Island, so ignore it and wait for a real origin.
        if abs(lat) < 1e-6 and abs(lon) < 1e-6:
            return

        if self.home_lat != lat or self.home_lon != lon:
            self.home_lat = lat
            self.home_lon = lon
            self.home_alt = msg.position.altitude

            # (Re)build the transformer whenever the origin changes — the first
            # valid origin may arrive after an earlier bogus one. When a shared
            # field origin was configured, the transformer is already fixed on
            # it and must not be re-centred on this drone's own home.
            if not self._fixed_origin:
                self.setup_coordinate_transformer(self.home_lat, self.home_lon)

            self.get_logger().info(
                f"Home position set: lat={self.home_lat:.8f}, "
                f"lon={self.home_lon:.8f}, alt={self.home_alt:.2f}"
            )

    def setup_coordinate_transformer(self, lat_0, lon_0):
        """Set up pyproj transformer for local to global conversions,
        centred on (lat_0, lon_0)."""
        # Create a local tangent plane projection centered at the origin
        # Using Transverse Mercator projection
        proj_string = (
            f"+proj=tmerc +lat_0={lat_0} +lon_0={lon_0} "
            f"+k=1 +x_0=0 +y_0=0 +ellps=WGS84 +units=m +no_defs"
        )
        self.transformer = Transformer.from_crs(
            proj_string,  # Local coordinate system
            "EPSG:4326",  # WGS84
            always_xy=True
        )

        self.get_logger().info("Coordinate transformer initialized")

    def xy_to_latlon(self, x, y):
        """Convert local NED coordinates to lat/lon using pyproj
        Args:
            x: North position in meters (positive north)
            y: East position in meters (positive east)
        """
        # pyproj expects x=east, y=north, so swap the inputs
        lon, lat = self.transformer.transform(y, x)
        return lat, lon

    def on_waypoint_reached(self, wp_index, lat, lon, alt):
        """Extension hook, called once the vehicle has actually arrived.

        lat/lon are the MEASURED position and alt the measured height above
        home — not the commanded waypoint. Anything that georeferences from
        these values (CameraDrone in camera_drone_dds.py) therefore records
        where the aircraft really was. No-op in the base drone."""
        pass

    def prepare_mission(self):
        """Extension hook, called on the ground BEFORE arming. No-op in the
        base drone; SprayerDrone overrides it to wait for and load its
        prescription plan."""
        pass

    def start_mission(self):
        """Start waypoint mission using DDS"""
        self.get_logger().info(f"Starting mission for drone {self.drone_id}")
        
        # Wait for transformer to be initialized
        if self.transformer is None:
            self.get_logger().info("Waiting for GPS origin to initialize transformer...")
            timeout = 30  # 30 second timeout
            start_time = time.time()
            while self.transformer is None and (time.time() - start_time) < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)

        self.state = DroneState.FLYING

        # Closed-loop waypoint following. Arrival is confirmed against the
        # MEASURED position and the mission aborts the moment the vehicle stops
        # flying — an earlier open-loop version advanced on a fixed 10 s timer,
        # so a node that respawned onto a landed aircraft happily "reached"
        # every waypoint and geotagged 32 of 33 weed detections from a drone
        # parked on the ground.
        for i, wp in enumerate(self.waypoints):
            lat, lon = self.xy_to_latlon(wp[0], wp[1])
            self.send_position_command_dds(lat, lon, wp[2])
            rclpy.spin_once(self)
            self.get_logger().info(
                f"Drone {self.drone_id} heading to waypoint {i + 1}: "
                f"{lat, lon, wp[2]}")

            outcome = self.wait_until_arrived(lat, lon, self.waypoint_timeout)
            if outcome == 'not_flying':
                self.state = DroneState.ERROR
                self.get_logger().error(
                    f"Drone {self.drone_id} is no longer airborne at waypoint "
                    f"{i + 1} (armed={self.armed}, rel_alt={self.rel_alt()}); "
                    "aborting mission rather than logging captures the "
                    "aircraft never actually made")
                return
            if outcome == 'timeout':
                self.get_logger().warning(
                    f"Drone {self.drone_id} did not reach waypoint {i + 1} "
                    f"within {self.waypoint_timeout:.0f}s "
                    f"({self.distance_to(lat, lon):.1f} m short); capturing "
                    "at the measured position and moving on")
            else:
                # Arrived: let the airframe settle before the capture.
                settle_until = time.time() + self.arrival_settle
                while time.time() < settle_until:
                    rclpy.spin_once(self, timeout_sec=0.1)

            # Hand the subclass hook the MEASURED pose, never the command.
            gp = self.current_geopose
            self.on_waypoint_reached(i + 1, gp['lat'], gp['lon'],
                                     self.rel_alt())

        self.get_logger().info("Mission complete, returning to launch")
        self.state = DroneState.RETURNING
        if not self.switch_mode(FlightMode.RTL):
            self.get_logger().error("Unable to switch to RTL mode, sending home position")
            self.send_position_command_dds(self.home_lat, self.home_lon, self.home_alt)


    def send_position_command_dds(self, lat, lon, alt):
        msg = GlobalPosition()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.type_mask = 0b0000111111111000  # Use position only, ignore velocity/acceleration
        msg.header.frame_id = "map"
        msg.coordinate_frame = 6  # MAV_FRAME_GLOBAL_RELATIVE_ALT_INT

        # ArduPilot's DDS handler (AP_DDS_ExternalControl.cpp) expects lat/lon
        # in DEGREES and altitude in METERS — it scales to degE7/cm itself
        # (Location(lat * 1E7, ...), alt * 100). Pre-scaling here caused int32
        # overflow in the firmware and sent the drones marching kilometres off
        # the field chasing wrapped garbage targets.
        msg.latitude = float(lat)    # degrees
        msg.longitude = float(lon)   # degrees
        msg.altitude = float(alt)    # metres (frame 6: relative to home)

        self._global_pos_pub.publish(msg)

    def handle_waypoints(self):
        assigned_waypoints_param = self.get_parameter('assigned_waypoints').get_parameter_value().string_value
        if not assigned_waypoints_param or assigned_waypoints_param == '':
            self.waypoints = generate_circular_waypoints()
            self.get_logger().info(f"Generated default circular waypoints for drone {self.drone_id}")
        else:
            try:
                self.waypoints = json.loads(assigned_waypoints_param)
                self.get_logger().info(
                    f"Using assigned waypoints for drone {self.drone_id}: {len(self.waypoints)} points")
            except Exception as e:
                self.get_logger().error(f"Error parsing waypoints: {e}, using default")
                self.waypoints = generate_circular_waypoints()


def main(args=None, node_class=BaseDrone):
    rclpy.init(args=args)
    # Start the node with default parameter (node_class lets extensions such
    # as CameraDrone reuse this arm/takeoff/mission sequence unchanged)
    node = node_class('base_drone_dds')

    # Pre-arm extension hook: runs ON THE GROUND before arming. SprayerDrone
    # uses it to wait for its prescription plan; no-op in the base drone.
    node.prepare_mission()

    # Block till armed, which will wait for EKF3 to initialize
    if not node.arm():
        raise RuntimeError("Unable to arm")

    # Wait for a position estimate before switching to GUIDED
    if not node.wait_for_position_estimate():
        raise RuntimeError("No position estimate available")

    # The GUIDED switch races EKF readiness and ArduPilot rejects it until the
    # filter settles. Letting the TimeoutError kill the node is worse than
    # waiting: launch respawns it onto an already-flying aircraft, and the new
    # process re-runs this whole sequence mid-air. Retry in-process instead.
    for attempt in range(1, GUIDED_SWITCH_ATTEMPTS + 1):
        try:
            if node.switch_mode(FlightMode.GUIDED):
                break
        except TimeoutError:
            pass
        node.get_logger().warning(
            f"GUIDED switch attempt {attempt}/{GUIDED_SWITCH_ATTEMPTS} "
            "failed; retrying")
    else:
        raise RuntimeError("Unable to switch to GUIDED mode")

    # Takeoff
    takeoff_result = node.takeoff(TAKEOFF_ALT_M)
    if not takeoff_result.status:
        raise RuntimeError("Takeoff failed")
    node.get_logger().info(f"Drone take off status: {takeoff_result.status}")

    # Wait for the climb to actually happen instead of guessing at 15 s.
    # is_airborne() is the liveness test (a couple of metres clears it), which
    # is far too low to start transiting to a waypoint - hold until the vehicle
    # is nearly at the commanded takeoff altitude.
    climb_target = TAKEOFF_ALT_M * TAKEOFF_ALT_FRACTION
    climb_deadline = time.time() + TAKEOFF_TIMEOUT_S
    while time.time() < climb_deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        alt = node.rel_alt()
        if alt is not None and alt >= climb_target:
            break
    if not node.is_airborne():
        raise RuntimeError(
            f"Takeoff acknowledged but the vehicle never left the ground "
            f"(rel_alt={node.rel_alt()}) - refusing to fly a mission that "
            "would log captures from a grounded aircraft")
    if node.rel_alt() < climb_target:
        node.get_logger().warning(
            f"Drone {node.drone_id} only reached {node.rel_alt():.1f} m of the "
            f"{climb_target:.1f} m climb target in {TAKEOFF_TIMEOUT_S:.0f}s; "
            "continuing since it is airborne")
    node.get_logger().info(
        f"Drone {node.drone_id} airborne at {node.rel_alt():.1f} m")

    node.get_logger().info(f"Drone {node.drone_id} initialized with DDS")
    node.get_logger().info(f"Drone {node.drone_id} started mission")
    node.start_mission()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()