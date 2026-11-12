
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped

from px4_msgs.msg import (
    VehicleLocalPosition,
    VehicleAttitude,
    VehicleStatus,
    VehicleCommand,
    VehicleCommandAck,
    VehicleLandDetected,
    OffboardControlMode,
    TrajectorySetpoint,
)


class PrecisionLandingController(Node):

    def __init__(self):
        super().__init__('precision_landing_controller')

        # Parameters
        self.declare_parameter('marker_timeout', 0.5)
        self.declare_parameter('alignment_tolerance', 0.10)
        self.declare_parameter('horizontal_speed', 0.30)
        self.declare_parameter('descent_speed', 0.15)
        self.declare_parameter('terminal_transition_height', 0.60)
        self.declare_parameter('handover_height', 0.50)
        self.declare_parameter('filter_alpha', 0.25)
        self.declare_parameter('auto_land_request_interval', 2.0)

        # Safety switches
        self.declare_parameter('request_offboard', False)
        self.declare_parameter('enable_landing', False)

        self.marker_timeout = float(
            self.get_parameter('marker_timeout').value
        )
        self.alignment_tolerance = float(
            self.get_parameter('alignment_tolerance').value
        )
        self.horizontal_speed = float(
            self.get_parameter('horizontal_speed').value
        )
        self.descent_speed = float(
            self.get_parameter('descent_speed').value
        )
        self.terminal_transition_height = float(
            self.get_parameter('terminal_transition_height').value
        )
        self.handover_height = float(
            self.get_parameter('handover_height').value
        )
        self.filter_alpha = float(
            self.get_parameter('filter_alpha').value
        )
        self.auto_land_request_interval = float(
            self.get_parameter('auto_land_request_interval').value
        )

        # PX4 navigation state constants
        self.auto_land_nav_state = getattr(
            VehicleStatus,
            'NAVIGATION_STATE_AUTO_LAND',
            18
        )

        # State machine
        self.state = 'PRESTREAM'

        self.local_position = None
        self.attitude_q = None

        self.vehicle_armed = False
        self.offboard_active = False
        self.auto_land_active = False

        # Marker world position
        self.marker_world = None
        self.marker_last_seen = None

        # Position setpoints
        self.command_x = None
        self.command_y = None
        self.command_z = None

        self.last_control_time = time.monotonic()

        # PX4 timestamp synchronization
        self.px4_timestamp = 0
        self.px4_timestamp_received = None

        # Offboard handover
        self.prestream_started = None
        self.last_mode_request = 0.0
        self.mode_request_interval = 2.0
        self.offboard_ack_received = False

        # Auto Land handover
        self.last_auto_land_request = 0.0
        self.auto_land_ack_received = False

        # PX4 landing detection
        self.px4_landed = False
        self.landed_logged = False

        # Subscribers
        self.create_subscription(
            PoseStamped,
            '/landing/target_pose_body',
            self.marker_callback,
            qos_profile_sensor_data
        )

        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.local_position_callback,
            qos_profile_sensor_data
        )

        self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
            qos_profile_sensor_data
        )

        self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v4',
            self.status_callback,
            qos_profile_sensor_data
        )

        self.create_subscription(
            VehicleCommandAck,
            '/fmu/out/vehicle_command_ack',
            self.command_ack_callback,
            qos_profile_sensor_data
        )

        self.create_subscription(
            VehicleLandDetected,
            '/fmu/out/vehicle_land_detected',
            self.land_detected_callback,
            qos_profile_sensor_data
        )

        # Publishers
        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10
        )

        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10
        )

        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10
        )

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            'Precision landing controller initialized.'
        )

        self.get_logger().info(
            f'PX4 Auto Land handover height: '
            f'{self.handover_height:.2f} m'
        )

    # -------------------------------------------------
    # PX4 callbacks
    # -------------------------------------------------

    def local_position_callback(self, msg):
        self.local_position = msg
        self.px4_timestamp = msg.timestamp
        self.px4_timestamp_received = time.monotonic()

    def attitude_callback(self, msg):
        self.attitude_q = list(msg.q)

    def status_callback(self, msg):

        self.vehicle_armed = (
            msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        )

        previous_offboard = self.offboard_active

        self.offboard_active = (
            msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        )

        self.auto_land_active = (
            msg.nav_state == self.auto_land_nav_state
        )

        # Confirm PX4 Auto Land handover.
        if (
            self.state == 'REQUEST_AUTO_LAND'
            and self.auto_land_active
        ):
            self.state = 'AUTO_LAND'

            self.get_logger().info(
                'PX4 entered AUTO_LAND. '
                'Stopping ROS 2 Offboard setpoint publishing.'
            )

        # Initial Offboard entry.
        if self.offboard_active and not previous_offboard:

            if self.state in ('PRESTREAM', 'HOLD_OFFBOARD'):
                self.initialize_hold_position()
                self.state = 'HOLD_OFFBOARD'

                self.get_logger().info(
                    'PX4 entered Offboard. Holding current position.'
                )

            elif self.state not in ('AUTO_LAND', 'LANDED'):
                self.get_logger().warning(
                    f'PX4 re-entered Offboard during {self.state}. '
                    'Preserving landing state.'
                )

        # PX4 automatically disarmed after landing.
        if (
            not self.vehicle_armed
            and self.state == 'AUTO_LAND'
        ):
            self.state = 'LANDED'

            self.get_logger().info(
                'PX4 disarmed after Auto Land. '
                'Autonomous landing completed.'
            )

    def command_ack_callback(self, msg):

        if msg.command == VehicleCommand.VEHICLE_CMD_DO_SET_MODE:

            if msg.result == VehicleCommand.VEHICLE_CMD_RESULT_ACCEPTED:

                if self.state == 'REQUEST_AUTO_LAND':
                    self.auto_land_ack_received = True

                    self.get_logger().info(
                        'PX4 accepted Auto Land mode request. '
                        'Waiting for navigation-state confirmation.'
                    )

                else:
                    self.offboard_ack_received = True

                    self.get_logger().info(
                        'PX4 accepted Offboard mode command.'
                    )

            else:
                self.get_logger().warning(
                    f'DO_SET_MODE rejected. ACK={msg.result}'
                )

    def land_detected_callback(self, msg):

        self.px4_landed = bool(getattr(msg, 'landed', False))

        if self.px4_landed and not self.landed_logged:
            self.landed_logged = True

            self.get_logger().info(
                'PX4 land detector reports landed=true.'
            )

        elif not self.px4_landed:
            self.landed_logged = False

    # -------------------------------------------------
    # Quaternion utilities
    # -------------------------------------------------

    @staticmethod
    def rotate_vector(q, vector):
        """Rotate vector using Hamilton quaternion [w, x, y, z]."""

        w, x, y, z = q
        vx, vy, vz = vector

        tx = 2.0 * (y * vz - z * vy)
        ty = 2.0 * (z * vx - x * vz)
        tz = 2.0 * (x * vy - y * vx)

        return [
            vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx),
        ]

    # -------------------------------------------------
    # Marker callback
    # -------------------------------------------------

    def marker_callback(self, msg):

        if msg.header.frame_id != 'base_link':
            self.get_logger().warning(
                f'Unexpected marker frame: {msg.header.frame_id}'
            )
            return

        if self.local_position is None or self.attitude_q is None:
            return

        if (
            not self.local_position.xy_valid
            or not self.local_position.z_valid
        ):
            return

        # ROS FLU -> PX4 FRD
        marker_frd = [
            msg.pose.position.x,
            -msg.pose.position.y,
            -msg.pose.position.z,
        ]

        # PX4 body FRD -> local NED
        marker_ned_offset = self.rotate_vector(
            self.attitude_q,
            marker_frd
        )

        measured_marker_world = [
            self.local_position.x + marker_ned_offset[0],
            self.local_position.y + marker_ned_offset[1],
            self.local_position.z + marker_ned_offset[2],
        ]

        if self.marker_world is None:
            self.marker_world = measured_marker_world

        else:
            a = self.filter_alpha

            self.marker_world = [
                (1.0 - a) * old + a * new
                for old, new in zip(
                    self.marker_world,
                    measured_marker_world
                )
            ]

        self.marker_last_seen = time.monotonic()

    # -------------------------------------------------
    # PX4 timestamp
    # -------------------------------------------------

    def get_px4_timestamp(self):

        if self.px4_timestamp_received is None:
            return 0

        elapsed_us = int(
            (time.monotonic() - self.px4_timestamp_received) * 1e6
        )

        return self.px4_timestamp + elapsed_us

    # -------------------------------------------------
    # Setpoint publishing
    # -------------------------------------------------

    def initialize_hold_position(self):

        self.command_x = float(self.local_position.x)
        self.command_y = float(self.local_position.y)
        self.command_z = float(self.local_position.z)

    def publish_setpoint(self):

        if self.local_position is None:
            return

        if self.command_x is None:
            self.initialize_hold_position()

        timestamp = self.get_px4_timestamp()

        mode = OffboardControlMode()
        mode.timestamp = timestamp
        mode.position = True
        mode.velocity = False
        mode.acceleration = False
        mode.attitude = False
        mode.body_rate = False
        mode.thrust_and_torque = False
        mode.direct_actuator = False

        self.offboard_pub.publish(mode)

        setpoint = TrajectorySetpoint()
        setpoint.timestamp = timestamp

        setpoint.position = [
            float(self.command_x),
            float(self.command_y),
            float(self.command_z),
        ]

        setpoint.velocity = [float('nan')] * 3
        setpoint.acceleration = [float('nan')] * 3
        setpoint.jerk = [float('nan')] * 3

        setpoint.yaw = float(self.local_position.heading)
        setpoint.yawspeed = float('nan')

        self.setpoint_pub.publish(setpoint)

    # -------------------------------------------------
    # PX4 vehicle commands
    # -------------------------------------------------

    def send_vehicle_command(
        self,
        command_id,
        param1,
        param2=0.0,
        param3=0.0
    ):

        command = VehicleCommand()
        command.timestamp = self.get_px4_timestamp()

        command.command = command_id
        command.param1 = float(param1)
        command.param2 = float(param2)
        command.param3 = float(param3)

        command.target_system = 1
        command.target_component = 1
        command.source_system = 1
        command.source_component = 1
        command.confirmation = 0
        command.from_external = True

        self.vehicle_command_pub.publish(command)

    def request_offboard_mode(self):

        now = time.monotonic()

        if now - self.last_mode_request < self.mode_request_interval:
            return

        self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            1.0,
            6.0
        )

        self.last_mode_request = now

        self.get_logger().info(
            'Sent PX4 Offboard mode request.'
        )

    def request_auto_land_mode(self):

        now = time.monotonic()

        if (
            now - self.last_auto_land_request
            < self.auto_land_request_interval
        ):
            return

        # MAV_CMD_DO_SET_MODE:
        # param1 = 1: custom mode enabled
        # param2 = 4: PX4 AUTO main mode
        # param3 = 6: PX4 AUTO LAND submode

        self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            1.0,
            4.0,
            6.0
        )

        self.last_auto_land_request = now

        self.get_logger().info(
            'Requested PX4 AUTO_LAND mode. '
            'Continuing Offboard setpoints until confirmed.'
        )

    # -------------------------------------------------
    # Movement helpers
    # -------------------------------------------------

    def move_horizontal_towards_target(
        self,
        target_x,
        target_y,
        dt
    ):

        dx = target_x - self.command_x
        dy = target_y - self.command_y

        distance = math.sqrt(dx * dx + dy * dy)
        max_step = self.horizontal_speed * dt

        if distance > max_step and distance > 0.0:
            scale = max_step / distance
            dx *= scale
            dy *= scale

        self.command_x += dx
        self.command_y += dy

    def marker_is_visible(self, now):

        return (
            self.marker_last_seen is not None
            and now - self.marker_last_seen <= self.marker_timeout
            and self.marker_world is not None
        )

    def height_above_marker(self):

        if self.marker_world is None:
            return None

        # NED: larger Z means further down.
        return self.marker_world[2] - self.local_position.z

    def begin_auto_land_handover(self, height):

        self.state = 'REQUEST_AUTO_LAND'

        self.last_auto_land_request = 0.0
        self.auto_land_ack_received = False

        self.get_logger().info(
            f'Handover height reached: {height:.2f} m. '
            'Requesting PX4 Auto Land.'
        )

    # -------------------------------------------------
    # Main control loop
    # -------------------------------------------------

    def control_loop(self):

        now = time.monotonic()

        dt = min(
            now - self.last_control_time,
            0.1
        )

        self.last_control_time = now

        if self.local_position is None:
            return

        if (
            not self.local_position.xy_valid
            or not self.local_position.z_valid
        ):
            return

        if self.attitude_q is None:
            return

        # Once PX4 confirms Auto Land, ROS 2 must stop
        # publishing OffboardControlMode and setpoints.
        if self.state in ('AUTO_LAND', 'LANDED'):
            return

        # During the Auto Land request, continue publishing
        # Offboard setpoints until PX4 confirms the transition.
        if not self.offboard_active:
            self.initialize_hold_position()

        self.publish_setpoint()

        if self.prestream_started is None:
            self.prestream_started = now

        request_enabled = self.get_parameter(
            'request_offboard'
        ).value

        if (
            request_enabled
            and self.vehicle_armed
            and not self.offboard_active
            and self.state in ('PRESTREAM', 'HOLD_OFFBOARD')
            and now - self.prestream_started >= 1.5
        ):
            self.request_offboard_mode()

        if not self.offboard_active:
            return

        landing_enabled = self.get_parameter(
            'enable_landing'
        ).value

        if not landing_enabled:
            return

        marker_visible = self.marker_is_visible(now)

        # -------------------------------------------------
        # HOLD_OFFBOARD
        # -------------------------------------------------

        if self.state == 'HOLD_OFFBOARD':

            if not marker_visible:
                return

            self.state = 'ALIGN'

            self.get_logger().info(
                'Marker acquired. Starting alignment.'
            )

        # -------------------------------------------------
        # ALIGN
        # -------------------------------------------------

        if self.state == 'ALIGN':

            if not marker_visible:
                self.state = 'HOLD_OFFBOARD'
                self.initialize_hold_position()

                self.get_logger().warning(
                    'Marker lost during alignment. '
                    'Returning to position hold.'
                )
                return

            target_x = self.marker_world[0]
            target_y = self.marker_world[1]

            self.move_horizontal_towards_target(
                target_x,
                target_y,
                dt
            )

            error_x = self.local_position.x - target_x
            error_y = self.local_position.y - target_y

            horizontal_error = math.sqrt(
                error_x ** 2 + error_y ** 2
            )

            if horizontal_error <= self.alignment_tolerance:

                self.state = 'DESCEND'

                self.get_logger().info(
                    'Aligned over marker. Starting descent.'
                )

        # -------------------------------------------------
        # DESCEND
        # -------------------------------------------------

        elif self.state == 'DESCEND':

            height = self.height_above_marker()

            if height is None:
                return

            if not marker_visible:

                # If the marker is lost too high, stop descending.
                if height > self.terminal_transition_height:

                    self.state = 'HOLD_OFFBOARD'
                    self.initialize_hold_position()

                    self.get_logger().warning(
                        'Marker lost above terminal transition height. '
                        'Pausing descent and holding position.'
                    )
                    return

                # Below terminal transition height, use the
                # last valid marker world position.
                self.get_logger().warning(
                    'Marker lost near ground. '
                    'Continuing using cached marker position.'
                )

            else:

                # Continue horizontal correction while visible.
                self.move_horizontal_towards_target(
                    self.marker_world[0],
                    self.marker_world[1],
                    dt
                )

            # Handover to PX4 at the configured height.
            if height <= self.handover_height:

                self.begin_auto_land_handover(height)
                self.request_auto_land_mode()
                return

            # NED: positive Z is downward.
            self.command_z += self.descent_speed * dt

        # -------------------------------------------------
        # REQUEST_AUTO_LAND
        # -------------------------------------------------

        elif self.state == 'REQUEST_AUTO_LAND':

            # Hold the last Offboard setpoint while requesting
            # Auto Land. Do not continue descending in Offboard.
            self.request_auto_land_mode()

        # -------------------------------------------------
        # AUTO_LAND / LANDED
        # -------------------------------------------------

        # These states are handled by status_callback.
        # No Offboard setpoints or manual disarm commands
        # are sent after PX4 takes over.


def main(args=None):

    rclpy.init(args=args)

    node = PrecisionLandingController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.get_logger().info(
            'Precision landing controller stopped.'
        )

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
