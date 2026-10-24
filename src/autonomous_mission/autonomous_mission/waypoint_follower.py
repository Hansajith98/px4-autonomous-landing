import math

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.node import Node
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)


class WaypointFollower(Node):
    """Simple PX4 Offboard waypoint mission for SITL."""

    def __init__(self):
        super().__init__('waypoint_follower')

        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)
        
        px4_qos = QoSProfile(depth=1)
        px4_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.position_callback,
            px4_qos,
        )
        self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v4',
            self.status_callback,
            px4_qos,
        )

        self.position = None
        self.status = None
        self.start_position = None
        self.waypoints = []
        self.waypoint_index = 0
        self.phase = 'WAIT_FOR_POSITION'
        self.phase_ticks = 0
        self.reached_ticks = 0
        self.finished = False

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(
            'Waypoint mission ready. Waiting for valid PX4 local position.')

    def position_callback(self, msg):
        self.position = msg

    def status_callback(self, msg):
        self.status = msg

    def timestamp_us(self):
        return self.get_clock().now().nanoseconds // 1000

    def publish_setpoint(self, x, y, z, yaw=0.0):
        mode = OffboardControlMode()
        mode.timestamp = self.timestamp_us()
        mode.position = True
        mode.velocity = False
        mode.acceleration = False
        mode.attitude = False
        mode.body_rate = False
        mode_pub = mode

        sp = TrajectorySetpoint()
        sp.timestamp = self.timestamp_us()
        sp.position = [float(x), float(y), float(z)]
        sp.velocity = [math.nan, math.nan, math.nan]
        sp.acceleration = [math.nan, math.nan, math.nan]
        sp.jerk = [math.nan, math.nan, math.nan]
        sp.yaw = float(yaw)
        sp.yawspeed = math.nan

        self.offboard_pub.publish(mode_pub)
        self.setpoint_pub.publish(sp)

    def send_command(self, command, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.timestamp_us()
        msg.command = command
        msg.param1 = float(p1)
        msg.param2 = float(p2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def build_route(self):
        x, y, z = self.start_position
        flight_z = z - 3.0  # NED: negative Z is above the starting point
        self.waypoints = [
            (x, y, flight_z),           # climb
            (x + 5.0, y, flight_z),     # north
            (x + 5.0, y + 5.0, flight_z),
            (x, y + 5.0, flight_z),
            (x, y, flight_z),           # return above start
        ]

    def control_loop(self):
        if self.finished:
            return

        if self.position is None or self.status is None:
            return

        if not self.position.xy_valid or not self.position.z_valid:
            if self.phase == 'WAIT_FOR_POSITION':
                self.get_logger().info(
                    'Waiting for valid local position estimate...',
                    throttle_duration_sec=5.0)
            return

        if self.phase == 'WAIT_FOR_POSITION':
            self.start_position = (
                self.position.x, self.position.y, self.position.z)
            self.build_route()
            self.phase = 'PRESTREAM'
            self.phase_ticks = 0
            self.get_logger().info(
                'Local position valid. Streaming initial setpoints.')

        if self.phase == 'PRESTREAM':
            self.publish_setpoint(*self.waypoints[0])
            self.phase_ticks += 1
            if self.phase_ticks >= 20:
                self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                self.phase = 'ARMING'
                self.phase_ticks = 0
                self.get_logger().info('Arm command sent.')
            return

        if self.phase == 'ARMING':
            self.publish_setpoint(*self.waypoints[0])
            if self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.phase = 'ENTER_OFFBOARD'
                self.phase_ticks = 0
                self.get_logger().info('Armed. Requesting Offboard mode.')
                return

            self.phase_ticks += 1
            if self.phase_ticks % 10 == 0:
                self.send_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            if self.phase_ticks > 100:
                self.abort('Arm was not confirmed within 10 seconds.')
            return

        if self.phase == 'ENTER_OFFBOARD':
            self.publish_setpoint(*self.waypoints[0])
            if self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.phase = 'FOLLOW_ROUTE'
                self.waypoint_index = 0
                self.get_logger().info(
                    'Offboard active. Starting waypoint route.')
                return

            self.phase_ticks += 1
            if self.phase_ticks % 10 == 0:
                self.send_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            if self.phase_ticks > 100:
                self.abort('Offboard mode was not confirmed within 10 seconds.')
            return

        if self.phase == 'FOLLOW_ROUTE':
            target = self.waypoints[self.waypoint_index]
            self.publish_setpoint(*target)

            dx = self.position.x - target[0]
            dy = self.position.y - target[1]
            dz = self.position.z - target[2]
            distance = math.sqrt(dx * dx + dy * dy + dz * dz)

            if distance < 0.35:
                self.reached_ticks += 1
            else:
                self.reached_ticks = 0

            if self.reached_ticks >= 10:
                self.get_logger().info(
                    f'Waypoint {self.waypoint_index + 1}/'
                    f'{len(self.waypoints)} reached.')
                self.waypoint_index += 1
                self.reached_ticks = 0

                if self.waypoint_index >= len(self.waypoints):
                    self.phase = 'LAND'
                    self.get_logger().info(
                        'Route complete. Requesting landing.')
                return

        elif self.phase == 'LAND':
            # Keep setpoints alive while requesting PX4's land mode.
            self.publish_setpoint(
                self.position.x, self.position.y, self.position.z)
            self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.phase = 'LANDING'
            self.phase_ticks = 0

        elif self.phase == 'LANDING':
            self.phase_ticks += 1
            if self.status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                self.finished = True
                self.get_logger().info('Landing complete; vehicle disarmed.')
            elif self.phase_ticks > 300:
                self.get_logger().error(
                    'Landing has not completed after 30 seconds. '
                    'Check vehicle state in QGroundControl.')

    def abort(self, reason):
        self.get_logger().error(f'Mission stopped: {reason}')
        self.finished = True
        # Do not issue a blind disarm command in flight.
        # Use QGroundControl to select a safe mode or land if needed.


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warning(
            'Controller interrupted. Check QGroundControl and vehicle mode.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()