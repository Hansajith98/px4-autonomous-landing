import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint, VehicleCommand,
    VehicleLocalPosition, VehicleStatus, VehicleAttitude,
)


class AutonomousMission(Node):
    """Single PX4 Offboard owner: takeoff -> local NED route -> ArUco landing."""

    def __init__(self):
        super().__init__('autonomous_mission')

        self.declare_parameter('takeoff_altitude', 2.5)
        self.declare_parameter('north_distance', 3.0)
        self.declare_parameter('east_distance', 3.0)
        self.declare_parameter('waypoint_tolerance', 0.35)
        self.declare_parameter('marker_timeout', 0.5)
        self.declare_parameter('alignment_tolerance', 0.10)
        self.declare_parameter('horizontal_speed', 0.30)
        self.declare_parameter('descent_speed', 0.15)
        self.declare_parameter('terminal_transition_height', 0.60)
        self.declare_parameter('handover_height', 0.50)
        self.declare_parameter('filter_alpha', 0.25)
        self.declare_parameter('auto_land_request_interval', 2.0)

        self.takeoff_altitude = float(self.get_parameter('takeoff_altitude').value)
        self.north_distance = float(self.get_parameter('north_distance').value)
        self.east_distance = float(self.get_parameter('east_distance').value)
        self.waypoint_tolerance = float(self.get_parameter('waypoint_tolerance').value)
        self.marker_timeout = float(self.get_parameter('marker_timeout').value)
        self.alignment_tolerance = float(self.get_parameter('alignment_tolerance').value)
        self.horizontal_speed = float(self.get_parameter('horizontal_speed').value)
        self.descent_speed = float(self.get_parameter('descent_speed').value)
        self.terminal_transition_height = float(self.get_parameter('terminal_transition_height').value)
        self.handover_height = float(self.get_parameter('handover_height').value)
        self.filter_alpha = float(self.get_parameter('filter_alpha').value)
        self.auto_land_request_interval = float(self.get_parameter('auto_land_request_interval').value)

        px4_qos = QoSProfile(depth=1)
        px4_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.position_callback, px4_qos)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v4', self.status_callback, px4_qos)
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.attitude_callback, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/landing/target_pose_body', self.marker_callback, qos_profile_sensor_data)

        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)

        self.position = None
        self.status = None
        self.attitude_q = None
        self.start_position = None
        self.waypoints = []
        self.waypoint_index = 0
        self.reached_ticks = 0
        self.phase_ticks = 0
        self.phase = 'WAIT_FOR_POSITION'
        self.finished = False
        self.command_x = self.command_y = self.command_z = None
        self.marker_world = None
        self.marker_last_seen = None
        self.offboard_active = False
        self.vehicle_armed = False
        self.auto_land_active = False
        self.last_auto_land_request = 0.0
        self.last_control_time = time.monotonic()
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info('Mission ready: takeoff -> NED waypoints -> ArUco precision landing.')

    def timestamp_us(self):
        return self.get_clock().now().nanoseconds // 1000

    def position_callback(self, msg):
        self.position = msg

    def attitude_callback(self, msg):
        self.attitude_q = list(msg.q)

    def status_callback(self, msg):
        self.status = msg  # IMPORTANT: store received PX4 status

        self.vehicle_armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self.offboard_active = msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

        auto_land_state = getattr(VehicleStatus, 'NAVIGATION_STATE_AUTO_LAND', 18)
        self.auto_land_active = msg.nav_state == auto_land_state

        if self.phase == 'REQUEST_AUTO_LAND' and self.auto_land_active:
            self.phase = 'AUTO_LAND'
            self.get_logger().info('PX4 AUTO_LAND confirmed. ROS 2 setpoint publishing stopped.')

        if self.phase == 'AUTO_LAND' and not self.vehicle_armed:
            self.phase = 'LANDED'
            self.finished = True
            self.get_logger().info('PX4 disarmed. Autonomous mission completed.')

    @staticmethod
    def rotate_vector(q, v):
        w, x, y, z = q
        vx, vy, vz = v
        tx = 2.0 * (y * vz - z * vy)
        ty = 2.0 * (z * vx - x * vz)
        tz = 2.0 * (x * vy - y * vx)
        return [vx + w * tx + (y * tz - z * ty), vy + w * ty + (z * tx - x * tz), vz + w * tz + (x * ty - y * tx)]

    def marker_callback(self, msg):
        if msg.header.frame_id != 'base_link' or self.position is None or self.attitude_q is None:
            return
        if not self.position.xy_valid or not self.position.z_valid:
            return
        # Pose estimator publishes ROS FLU body coordinates; PX4 attitude is FRD/NED.
        marker_frd = [msg.pose.position.x, -msg.pose.position.y, -msg.pose.position.z]
        offset = self.rotate_vector(self.attitude_q, marker_frd)
        measured = [self.position.x + offset[0], self.position.y + offset[1], self.position.z + offset[2]]
        if self.marker_world is None:
            self.marker_world = measured
        else:
            a = self.filter_alpha
            self.marker_world = [(1.0-a)*old + a*new for old, new in zip(self.marker_world, measured)]
        self.marker_last_seen = time.monotonic()

    def publish_setpoint(self, x, y, z):
        ts = self.timestamp_us()
        mode = OffboardControlMode()
        mode.timestamp = ts
        mode.position = True
        mode.velocity = mode.acceleration = mode.attitude = mode.body_rate = False
        self.offboard_pub.publish(mode)
        sp = TrajectorySetpoint()
        sp.timestamp = ts
        sp.position = [float(x), float(y), float(z)]
        sp.velocity = [math.nan] * 3
        sp.acceleration = [math.nan] * 3
        sp.jerk = [math.nan] * 3
        sp.yaw = float(self.position.heading) if self.position is not None else 0.0
        sp.yawspeed = math.nan
        self.setpoint_pub.publish(sp)

    def send_command(self, command, p1=0.0, p2=0.0, p3=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.timestamp_us()
        msg.command = command
        msg.param1, msg.param2, msg.param3 = float(p1), float(p2), float(p3)
        msg.target_system = msg.source_system = 1
        msg.target_component = msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def build_route(self):
        x, y, z = self.start_position
        cruise_z = z - self.takeoff_altitude  # NED: negative Z is up
        self.waypoints = [
            (x, y, cruise_z),
            (x + self.north_distance, y, cruise_z),
            (x + self.north_distance, y + self.east_distance, cruise_z),
            (x, y + self.east_distance, cruise_z),
            (x, y, cruise_z),
        ]

    def move_horizontal_towards_target(self, tx, ty, dt):
        dx, dy = tx - self.command_x, ty - self.command_y
        distance = math.hypot(dx, dy)
        step = self.horizontal_speed * dt
        if distance > step and distance > 0.0:
            dx, dy = dx * step / distance, dy * step / distance
        self.command_x += dx
        self.command_y += dy

    def marker_visible(self, now):
        return self.marker_world is not None and self.marker_last_seen is not None and now - self.marker_last_seen <= self.marker_timeout

    def request_auto_land(self, now):
        if now - self.last_auto_land_request >= self.auto_land_request_interval:
            # PX4 custom mode: AUTO main mode, LAND submode.
            self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0, 6.0)
            self.last_auto_land_request = now
            self.get_logger().info('Requesting PX4 AUTO_LAND...')

    def control_loop(self):
        now = time.monotonic()
        dt = min(now - self.last_control_time, 0.1)
        self.last_control_time = now
        if self.finished or self.phase in ('AUTO_LAND', 'LANDED'):
            return
        if self.position is None or self.status is None:
            return
        if not self.position.xy_valid or not self.position.z_valid:
            return

        if self.phase == 'WAIT_FOR_POSITION':
            self.start_position = (self.position.x, self.position.y, self.position.z)
            self.command_x, self.command_y, self.command_z = self.start_position
            self.build_route()
            self.phase = 'PRESTREAM'
            self.get_logger().info('Valid local NED position. Pre-streaming hold setpoints.')

        if self.phase == 'PRESTREAM':
            self.publish_setpoint(*self.start_position)
            self.phase_ticks += 1
            if self.phase_ticks >= 40:  # 2 seconds at 20 Hz
                self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                self.phase = 'ARMING'
                self.phase_ticks = 0
                self.get_logger().info('Arm command sent.')
            return

        if self.phase == 'ARMING':
            self.publish_setpoint(*self.start_position)
            if self.vehicle_armed:
                self.phase = 'ENTER_OFFBOARD'
                self.phase_ticks = 0
                self.get_logger().info('Vehicle armed. Requesting Offboard.')
            else:
                self.phase_ticks += 1
                if self.phase_ticks % 20 == 0:
                    self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                if self.phase_ticks > 200:
                    self.abort('Arming was not confirmed within 10 seconds.')
            return

        if self.phase == 'ENTER_OFFBOARD':
            self.publish_setpoint(*self.start_position)
            if self.offboard_active:
                self.phase = 'TAKEOFF'
                self.get_logger().info(f'Offboard active. Taking off to {self.takeoff_altitude:.1f} m.')
            else:
                self.phase_ticks += 1
                if self.phase_ticks % 20 == 0:
                    self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                if self.phase_ticks > 200:
                    self.abort('Offboard mode was not confirmed within 10 seconds.')
            return

        if self.phase == 'TAKEOFF':
            target = self.waypoints[0]
            self.publish_setpoint(*target)
            if abs(self.position.z - target[2]) < self.waypoint_tolerance:
                self.phase = 'FOLLOW_ROUTE'
                self.waypoint_index = 1
                self.reached_ticks = 0
                self.get_logger().info('Takeoff altitude reached. Starting NED waypoint route.')
            return

        if self.phase == 'FOLLOW_ROUTE':
            target = self.waypoints[self.waypoint_index]
            self.publish_setpoint(*target)
            distance = math.sqrt(sum((a-b)**2 for a, b in zip((self.position.x, self.position.y, self.position.z), target)))
            self.reached_ticks = self.reached_ticks + 1 if distance < self.waypoint_tolerance else 0
            if self.reached_ticks >= 10:
                self.get_logger().info(f'Waypoint {self.waypoint_index}/{len(self.waypoints)-1} reached.')
                self.waypoint_index += 1
                self.reached_ticks = 0
                if self.waypoint_index >= len(self.waypoints):
                    self.phase = 'SEARCH_MARKER'
                    self.command_x, self.command_y, self.command_z = target
                    self.get_logger().info('Route complete. Holding position and waiting for ArUco marker.')
            return

        if self.phase in ('SEARCH_MARKER', 'ALIGN', 'DESCEND', 'REQUEST_AUTO_LAND'):
            # Keep one and only one Offboard stream alive through precision landing.
            self.publish_setpoint(self.command_x, self.command_y, self.command_z)

            if self.phase == 'SEARCH_MARKER':
                if self.marker_visible(now):
                    self.phase = 'ALIGN'
                    self.get_logger().info('ArUco marker acquired. Starting alignment.')
                return

            if self.phase == 'ALIGN':
                if not self.marker_visible(now):
                    self.get_logger().warning('Marker lost. Holding current position; waiting to reacquire.')
                    self.phase = 'SEARCH_MARKER'
                    return
                tx, ty = self.marker_world[0], self.marker_world[1]
                self.move_horizontal_towards_target(tx, ty, dt)
                error = math.hypot(self.position.x - tx, self.position.y - ty)
                if error <= self.alignment_tolerance:
                    self.phase = 'DESCEND'
                    self.get_logger().info('Aligned over marker. Starting controlled descent.')
                return

            if self.phase == 'DESCEND':
                height = self.marker_world[2] - self.position.z if self.marker_world else None
                visible = self.marker_visible(now)
                if not visible and height is not None and height > self.terminal_transition_height:
                    self.phase = 'SEARCH_MARKER'
                    self.command_x, self.command_y, self.command_z = self.position.x, self.position.y, self.position.z
                    self.get_logger().warning('Marker lost above terminal height. Pausing descent and holding.')
                    return
                if visible:
                    self.move_horizontal_towards_target(self.marker_world[0], self.marker_world[1], dt)
                if height is not None and height <= self.handover_height:
                    self.phase = 'REQUEST_AUTO_LAND'
                    self.last_auto_land_request = 0.0
                    self.get_logger().info(f'{height:.2f} m above marker. Handing over to PX4 Auto Land.')
                    return
                self.command_z += self.descent_speed * dt  # NED positive Z is down
                return

            if self.phase == 'REQUEST_AUTO_LAND':
                self.request_auto_land(now)
                return

    def abort(self, reason):
        self.get_logger().error(f'Mission aborted: {reason}. Check QGroundControl; no blind disarm sent.')
        self.finished = True


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warning('Mission interrupted. Check vehicle state in QGroundControl.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
