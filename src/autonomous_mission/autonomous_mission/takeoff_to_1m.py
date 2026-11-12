
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)


class TakeoffTo1m(Node):
    """Take off to a configurable altitude and hold position."""

    def __init__(self):
        super().__init__("takeoff_to_1m")

        # Configurable target altitude in metres above the local origin.
        self.declare_parameter("target_altitude", 0.5)
        self.target_altitude = float(
            self.get_parameter("target_altitude").value
        )

        if not 0.1 <= self.target_altitude <= 2.0:
            raise ValueError(
                "target_altitude must be between 0.1 and 2.0 metres."
            )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.mode_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", 10
        )
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10
        )
        self.command_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", 10
        )

        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self.position_callback,
            qos,
        )
        self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status_v4",
            self.status_callback,
            qos,
        )

        self.position = None
        self.status = None

        self.target_x = None
        self.target_y = None
        self.target_z = -self.target_altitude

        self.get_logger().info(
            f"Configured target altitude: {self.target_altitude:.2f} m"
        )

    def position_callback(self, msg):
        self.position = msg

    def status_callback(self, msg):
        self.status = msg

    def timestamp(self):
        return self.get_clock().now().nanoseconds // 1000

    def publish_setpoint(self):
        mode = OffboardControlMode()
        mode.timestamp = self.timestamp()
        mode.position = True
        mode.velocity = False
        mode.acceleration = False
        mode.attitude = False
        mode.body_rate = False
        self.mode_pub.publish(mode)

        sp = TrajectorySetpoint()
        sp.timestamp = self.timestamp()
        sp.position = [self.target_x, self.target_y, self.target_z]
        sp.velocity = [math.nan, math.nan, math.nan]
        sp.acceleration = [math.nan, math.nan, math.nan]
        sp.jerk = [math.nan, math.nan, math.nan]
        sp.yaw = float(self.position.heading) if self.position else 0.0
        sp.yawspeed = math.nan
        self.setpoint_pub.publish(sp)

    def command(self, command, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.timestamp()
        msg.param1 = float(p1)
        msg.param2 = float(p2)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = TakeoffTo1m()

        # Wait for PX4 telemetry.
        deadline = time.monotonic() + 10.0

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

            if node.position is not None and node.status is not None:
                break

        p = node.position

        if (
            p is None
            or node.status is None
            or not p.xy_valid
            or not p.z_valid
        ):
            raise RuntimeError(
                "PX4 telemetry unavailable or position invalid."
            )

        if node.status.arming_state != VehicleStatus.ARMING_STATE_DISARMED:
            raise RuntimeError("Vehicle is not disarmed. Takeoff aborted.")

        if abs(float(p.z)) > 0.35:
            raise RuntimeError(
                f"Unexpected starting z={p.z:.2f} m; expected near ground."
            )

        # Hold the initial horizontal position.
        node.target_x = float(p.x)
        node.target_y = float(p.y)

        node.get_logger().info(
            f"Starting at x={p.x:.2f}, y={p.y:.2f}, z={p.z:.2f}; "
            f"target z={node.target_z:.2f} m "
            f"(altitude {node.target_altitude:.2f} m)"
        )

        # Pre-stream setpoints for PX4 Offboard mode.
        prestream_deadline = time.monotonic() + 2.0

        while rclpy.ok() and time.monotonic() < prestream_deadline:
            node.publish_setpoint()
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.05)

        node.get_logger().info("Requesting Offboard mode.")

        node.command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            1.0,
            6.0,
        )

        time.sleep(0.3)
        rclpy.spin_once(node, timeout_sec=0.1)

        node.get_logger().info("Requesting arm.")

        node.command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            1.0,
        )

        start = time.monotonic()
        last_log = 0.0
        reached = False

        while rclpy.ok():
            node.publish_setpoint()
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.05)

            elapsed = time.monotonic() - start
            p = node.position
            status = node.status

            if (
                status
                and status.arming_state == VehicleStatus.ARMING_STATE_DISARMED
                and elapsed > 1.0
            ):
                raise RuntimeError(
                    "PX4 remains disarmed. Check PX4 logs and command ACKs."
                )

            if p is not None and p.z_valid:
                altitude = -float(p.z)

                if (
                    abs(altitude - node.target_altitude) < 0.10
                    and abs(float(p.vz)) < 0.25
                ):
                    if not reached:
                        reached = True
                        node.get_logger().info(
                            f"Reached approximately "
                            f"{node.target_altitude:.2f} m. "
                            "Holding position; keep this node running."
                        )

                if time.monotonic() - last_log >= 1.0:
                    node.get_logger().info(
                        f"Local position z={p.z:.2f} m, "
                        f"estimated height={altitude:.2f} m, "
                        f"target={node.target_altitude:.2f} m"
                    )
                    last_log = time.monotonic()

            if elapsed > 60.0 and not reached:
                node.get_logger().error(
                    "Takeoff timed out. Requesting PX4 land mode."
                )
                node.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                break

    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().warning(
                "Stop requested. Sending PX4 land command; "
                "monitor the vehicle."
            )
            node.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

            end = time.monotonic() + 1.0

            while rclpy.ok() and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.1)

    except Exception as exc:
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print(f"Takeoff node initialization failed: {exc}")

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
