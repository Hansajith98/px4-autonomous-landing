
import json
import math

import cv2
import numpy as np
import rclpy


from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, TransformException

from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped


class ArucoPoseEstimator(Node):

    def __init__(self):
        super().__init__("aruco_pose_estimator")

        # Configuration parameters
        self.declare_parameter("marker_size", 0.5)
        self.declare_parameter("marker_id", 0)

        self.declare_parameter(
            "detections_topic", "/landing/aruco_detections"
        )
        self.declare_parameter(
            "camera_info_topic",
            "/world/lawn/model/x500_mono_cam_down_0/link/"
            "camera_link/sensor/camera/camera_info"
        )
        self.declare_parameter(
            "pose_topic", "/landing/target_pose"
        )

        self.marker_size = float(
            self.get_parameter("marker_size").value
        )
        self.marker_id = int(
            self.get_parameter("marker_id").value
        )

        if self.marker_size <= 0.0:
            raise ValueError("marker_size must be positive.")

        detections_topic = (
            self.get_parameter("detections_topic").value
        )
        camera_info_topic = (
            self.get_parameter("camera_info_topic").value
        )
        pose_topic = self.get_parameter("pose_topic").value

        # Camera calibration (received from CameraInfo)
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_frame_id = None

        # Object points in the marker coordinate system.
        # Required ordering for SOLVEPNP_IPPE_SQUARE.
        half_size = self.marker_size / 2.0

        self.object_points = np.array(
            [
                [-half_size, half_size, 0.0],
                [half_size, half_size, 0.0],
                [half_size, -half_size, 0.0],
                [-half_size, -half_size, 0.0],
            ],
            dtype=np.float32,
        )

        self.pose_pub = self.create_publisher(
            PoseStamped,
            pose_topic,
            10
        )

        # TF2 configuration
        self.target_frame = "base_link"

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # Publish marker pose relative to drone body
        self.body_pose_pub = self.create_publisher(
            PoseStamped,
            "/landing/target_pose_body",
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            qos_profile_sensor_data
        )

        self.detection_sub = self.create_subscription(
            String,
            detections_topic,
            self.detection_callback,
            10
        )

        self.get_logger().info(
            "ArUco pose estimator initialized."
        )
        self.get_logger().info(
            f"Marker ID: {self.marker_id}, "
            f"marker size: {self.marker_size:.3f} m"
        )
        self.get_logger().info(
            f"Waiting for camera calibration on {camera_info_topic}"
        )

    def camera_info_callback(self, msg):

        self.camera_matrix = np.array(
            msg.k, dtype=np.float64
        ).reshape(3, 3)

        self.dist_coeffs = np.array(
            msg.d, dtype=np.float64
        )

        # solvePnP returns coordinates in the camera optical
        # convention: X right, Y down, Z forward.
        self.camera_frame_id = "camera_optical_frame"

        self.get_logger().info(
            "Camera calibration received: "
            f"{msg.width}x{msg.height}, "
            f"fx={self.camera_matrix[0, 0]:.3f}, "
            f"fy={self.camera_matrix[1, 1]:.3f}",
            once=True
        )

    @staticmethod
    def rotation_matrix_to_quaternion(R):

        trace = np.trace(R)

        if trace > 0.0:
            s = math.sqrt(trace + 1.0) * 2.0
            qw = 0.25 * s
            qx = (R[2, 1] - R[1, 2]) / s
            qy = (R[0, 2] - R[2, 0]) / s
            qz = (R[1, 0] - R[0, 1]) / s

        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = math.sqrt(
                1.0 + R[0, 0] - R[1, 1] - R[2, 2]
            ) * 2.0
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s

        elif R[1, 1] > R[2, 2]:
            s = math.sqrt(
                1.0 + R[1, 1] - R[0, 0] - R[2, 2]
            ) * 2.0
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s

        else:
            s = math.sqrt(
                1.0 + R[2, 2] - R[0, 0] - R[1, 1]
            ) * 2.0
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s

        return qx, qy, qz, qw

    @staticmethod
    def rotate_vector(q, vector):
        """
        Rotate a 3D vector using quaternion q (x, y, z, w).
        """
        qv = np.array(
            [q.x, q.y, q.z],
            dtype=np.float64
        )

        v = np.array(vector, dtype=np.float64)

        return (
            v
            + 2.0 * np.cross(
                qv,
                np.cross(qv, v) + q.w * v
            )
        )

    @staticmethod
    def multiply_quaternions(q1, q2):
        """
        Quaternion multiplication: q1 * q2.
        """

        x = (
            q1.w * q2.x
            + q1.x * q2.w
            + q1.y * q2.z
            - q1.z * q2.y
        )

        y = (
            q1.w * q2.y
            - q1.x * q2.z
            + q1.y * q2.w
            + q1.z * q2.x
        )

        z = (
            q1.w * q2.z
            + q1.x * q2.y
            - q1.y * q2.x
            + q1.z * q2.w
        )

        w = (
            q1.w * q2.w
            - q1.x * q2.x
            - q1.y * q2.y
            - q1.z * q2.z
        )

        return x, y, z, w

    def transform_pose_to_body(self, camera_pose):
        """
        Transform marker pose from camera_optical_frame
        into base_link using TF2.
        """

        transform = self.tf_buffer.lookup_transform(
            self.target_frame,
            camera_pose.header.frame_id,
            Time()
        )

        translation = transform.transform.translation
        rotation = transform.transform.rotation

        # Transform position
        camera_position = [
            camera_pose.pose.position.x,
            camera_pose.pose.position.y,
            camera_pose.pose.position.z
        ]

        rotated_position = self.rotate_vector(
            rotation,
            camera_position
        )

        body_pose = PoseStamped()

        body_pose.header.stamp = camera_pose.header.stamp
        body_pose.header.frame_id = self.target_frame

        body_pose.pose.position.x = (
            rotated_position[0] + translation.x
        )

        body_pose.pose.position.y = (
            rotated_position[1] + translation.y
        )

        body_pose.pose.position.z = (
            rotated_position[2] + translation.z
        )

        # Transform orientation
        qx, qy, qz, qw = self.multiply_quaternions(
            rotation,
            camera_pose.pose.orientation
        )

        # Normalize quaternion
        norm = math.sqrt(
            qx * qx + qy * qy + qz * qz + qw * qw
        )

        body_pose.pose.orientation.x = qx / norm
        body_pose.pose.orientation.y = qy / norm
        body_pose.pose.orientation.z = qz / norm
        body_pose.pose.orientation.w = qw / norm

        return body_pose

    def detection_callback(self, msg):

        if self.camera_matrix is None:
            return

        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(
                f"Invalid detection JSON: {exc}"
            )
            return

        detections = data.get("detections", [])

        for detection in detections:

            if int(detection["id"]) != self.marker_id:
                continue

            corners = detection.get("corners_px", [])

            if len(corners) != 4:
                self.get_logger().warning(
                    "Expected four marker corners."
                )
                continue

            image_points = np.array(
                [
                    [corner["x"], corner["y"]]
                    for corner in corners
                ],
                dtype=np.float32
            )

            edge_width = np.linalg.norm(
                image_points[2] - image_points[1]
            )

            edge_height = np.linalg.norm(
                image_points[0] - image_points[1]
            )

            self.get_logger().info(
                f"Detected marker dimensions: "
                f"width={edge_width:.1f}px, "
                f"height={edge_height:.1f}px"
            )

            # Estimate marker pose relative to camera optical frame.
            success, rvec, tvec = cv2.solvePnP(
                self.object_points,
                image_points,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )

            if not success:
                self.get_logger().warning(
                    "solvePnP failed for marker."
                )
                continue

            # Calculate reprojection error as a quality indicator.
            projected_points, _ = cv2.projectPoints(
                self.object_points,
                rvec,
                tvec,
                self.camera_matrix,
                self.dist_coeffs
            )

            projected_points = projected_points.reshape(-1, 2)

            reprojection_error = float(
                np.mean(
                    np.linalg.norm(
                        projected_points - image_points,
                        axis=1
                    )
                )
            )

            rotation_matrix, _ = cv2.Rodrigues(rvec)

            qx, qy, qz, qw = (
                self.rotation_matrix_to_quaternion(
                    rotation_matrix
                )
            )

            # Use the image timestamp from the detector.
            stamp_data = data.get("image_stamp", {})

            pose_msg = PoseStamped()
            pose_msg.header.frame_id = self.camera_frame_id

            if "sec" in stamp_data and "nanosec" in stamp_data:
                pose_msg.header.stamp.sec = int(
                    stamp_data["sec"]
                )
                pose_msg.header.stamp.nanosec = int(
                    stamp_data["nanosec"]
                )
            else:
                pose_msg.header.stamp = (
                    self.get_clock().now().to_msg()
                )

            pose_msg.pose.position.x = float(tvec[0, 0])
            pose_msg.pose.position.y = float(tvec[1, 0])
            pose_msg.pose.position.z = float(tvec[2, 0])

            pose_msg.pose.orientation.x = qx
            pose_msg.pose.orientation.y = qy
            pose_msg.pose.orientation.z = qz
            pose_msg.pose.orientation.w = qw

            self.pose_pub.publish(pose_msg)

            # Transform marker pose into drone body frame
            try:
                body_pose = self.transform_pose_to_body(
                    pose_msg
                )

                self.body_pose_pub.publish(body_pose)

                self.get_logger().info(
                    f"Marker {self.marker_id} | "
                    f"Body position: "
                    f"x={body_pose.pose.position.x:.3f}, "
                    f"y={body_pose.pose.position.y:.3f}, "
                    f"z={body_pose.pose.position.z:.3f} m"
                )

            except TransformException as exc:
                self.get_logger().warning(
                    f"Could not transform marker pose: {exc}"
                )

            self.get_logger().info(
                f"Marker {self.marker_id} | "
                f"Camera position: "
                f"x={tvec[0, 0]:.3f}, "
                f"y={tvec[1, 0]:.3f}, "
                f"z={tvec[2, 0]:.3f} m | "
                f"Reprojection error={reprojection_error:.2f} px"
            )


def main(args=None):

    rclpy.init(args=args)

    node = ArucoPoseEstimator()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
