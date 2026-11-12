import json
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


class LiveArucoDetector(Node):
    """Detect ArUco markers in the live Gazebo camera stream."""

    def __init__(self):
        super().__init__('live_aruco_detector')

        self.declare_parameter(
            'image_topic',
            '/world/lawn/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image'
        )
        self.declare_parameter('dictionary', 'DICT_4X4_50')

        image_topic = self.get_parameter('image_topic').value
        dictionary_name = self.get_parameter('dictionary').value

        if not hasattr(cv2.aruco, dictionary_name):
            raise ValueError(f'Unsupported ArUco dictionary: {dictionary_name}')

        dictionary_id = getattr(cv2.aruco, dictionary_name)
        self.dictionary = cv2.aruco.Dictionary_get(dictionary_id)
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.bridge = CvBridge()

        self.detection_pub = self.create_publisher(
            String, '/landing/aruco_detections', 10
        )
        self.annotated_pub = self.create_publisher(
            Image, '/landing/aruco_annotated', 10
        )

        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.frames_since_report = 0
        self.last_report_time = time.monotonic()
        self.total_frames = 0

        self.get_logger().info(f'OpenCV version: {cv2.__version__}')
        self.get_logger().info(f'ArUco dictionary: {dictionary_name}')
        self.get_logger().info(f'Subscribing to: {image_topic}')
        self.get_logger().info('Publishing detections: /landing/aruco_detections')
        self.get_logger().info('Publishing annotated images: /landing/aruco_annotated')

    def image_callback(self, msg):
        try:
            # cv_bridge converts ROS rgb8 images to OpenCV BGR images.
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray,
                self.dictionary,
                parameters=self.parameters,
            )

            detections = []

            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                for index, marker_id in enumerate(ids.flatten()):
                    points = corners[index][0]
                    center_x = float(points[:, 0].mean())
                    center_y = float(points[:, 1].mean())

                    detections.append({
                        'id': int(marker_id),
                        'center_px': {
                            'x': center_x,
                            'y': center_y,
                        },
                        'corners_px': [
                            {'x': float(point[0]), 'y': float(point[1])}
                            for point in points
                        ],
                    })

                    cv2.circle(
                        frame,
                        (round(center_x), round(center_y)),
                        5,
                        (0, 255, 0),
                        -1,
                    )
                    cv2.putText(
                        frame,
                        f'ID {int(marker_id)}',
                        (round(center_x) + 10, round(center_y)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 0),
                        2,
                    )

            result = String()
            result.data = json.dumps({
                'image_stamp': {
                    'sec': msg.header.stamp.sec,
                    'nanosec': msg.header.stamp.nanosec,
                },
                'image_width': msg.width,
                'image_height': msg.height,
                'marker_count': len(detections),
                'detections': detections,
            })
            self.detection_pub.publish(result)

            annotated_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            annotated_msg.header = msg.header
            self.annotated_pub.publish(annotated_msg)

            self.total_frames += 1
            self.frames_since_report += 1

            now = time.monotonic()
            elapsed = now - self.last_report_time

            if elapsed >= 5.0:
                fps = self.frames_since_report / elapsed
                marker_ids = [item['id'] for item in detections]
                self.get_logger().info(
                    f'Processing rate: {fps:.1f} Hz | '
                    f'Latest marker IDs: {marker_ids}'
                )
                self.frames_since_report = 0
                self.last_report_time = now

        except Exception as exc:
            self.get_logger().error(f'Image processing failed: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = LiveArucoDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()