#!/usr/bin/env python3
import csv
from pathlib import Path
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Int32
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformListener


class VisionDetector(Node):
    def __init__(self):
        super().__init__('vision_aruco_detector')
        self.bridge = CvBridge()
        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.tf = Buffer(node=self)
        self.listener = TransformListener(self.tf, self)
        self.camera_matrix = None
        self.distortion = None
        self.found = set()
        self.target = None
        self.capture_after = None
        self.measuring = False
        self.retry_after = self.get_clock().now()

        h = 0.07 / 2
        self.object_points = np.array([
            [-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]
        ], dtype=np.float32)
        self.path = Path.home() / '.ros' / 'arm05_sim' / 'detected_aruco.csv'
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.write_text('id,x,y,z\n')

        self.candidate = self.create_publisher(Int32, '/aruco_candidate', 10)
        self.result = self.create_publisher(Int32, '/aruco_found', 10)
        self.create_subscription(Int32, '/aruco_capture', self.on_capture, 10)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_info, qos_profile_sensor_data)
        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, image_qos)

    def on_info(self, msg):
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.distortion = np.array(msg.d)

    def on_capture(self, msg):
        self.target = msg.data
        self.capture_after = self.get_clock().now()

    def on_image(self, msg):
        if self.measuring or self.camera_matrix is None or len(self.found) == 5:
            return
        stamp = Time.from_msg(msg.header.stamp)
        if self.target is not None and stamp <= self.capture_after:
            return
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        corners, ids, _ = cv2.aruco.detectMarkers(image, self.dictionary, parameters=self.parameters)
        markers = {} if ids is None else dict(zip(ids.flatten().tolist(), corners))

        if self.target is None:
            if stamp < self.retry_after:
                return
            for marker in markers:
                if marker in range(5) and marker not in self.found:
                    self.candidate.publish(Int32(data=marker))
                    break
            return

        marker = self.target
        self.target = None
        if marker not in markers:
            self.failed_capture(stamp)
            return
        ok, _, tvec = cv2.solvePnP(
            self.object_points, markers[marker].reshape(4, 2),
            self.camera_matrix, self.distortion, flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        if not ok:
            self.failed_capture(stamp)
            return

        point = PointStamped()
        point.header = msg.header
        point.header.frame_id = 'camera_rgb_optical_frame'
        point.point.x, point.point.y, point.point.z = tvec.flatten().tolist()
        self.measuring = True
        self.tf.wait_for_transform_async('map', point.header.frame_id, stamp).add_done_callback(
            lambda future: self.save_position(marker, point, future.result())
        )

    def failed_capture(self, stamp):
        self.retry_after = stamp + Duration(seconds=1.0)
        self.result.publish(Int32(data=-1))

    def save_position(self, marker, point, transform):
        p = do_transform_point(point, transform).point
        with self.path.open('a', newline='') as file:
            csv.writer(file).writerow([marker, round(p.x, 4), round(p.y, 4), round(p.z, 4)])
        self.found.add(marker)
        self.measuring = False
        self.get_logger().info(f'ArUco {marker}: ({p.x:.2f}, {p.y:.2f}, {p.z:.2f}) | {len(self.found)}/5')
        self.result.publish(Int32(data=marker))


def main():
    rclpy.init()
    node = VisionDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
