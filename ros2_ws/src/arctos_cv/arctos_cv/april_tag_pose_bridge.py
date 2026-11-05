"""Estimate calibrated AprilTag poses and transform them into Gazebo targets."""
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
import tf2_geometry_msgs  # noqa: F401
from tf2_ros import Buffer, TransformException, TransformListener


class AprilTagPoseBridge(Node):
    def __init__(self):
        super().__init__('april_tag_pose_bridge')
        path = self.declare_parameter('calibration_path', '').value
        size = self.declare_parameter('tag_size', 0.06).value
        self.tag_id = self.declare_parameter('tag_id', 0).value
        self.camera_frame = self.declare_parameter(
            'camera_frame', 'physical_camera_optical_frame').value
        self.target_frame = self.declare_parameter('target_frame', 'world').value
        offset = self.declare_parameter('target_offset', [0.0, 0.0, 0.0]).value
        self.offset = np.asarray(offset, dtype=float)
        if size <= 0 or not np.isfinite(size):
            raise ValueError('tag_size must be positive and finite')
        if self.offset.shape != (3,) or not np.isfinite(self.offset).all():
            raise ValueError('target_offset must contain three finite tag-frame meters')
        with np.load(path, allow_pickle=False) as data:
            self.K = data['K'].astype(np.float64)
            self.dist = data['dist'].astype(np.float64)
            self.image_size = tuple(map(int, data['image_size']))
        if (self.K.shape != (3, 3) or not np.isfinite(self.K).all()
                or not np.isfinite(self.dist).all()):
            raise ValueError('Invalid camera calibration')
        half = size / 2
        self.points = np.array([
            [-half, half, 0], [half, half, 0],
            [half, -half, 0], [-half, -half, 0]], dtype=np.float32)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        # Ubuntu Jazzy's OpenCV can predate ArucoDetector.
        self.detector = (cv2.aruco.ArucoDetector(dictionary)
                         if hasattr(cv2.aruco, 'ArucoDetector') else None)
        self.dictionary = dictionary
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(PoseStamped, 'desired_pose', 10)
        self.image_pub = self.create_publisher(
            Image, 'camera/image_with_tags', qos_profile_sensor_data)
        self.create_subscription(
            Image, 'camera/image_raw', self.on_image, qos_profile_sensor_data)

    def on_image(self, msg):
        if (msg.width, msg.height) != self.image_size:
            self.get_logger().error(
                'Camera resolution differs from calibration; dropping image',
                throttle_duration_sec=5.0)
            return
        gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        corners, ids, _ = (self.detector.detectMarkers(gray) if self.detector
                           else cv2.aruco.detectMarkers(gray, self.dictionary))
        display = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        label = f'NO TARGET: waiting for tag {self.tag_id}'
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(display, corners, ids)
            if self.tag_id in ids.flatten():
                label = f'Tag {self.tag_id} detected'
        cv2.putText(display, label, (20, 35), cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, (0, 255, 0) if ids is not None and self.tag_id in ids.flatten()
                    else (0, 0, 255), 2)
        annotated = self.bridge.cv2_to_imgmsg(display, encoding='bgr8')
        annotated.header = msg.header
        self.image_pub.publish(annotated)
        if ids is None:
            return
        for index, tag_id in enumerate(ids.flatten()):
            if int(tag_id) != self.tag_id:
                continue
            ok, rvec, tvec = cv2.solvePnP(
                self.points, corners[index].reshape(4, 2), self.K, self.dist,
                flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if (not ok or not np.isfinite(tvec).all()
                    or not np.isfinite(rvec).all() or tvec[2, 0] <= 0):
                continue
            rotation, _ = cv2.Rodrigues(rvec)
            position = tvec.reshape(3) + rotation @ self.offset
            # Rodrigues vector is axis * angle; convert directly to xyzw.
            angle = float(np.linalg.norm(rvec))
            xyz = rvec.reshape(3) * (np.sin(angle / 2) / angle if angle > 1e-12 else 0.5)
            pose = PoseStamped()
            pose.header.stamp = msg.header.stamp
            pose.header.frame_id = self.camera_frame
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, position)
            q = pose.pose.orientation
            q.x, q.y, q.z = map(float, xyz)
            q.w = float(np.cos(angle / 2))
            try:
                if self.camera_frame != self.target_frame:
                    pose = self.tf_buffer.transform(pose, self.target_frame)
            except TransformException as exc:
                self.get_logger().warn(f'No camera-to-target transform: {exc}',
                                       throttle_duration_sec=2.0)
                return
            self.get_logger().info(
                f'Tracking tag {self.tag_id}: world target '
                f'({pose.pose.position.x:.3f}, {pose.pose.position.y:.3f}, '
                f'{pose.pose.position.z:.3f}) m', throttle_duration_sec=3.0)
            self.pub.publish(pose)
            return


def main():
    rclpy.init()
    node = None
    try:
        node = AprilTagPoseBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
