# ROS2 libraries
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

# Computer Vision libraries
import cv2
from cv_bridge import CvBridge


class AprilTagDetector(Node):
    def __init__(self):
        super().__init__('april_tag_detector')

        # Parameters
        self.frame = self.declare_parameter('frame', 'camera_link').value
        self.tag_family = self.declare_parameter('tag_family', 'tag36h11').value

        # AprilTag detector setup, through OpenCV's ArUco module
        # (tag16h5, tag25h9, tag36h10 or tag36h11)
        dictionary_id = getattr(cv2.aruco, 'DICT_APRILTAG_' + self.tag_family.removeprefix('tag'), None)
        if dictionary_id is None:
            raise ValueError(f'Tag family {self.tag_family} is not supported')
        self.detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(dictionary_id), cv2.aruco.DetectorParameters())

        # ROS2 subscriptions and publications
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, 'camera/image_raw', self.image_callback, qos_profile_sensor_data)
        self.pub = self.create_publisher(Image, 'camera/image_with_tags', qos_profile_sensor_data)
    
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Find the tags and outline them with their ids
        corners, ids, _ = self.detector.detectMarkers(frame)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # Publish the annotated image
        msg_out = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg_out.header.frame_id = self.frame
        msg_out.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg_out)   

def main():
    rclpy.init()
    node = None
    try:
        node = AprilTagDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()

if  __name__ == '__main__':
    main()