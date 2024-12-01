#! /usr/bin/env python3

#camera_node.py
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.publisher_ = self.create_publisher(Image, 'camera_frames', 10)
        self.cap = cv2.VideoCapture(0)  # Webcam
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.03, self.publish_frame)  # 30 FPS

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture image')
            return
        # Convert OpenCV image to ROS Image message
        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher_.publish(ros_image)
        self.get_logger().info('Published frame')

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
