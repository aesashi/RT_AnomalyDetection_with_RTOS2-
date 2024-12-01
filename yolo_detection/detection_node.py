#! /usr/bin/env python3
#detectino_node.py
from ultralytics import YOLO
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker
import os
import time


class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')
        self.subscription = self.create_subscription(Image, 'camera_frames', self.process_frame, 10)
        self.marker_publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        self.bridge = CvBridge()
        self.model = YOLO("yolo-Weights/yolov8n.pt")  # Load YOLO model
        self.class_names = ["person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
                            "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
                            "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
                            "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
                            "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle",
                            "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich",
                            "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa",
                            "pottedplant", "bed", "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote",
                            "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book",
                            "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"]
        self.log_file = os.path.join(os.getcwd(), 'detection_log.txt')  # Log file
        self.deadline_ms = 100  # Initial deadline in milliseconds
        self.current_frame_rate = 30  # Initial frame rate in FPS
        self.start_time = time.time()  # Timer for demonstration
        self.deadline_missed = False
    def process_frame(self, msg):
        start_time = self.get_clock().now()  # Start timing
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # Perform YOLO object detection
        results = self.model(frame, stream=True)
        for r in results:
            boxes = r.boxes
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                confidence = round(float(box.conf[0]) * 100) / 100
                cls = int(box.cls[0])
                class_name = self.class_names[cls]

                # Draw bounding box and label on the frame
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 255), 3)
                cv2.putText(frame, f'{class_name} ({confidence})', (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                # Publish bounding box marker
                self.publish_marker(class_name, x1, y1, x2, y2, confidence)

                # Log warning if a "person" is detected
                if class_name == "person":
                    red_bold_text = f"\033[1;31m{'!'*40}\n⚠️  WARNING: PERSON DETECTED\nConfidence: {confidence}\n{'!'*40}\033[0m"
                    self.get_logger().info(red_bold_text)

        # Display the frame with detections
        cv2.imshow('Detection Output', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to quit
            self.destroy_node()
            cv2.destroyAllWindows()
            rclpy.shutdown()

        # Check processing time
        elapsed_time_ms = (self.get_clock().now() - start_time).nanoseconds / 1e6
        if elapsed_time_ms > self.deadline_ms:
            self.get_logger().error(
                f"⚠️ EXCEEDED DEADLINE: Frame processed in {elapsed_time_ms:.2f} ms (Deadline: {self.deadline_ms} ms)"
            )
            self.deadline_missed = True
        else:
            self.get_logger().info(f"✅ Frame processed in {elapsed_time_ms:.2f} ms")

        # Adjust frame rate and deadline for demonstration purposes
        self.adjust_frame_rate_and_deadline(self.deadline_missed)

    def adjust_frame_rate_and_deadline(self, deadline_missed):
        if deadline_missed:
            # If deadline missed, reduce frame rate and extend the deadline
            if self.current_frame_rate > 15:  # Prevent reducing too low
                self.current_frame_rate -= 15  # Decrease frame rate
                self.deadline_ms += 50  # Extend deadline
                self.get_logger().warning(
                    f"⚠️ Deadline missed! Adjusting frame rate to {self.current_frame_rate} FPS and deadline to {self.deadline_ms} ms"
                )
            else:
                self.get_logger().warning("⚠️ Frame rate and deadline already at lowest safe levels.")
        else:
            # If no deadline misses, maintain or restore normal frame rate
            if self.current_frame_rate < 30:  # Prevent increasing beyond normal
                self.current_frame_rate += 15  # Restore frame rate
                self.deadline_ms -= 50  # Restore deadline
                self.get_logger().info(
                    f"✅ Deadline met! Restoring frame rate to {self.current_frame_rate} FPS and deadline to {self.deadline_ms} ms"
                )
            else:
                self.get_logger().info("🟢 System operating at optimal frame rate and deadline.")

    def publish_marker(self, class_name, x1, y1, x2, y2, confidence):
        marker = Marker()
        marker.header.frame_id = "camera_frame"
        marker.type = Marker.TEXT_VIEW_FACING
        marker.text = f"{class_name} ({confidence})"
        marker.pose.position.x = (x1 + x2) / 2
        marker.pose.position.y = (y1 + y2) / 2
        marker.pose.position.z = 0.0
        marker.scale.z = 0.1  # Font size
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        self.marker_publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
