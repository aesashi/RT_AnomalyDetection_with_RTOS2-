# **Real-Time Human Detection with ROS 2 and YOLOv8**

## **Tech Stack**
- **Programming Language**: Python
- **Framework**: ROS 2 (Robot Operating System 2)
- **Libraries**:
  - **YOLOv8**: Ultralytics YOLOv8 for object detection.
  - **OpenCV**: For processing and displaying live video frames.
  - **rclpy**: Python client library for ROS 2 to manage nodes, topics, and messages.
  - **Visualization_msgs/Marker**: For visualizing detected objects in RViz.
  - **cv_bridge**: To convert images between ROS messages and OpenCV.

---

## **Story Line**
This project showcases a **real-time human detection system** using a webcam and YOLOv8 object detection:
- A live camera feed is processed in real time.
- When a **person** is detected, the system:
  - Displays bounding boxes and labels on the video feed.
  - Logs a **bold and red alert message** in the terminal, warning the user of the detection.

---
## **Demo Video**
[Click here to watch the demo video](https://drive.google.com/file/d/1vuJ7HEpXFULmdWSzAygJwXPG-kVcWIYq/view?usp=sharing)

## **Demonstration Scenario for Real Time**
This demonstrates that when deadline miss occur, the system will automatically adjust frame rates to meet the deadline to maintain the system.
### **1. Normal Operation**
- **Initial Frame Rate**: 30 FPS.
- **Initial Deadline**: 100ms per frame.
- The system processes frames in real-time and raises alerts for "person" detection.

### **2. Deadline Miss Handling**
- If the frame processing exceeds the 100ms deadline:
  - **Frame Rate**: Reduced to 15 FPS.
  - **Deadline**: Increased to 150ms to stabilize the system.

### **3. Scenario Walkthrough**
- **First 10 seconds**: The system operates at 30 FPS with a 100ms deadline.
- **After 10 seconds**:
  - Increases workload (e.g., 60 FPS frame rate).
  - Detects deadline misses due to increased workload.
- **Deadline Miss**:
  - Reduces frame rate to 15 FPS.
  - Adjusts the deadline to 150ms.
- **System Recovery**:
  - Gradually restores frame rate to 30 FPS and deadline to 100ms once deadlines are consistently met.

---

## **How to Run**
Set up ROS2 first then follow below command.

1. **Run the Camera Node**:
   ```bash
   ros2 run yolo_detection camera_node
1. **Run the Detection Node**:
   ```bash
   ros2 run yolo_detection detection_node