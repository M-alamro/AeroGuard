# Computer Vision

Drone detection and tracking module for the AeroGuard autonomous drone system.

## Components

- YOLO-based drone detection
- Multi-object tracking
- Track identity management
- ROS 2 camera image subscription
- Gazebo camera integration through ROS-Gazebo bridges

## Requirements

- Ubuntu with a compatible ROS 2 installation
- Gazebo and ros_gz_bridge
- Python dependencies: OpenCV, NumPy, Requests, Ultralytics
- ROS 2 Python packages: rclpy, sensor_msgs, cv_bridge
- Compatible NVIDIA GPU and CUDA setup recommended for GPU inference

## Model

The default model path is weights/best.pt.
Ensure the model weights are present before running.

## Run

From the computer-vision directory, with ROS 2 and Gazebo bridges configured:

    python3 -m final_project.main_ros

The application expects the configured Gazebo camera topic and starts ROS-Gazebo bridges.

## Notes

- Dataset files and training outputs are not included by default.
- Verify ROS 2, Gazebo, camera topic, and GPU compatibility before running.
