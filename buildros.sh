cd ~/cameraimu_ws/src
ros2 pkg create camera_imu --dependencies rclcpp sensor_msgs sensor_msgs cv_bridge builtin_interfaces --build-type ament_cmake

cd ..
colcon build --packages-select camera_imu