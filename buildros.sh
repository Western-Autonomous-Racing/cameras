cd ~/cameraimu_ws/src
ros2 pkg create camera-imu --dependencies rclcpp sensor_msgs std_msgs cv_bridge builtin_interfaces --build-type ament_cmake

cd ~/cameraimu_ws
colcon build --packages-select camera-imu