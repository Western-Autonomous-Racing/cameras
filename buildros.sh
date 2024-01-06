cd ThirdParty/MPU6050-Jetson-Orin
make uninstall
make install
make example

cd ~/cameraimu_ws
rm -rf build
rm -rf install
rm -rf log

cd ~/cameraimu_ws/src
rm -rf camera-imu
rm -rf build
rm -rf lib
ros2 pkg create camera-imu --dependencies rclcpp sensor_msgs std_msgs cv_bridge builtin_interfaces --build-type ament_cmake

cd ~/cameraimu_ws
colcon build --packages-select camera-imu