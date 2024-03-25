#include "../include/CameraIMU_node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc != 2) {
        cerr << "Usage: ros2 run camera-imu CameraIMUNode path_to_config_file" << endl;
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::spin(std::make_shared<CameraImuNode>(argv[1]));
    rclcpp::shutdown();
    return 0;
}