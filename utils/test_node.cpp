#include "../include/camera_imu_node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraImuNode>());
    rclcpp::shutdown();
    return 0;
}