#ifndef __CAMERA_IMU_NODE_HPP__
#define __CAMERA_IMU_NODE_HPP__

#include "Camera.hpp"
#include <MPU6050.h>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <thread>
#include <mutex>

using namespace std;

class CameraImuNode : public rclcpp::Node
{
public:
    CameraImuNode();
    ~CameraImuNode();
    void SyncandPublish();
    void CameraThreadFunc();
    void ImuThreadFunc();

    thread *cameraThread;
    thread *imuThread;

private:
    Camera camera;
    MPU6050 imu;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imagePublisher;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPublisher;
};

#endif