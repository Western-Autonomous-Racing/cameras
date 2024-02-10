#ifndef __CAMERA_IMU_NODE_HPP__
#define __CAMERA_IMU_NODE_HPP__

#include "RGBCamera.hpp"
#include "StereoCamera.hpp"
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
    void RGBCameraThreadFunc();
    void StereoCameraThreadFunc();
    void ImuThreadFunc();

    thread *rgbCameraThread;
    thread *stereoCameraThread;
    thread *imuThread;

private:
    RGBCamera rgb_camera;
    StereoCamera stereo_camera;
    MPU6050 imu;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgbImagePublisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr leftStereoPublisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rightStereoPublisher;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPublisher;
};

#endif