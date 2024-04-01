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
    void RGBCameraThreadFunc();
    void StereoCameraThreadFunc();
    void ImuThreadFunc();

    void runImuThread();
    void runRGBThread();
    void runStereoThread();

    thread *rgbCameraThread;
    thread *stereoCameraThread;
    thread *imuThread;

    rclcpp::TimerBase::SharedPtr rgbCameraTimer;
    rclcpp::TimerBase::SharedPtr stereoCameraTimer;
    rclcpp::TimerBase::SharedPtr imuTimer;

private:
    RGBCamera rgb_camera;
    StereoCamera stereo_camera;
    MPU6050 imu;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgbImagePublisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr leftStereoPublisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rightStereoPublisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depthStereoPublisher;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPublisher;
    int declare_and_get_parameter(const std::string& name, int default_value)
    {
        this->declare_parameter(name, default_value);
        return this->get_parameter(name).as_int();
    }
};

#endif