// Author : Daniel Xie

#include "../include/RGBCamera.hpp"
#include <opencv2/opencv.hpp>
#include <chrono>

std::map<CameraMode, CameraConfig> cameraConfigs = {
    {MODE_0, {0, "3280", "2464", "21"}},
    {MODE_1, {1, "3280", "1848", "28"}},
    {MODE_2, {2, "1920", "1080", "30"}},
    {MODE_3, {3, "1640", "1232", "30"}},
    {MODE_4, {4, "1280", "720", "60"}}
};

RGBCamera::RGBCamera(bool isColor, CameraMode mode, bool vflip) : isColor(isColor), mode(mode), vflip(vflip)
{
    setPipeline();

    cap = cv::VideoCapture(pipeline, cv::CAP_GSTREAMER);

    if (!cap.isOpened())
    {
        std::cout << "RGBCamera failed to open" << std::endl;
    }
    else
    {
        std::cout << "RGBCamera opened successfully" << std::endl;
    }
}

RGBCamera::RGBCamera() : RGBCamera(true, MODE_2, true) {}

RGBCamera::~RGBCamera()
{
    cap.release();
}

void RGBCamera::setPipeline()
{
    this->isColor = isColor;
    this->mode = mode;

    pipeline = "nvarguscamerasrc sensor-mode=" + to_string(cameraConfigs[mode].mode) + " ! video/x-raw(memory:NVMM),width=" + 
                cameraConfigs[mode].width + ",height=" + cameraConfigs[mode].height + ",format=(string)NV12,framerate=(fraction)" + 
                cameraConfigs[mode].framerate + "/1 ! nvvidconv" + (isColor ? " ! video/x-raw, format=(string)BGRx " : " ") + 
                "! videoconvert" + (vflip ? " ! videoflip method=rotate-180 " : " ") + "! appsink";

    cout << "Pipeline: " << pipeline << endl;
}

bool RGBCamera::isOpened() const
{
    return cap.isOpened();
}

RGBFrame RGBCamera::getFrame()
{
    cv::Mat image;
    cap >> image;
    rclcpp::Time timestamp = rclcpp::Clock().now();
    RGBFrame frame{image, timestamp};

    if (image.empty())
        return RGBFrame{cv::Mat(), rclcpp::Time()};

    // cv::medianBlur(image, image, 5);

    // You can use the timestamp as needed
    // cout << "Image timestamp: " << timestamp << endl;

    return frame;
}
