// Author : Daniel Xie

#include "../include/camera.hpp"
#include <opencv2/opencv.hpp>
#include <chrono>

std::map<CameraMode, CameraConfig> cameraConfigs = {
    {MODE_0, {0, "3280", "2464", "21"}},
    {MODE_1, {1, "3280", "1848", "28"}},
    {MODE_2, {2, "1920", "1080", "30"}},
    {MODE_3, {3, "1640", "1232", "30"}},
    {MODE_4, {4, "1280", "720", "60"}}
};

Camera::Camera(bool isColor = true, CameraMode mode = MODE_2, bool vflip = true) : isColor(isColor), mode(mode), vflip(vflip)
{
    setPipeline();

    cap = cv::VideoCapture(pipeline, cv::CAP_GSTREAMER);

    if (!cap.isOpened())
    {
        std::cout << "Camera failed to open" << std::endl;
    }
    else
    {
        std::cout << "Camera opened successfully" << std::endl;
    }
}

Camera::~Camera()
{
    cap.release();
}


void Camera::setPipeline()
{
    this->isColor = isColor;
    this->mode = mode;

    pipeline = "nvarguscamerasrc sensor-mode=" + to_string(cameraConfigs[mode].mode) + " ! video/x-raw(memory:NVMM),width=" + 
                cameraConfigs[mode].width + ",height=" + cameraConfigs[mode].height + ",format=(string)NV12,framerate=(fraction)" + 
                cameraConfigs[mode].framerate + "/1 ! nvvidconv" + (isColor ? " ! video/x-raw, format=(string)BGRx " : " ") + 
                "! videoconvert" + (vflip ? " ! videoflip method=rotate-180 " : " ") + "! appsink";

    cout << "Pipeline: " << pipeline << endl;
}

bool Camera::isOpened() const
{
    return cap.isOpened();
}

Image Camera::getFrame()
{
    cv::Mat frame;
    cap >> frame;
    long long timestamp = chrono::time_point_cast<chrono::nanoseconds>(chrono::system_clock::now()).time_since_epoch().count();
    Image image{frame, timestamp};

    if (frame.empty())
        return Image{cv::Mat(), -1};

    // cv::medianBlur(image, image, 5);

    // You can use the timestamp as needed

    return Image{frame, timestamp};
    }
