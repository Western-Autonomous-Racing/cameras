// Author : Daniel Xie

#include "../include/camera.hpp"
#include <opencv2/opencv.hpp>

Camera::Camera(bool isColor = true, CameraMode mode = MODE_2, bool vflip = true) : isColor(isColor), mode(mode), vflip(vflip), frame(cv::Mat())
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
    cv::destroyAllWindows();
}


void Camera::setPipeline()
{
    this->isColor = isColor;
    this->mode = mode;

    pipeline = "nvarguscamerasrc sensor-mode=" + to_string(mode) + " ! video/x-raw(memory:NVMM),width=" + 
                cameraConfigs[mode].width + ",height=" + cameraConfigs[mode].height + ",format=(string)NV12,framerate=" + 
                cameraConfigs[mode].framerate + "/1 ! nvvidconv" + (isColor ? "!  video/x-raw, format=(string)BGRx " : " ") + 
                "! videoconvert" + (vflip ? " ! videoflip method=rotate-180" : " ") + "! appsink";
}

bool Camera::isOpened() const
{
    return cap.isOpened();
}

cv::Mat Camera::getFrame()
{
    cap >> frame;
    
    if (frame.empty())
        return cv::Mat();
    
    return frame;
}

void Camera::previewFrame() const
{
    cv::imshow("Preview", frame);
    cv::waitKey(1);
}