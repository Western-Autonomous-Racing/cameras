// Author : Daniel Xie
// Class for streaming video from a camera or file with Gstreamer as a multi threaded process
// functions:
//  - check if pipeline is up
//  - get frame from pipeline
//  - preview frame from pipeline
//  - set grayscale or RGB
//  - set camera mode

#ifndef __CAMERA_HPP__
#define __CAMERA_HPP__

#define GRAYSCALE "NV12"
#define RGB "BGRx"

#include <opencv2/core/core.hpp>
#include <opencv2/videoio.hpp>

using namespace std;

enum CameraMode {
    MODE_0,
    MODE_1,
    MODE_2,
    MODE_3,
    MODE_4
};

struct CameraConfig {
    string width;
    string height;
    string framerate;
};

std::map<CameraMode, CameraConfig> cameraConfigs = {
    {MODE_0, {"3280", "2464", "21"}},
    {MODE_1, {"3280", "1848", "28"}},
    {MODE_2, {"1920", "1080", "30"}},
    {MODE_3, {"1640", "1232", "30"}},
    {MODE_4, {"1280", "720", "60"}}
};


class Camera : public cv::VideoCapture
{
    // pipeline: "nvarguscamerasrc sensor-mode=2 ! video/x-raw(memory:NVMM),width=1920,height=1080,format=(string)NV12,framerate=(fraction)30/1 ! "
    //           "nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! videoflip method=rotate-180  ! appsink"

    public:
        Camera(bool isColor, CameraMode mode, bool vflip);
        ~Camera();

        bool isOpened() const;
        cv::Mat getFrame();
        void previewFrame() const;

        void setPipeline();

    private:
        cv::Mat frame;
        bool isColor;
        CameraMode mode;
        cv::VideoCapture cap;
        std::string pipeline;
        bool vflip;
};

#endif