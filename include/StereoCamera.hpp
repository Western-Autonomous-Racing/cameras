#ifndef __STEREOCAMERA_HPP__
#define __STEREOCAMERA_HPP__

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <thread>
#include <csignal>

using namespace std;

struct StereoFrame {
    cv::Mat frame;
    long long timestamp;
};

class StereoCamera
{
    public:
        StereoCamera();
        ~StereoCamera();

        StereoFrame getLeftFrame();
        StereoFrame getRightFrame();
        void updateFrame();
        bool isOpened() const;
        void signal_interrupted(int signal);

    private:
        rs2::config cfg;
        rs2::pipeline pipe;
        rs2::pipeline_profile pipeline_profile;
        rs2::device dev;
        rs2::sensor depth_sensor;

        cv::Mat leftFrame;
        cv::Mat rightFrame;
        thread *camThread;
        bool is_opened;
};

#endif