#ifndef __STEREOCAMERA_HPP__
#define __STEREOCAMERA_HPP__

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <thread>

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

    private:
        rs2::config cfg;
        rs2::pipeline pipe;
        cv::Mat leftFrame;
        cv::Mat rightFrame;
        thread *camThread;
        bool is_opened;
};

#endif