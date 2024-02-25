#ifndef __STEREOCAMERA_HPP__
#define __STEREOCAMERA_HPP__

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <thread>
#include <csignal>
#include <rclcpp/rclcpp.hpp>

using namespace std;

struct StereoFrame {
    cv::Mat frame;
    rclcpp::Time timestamp;
};

class StereoCamera
{
    public:
        StereoCamera();
        ~StereoCamera();

        // StereoFrame getLeftFrame();
        // StereoFrame getRightFrame();
        // void updateFrame();
        void getFrames(StereoFrame *leftFrame, StereoFrame *rightFrame);
        bool isOpened() const;
        void signal_interrupted(int signal);

    private:
        rs2::config cfg;
        rs2::pipeline pipe;
        rs2::pipeline_profile pipeline_profile;
        rs2::device dev;
        rs2::sensor depth_sensor;

        cv::Mat leftImage;
        cv::Mat rightImage;
        thread *camThread;
        bool is_opened;
};

#endif