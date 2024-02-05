
#include "../include/StereoCamera.hpp"

StereoCamera::StereoCamera()
{
    is_opened = false;
    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    cfg.enable_stream(RS2_STREAM_INFRARED, 1);
    cfg.enable_stream(RS2_STREAM_INFRARED, 2);
    pipe.start(cfg);

    camThread = new std::thread(&StereoCamera::updateFrame, this);
}

StereoCamera::~StereoCamera()
{
    pipe.stop();
}

StereoFrame StereoCamera::getLeftFrame()
{
    long long timestamp = chrono::time_point_cast<chrono::nanoseconds>(chrono::system_clock::now()).time_since_epoch().count();
    StereoFrame frame{leftFrame, timestamp};

    if (leftFrame.empty())
        return StereoFrame{cv::Mat(), -1};

    return frame;
}

StereoFrame StereoCamera::getRightFrame()
{
    long long timestamp = chrono::time_point_cast<chrono::nanoseconds>(chrono::system_clock::now()).time_since_epoch().count();
    StereoFrame frame{rightFrame, timestamp};

    if (rightFrame.empty())
        return StereoFrame{cv::Mat(), -1};

    return frame;
}

void StereoCamera::updateFrame()
{
    while (1)
    {
        // Wait for the next set of frames from the camera
        rs2::frameset frames = pipe.wait_for_frames();

        // Get each frame
        rs2::frame left = frames.get_infrared_frame(1);
        rs2::frame right = frames.get_infrared_frame(2);

        is_opened = pipe.poll_for_frames(&frames);

        const int l_width = left.as<rs2::video_frame>().get_width();
        const int l_height = left.as<rs2::video_frame>().get_height();
        const int r_width = right.as<rs2::video_frame>().get_width();
        const int r_height = right.as<rs2::video_frame>().get_height();

        // Convert the frames to OpenCV Mat
        leftFrame = cv::Mat(cv::Size(l_width, l_height), CV_8UC1, (void *)left.get_data(), cv::Mat::AUTO_STEP);
        rightFrame = cv::Mat(cv::Size(r_width, r_height), CV_8UC1, (void *)right.get_data(), cv::Mat::AUTO_STEP);
    }
}

bool StereoCamera::isOpened() const
{
    return is_opened;
}