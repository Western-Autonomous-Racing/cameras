#include "../include/StereoCamera.hpp"
#include <rclcpp/rclcpp.hpp>


StereoCamera::StereoCamera()
{
    is_opened = false;
    // Create a Pipeline - this serves as a top-level API for streaming and processing frames

    // 640x480 30fps locked because depth module 
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_INFRARED, 2, 640, 480, RS2_FORMAT_Y8, 30);
    
    pipeline_profile = pipe.start(cfg); 
    dev = pipeline_profile.get_device();
    depth_sensor = dev.query_sensors()[0];
    
    if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
    {
        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f); // Enable emitter
    }

    // Start the thread
    camThread = new std::thread(&StereoCamera::updateFrame, this);
}

StereoCamera::~StereoCamera()
{
    is_opened = false;
    try {
        pipe.stop();
    } catch (const rs2::error & e) {
        std::cerr << "Failed to stop the pipeline: " << e.what() << std::endl;
    }
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
    rs2::frameset frames;
    rs2::frame left;
    rs2::frame right;

    is_opened = true;

    try {
        while (rclcpp::ok())
        {
            // Wait for the next set of frames from the camera
            frames = pipe.wait_for_frames();
            
            // Get each frame
            left = frames.get_infrared_frame(1);
            right = frames.get_infrared_frame(2);

            const int l_width = left.as<rs2::video_frame>().get_width();
            const int l_height = left.as<rs2::video_frame>().get_height();
            const int r_width = right.as<rs2::video_frame>().get_width();
            const int r_height = right.as<rs2::video_frame>().get_height();

            // Convert the frames to OpenCV Mat
            cv::Mat leftConv = cv::Mat(cv::Size(l_width, l_height), CV_8UC1, (void *)left.get_data(), cv::Mat::AUTO_STEP);
            cv::flip(leftConv, leftFrame, -1);
            
            cv::Mat rightConv = cv::Mat(cv::Size(r_width, r_height), CV_8UC1, (void *)right.get_data(), cv::Mat::AUTO_STEP);
            cv::flip(rightConv, rightFrame, -1);        

            if (!leftFrame.empty() && !rightFrame.empty())
            {
                is_opened = true;
            }
        }
    } catch (const std::exception& e) {
        // Handle the exception (e.g., log the error, clean up resources, etc.)
        std::cerr << "Exception occurred in updateFrame: " << e.what() << std::endl;
    }
}

bool StereoCamera::isOpened() const
{
    return is_opened;
}