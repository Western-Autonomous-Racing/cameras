#include "../include/StereoCamera.hpp"

StereoCamera::StereoCamera(int enable_auto, int manual_exp)
{

    is_opened = false;
    // Create a Pipeline - this serves as a top-level API for streaming and processing frames

    // 640x480 30fps locked because depth module
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_INFRARED, 2, 640, 480, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, -1, 640, 480, RS2_FORMAT_Z16, 30);

    pipeline_profile = pipe.start(cfg);
    dev = pipeline_profile.get_device();
    depth_sensor = dev.query_sensors()[0];

    if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
    {
        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f); // Enable emitter
    }
    
    if (enable_auto > 1 || enable_auto < 0 || manual_exp < 0)
    {
      cerr << "ERROR: Bad camera settings!" << endl;
      return;
    }

    if (depth_sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE))
    {
        if (enable_auto)
        {
            depth_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1); // Enable auto-exposure
        }
        else
        {
            depth_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0); // Enable auto-exposure
            if (depth_sensor.supports(RS2_OPTION_EXPOSURE))
            {
                depth_sensor.set_option(RS2_OPTION_EXPOSURE, manual_exp); // Set exposure
            }
        }
    }

    cout << "Stereo Camera is opened" << endl;
}

StereoCamera::~StereoCamera()
{
    is_opened = false;

    try
    {
        pipe.stop();
    }
    catch (const rs2::error &e)
    {
        std::cerr << "Failed to stop the pipeline: " << e.what() << std::endl;
    }
}

void StereoCamera::getFrames(StereoFrame *leftFrame, StereoFrame *rightFrame, StereoFrame *depthFrame)
{
    rs2::frameset frames;
    rs2::frame left;
    rs2::frame right;

    is_opened = true;

    // Wait for the next set of frames from the camera
    frames = pipe.wait_for_frames();

    // Get each frame
    right = frames.get_infrared_frame(1);
    left = frames.get_infrared_frame(2);
    rs2::depth_frame depth = frames.get_depth_frame();

    const int l_width = left.as<rs2::video_frame>().get_width();
    const int l_height = left.as<rs2::video_frame>().get_height();
    const int r_width = right.as<rs2::video_frame>().get_width();
    const int r_height = right.as<rs2::video_frame>().get_height();
    const int d_width = depth.as<rs2::video_frame>().get_width();
    const int d_height = depth.as<rs2::video_frame>().get_height();

    // Convert the frames to OpenCV Mat
    cv::Mat leftImage = cv::Mat(cv::Size(l_width, l_height), CV_8UC1, (void *)left.get_data(), cv::Mat::AUTO_STEP);
    cv::Mat rightImage = cv::Mat(cv::Size(r_width, r_height), CV_8UC1, (void *)right.get_data(), cv::Mat::AUTO_STEP);
    cv::Mat depthImage = cv::Mat(cv::Size(d_width, d_height), CV_16UC1, (void *)depth.get_data(), cv::Mat::AUTO_STEP);

    if (!leftImage.empty() && !rightImage.empty())
    {
        is_opened = true;
    }

    rclcpp::Time timestamp = rclcpp::Clock().now();

    if (leftImage.empty() || rightImage.empty() || depthImage.empty())
    {
        *leftFrame = StereoFrame{cv::Mat(), rclcpp::Time()};
        *rightFrame = StereoFrame{cv::Mat(), rclcpp::Time()};
        *depthFrame = StereoFrame{cv::Mat(), rclcpp::Time()};
        return; // Return an empty rclcpp::Time object instead of nullptr
    }

    *leftFrame = StereoFrame{leftImage, timestamp};
    *rightFrame = StereoFrame{rightImage, timestamp};
    *depthFrame = StereoFrame{depthImage, timestamp};
}

bool StereoCamera::isOpened() const
{
    return is_opened;
}