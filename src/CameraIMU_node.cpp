#include "CameraIMU_node.hpp"
#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/qos.hpp>
#include <chrono>

CameraImuNode::CameraImuNode() : Node("camera_imu_node"),
                                 rgb_camera(true, MODE_2, true),
                                 stereo_camera(declare_and_get_parameter("Enable_Auto", 1), declare_and_get_parameter("Manual_Exposure", 1000)),
                                 imu()
{
    rclcpp::QoS qos_cam(rclcpp::KeepLast(100));
    rclcpp::QoS qos_imu(rclcpp::KeepLast(1000));

    // Initialize publishers
    rgbImagePublisher = this->create_publisher<sensor_msgs::msg::Image>("/rgb_camera/color/image_raw", qos_cam);
    leftStereoPublisher = this->create_publisher<sensor_msgs::msg::Image>("/stereo_camera/left/image_raw", qos_cam);
    rightStereoPublisher = this->create_publisher<sensor_msgs::msg::Image>("/stereo_camera/right/image_raw", qos_cam);
    depthStereoPublisher = this->create_publisher<sensor_msgs::msg::Image>("/stereo_camera/depth/image_raw", qos_cam);
    imuPublisher = this->create_publisher<sensor_msgs::msg::Imu>("/imu", qos_imu);

    // Initialize threads
    rgbCameraThread = new thread(&CameraImuNode::RGBCameraThreadFunc, this);
    stereoCameraThread = new thread(&CameraImuNode::StereoCameraThreadFunc, this);
    imuThread = new thread(&CameraImuNode::runImuThread, this);
}

CameraImuNode::~CameraImuNode()
{
    rgbCameraThread->join();
    stereoCameraThread->join();
    imuThread->join();

    delete rgbCameraThread;
    delete stereoCameraThread;
    delete imuThread;
}

void CameraImuNode::runImuThread()
{
    imuTimer = this->create_wall_timer(std::chrono::milliseconds(5), std::bind(&CameraImuNode::ImuThreadFunc, this));
}

void CameraImuNode::RGBCameraThreadFunc()
{
    RGBFrame image;
    cv_bridge::CvImage cvImage;
    while (rclcpp::ok())
    {
        // Get image
        image = rgb_camera.getFrame();

        cvImage = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image.frame);
        cvImage.header.stamp = rclcpp::Time(image.timestamp);
        rgbImagePublisher->publish(*cvImage.toImageMsg());
    }
}

void CameraImuNode::StereoCameraThreadFunc()
{
    StereoFrame leftImage, rightImage, depthImage;
    cv_bridge::CvImage leftCvImage, rightCvImage, depthCvImage;
    while (rclcpp::ok())
    {
        auto start = std::chrono::high_resolution_clock::now(); // Start the timer

        // Get images
        stereo_camera.getFrames(&leftImage, &rightImage, &depthImage);

        leftCvImage = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", leftImage.frame);
        leftCvImage.header.stamp = rclcpp::Time(leftImage.timestamp);
        leftStereoPublisher->publish(*leftCvImage.toImageMsg());

        rightCvImage = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", rightImage.frame);
        rightCvImage.header.stamp = rclcpp::Time(rightImage.timestamp);
        rightStereoPublisher->publish(*rightCvImage.toImageMsg());

        depthCvImage = cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", depthImage.frame);
        depthCvImage.header.stamp = rclcpp::Time(depthImage.timestamp);
        depthStereoPublisher->publish(*depthCvImage.toImageMsg());

        auto end = std::chrono::high_resolution_clock::now();                               // Stop the timer
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start); // Calculate the duration in microseconds

        // Delay to achieve 30 fps
        std::chrono::microseconds targetDelay(1000000 / 30); // 30 fps
        std::this_thread::sleep_for(targetDelay - duration);
    }
}

void CameraImuNode::ImuThreadFunc()
{
    float ax, ay, az, gr, gp, gy, temp;
    long long imu_ts;

    imu.getIMU(&ax, &ay, &az, &gr, &gp, &gy, &temp, &imu_ts);

    // Publish IMU data
    sensor_msgs::msg::Imu imuMsg = sensor_msgs::msg::Imu();
    imuMsg.header.stamp = rclcpp::Time(imu_ts);

    imuMsg.linear_acceleration.x = ax;
    imuMsg.linear_acceleration.y = ay;
    imuMsg.linear_acceleration.z = az;

    imuMsg.angular_velocity.x = gr;
    imuMsg.angular_velocity.y = gp;
    imuMsg.angular_velocity.z = gy;

    // fill in with blank data
    imuMsg.orientation.x = 0.0;
    imuMsg.orientation.y = 0.0;
    imuMsg.orientation.z = 0.0;
    imuMsg.orientation.w = 1.0;

    imuMsg.angular_velocity_covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    imuMsg.linear_acceleration_covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    imuMsg.orientation_covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    imuPublisher->publish(imuMsg);
}
