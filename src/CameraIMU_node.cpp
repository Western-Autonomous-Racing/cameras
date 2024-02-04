#include <chrono>
#include "CameraIMU_node.hpp"
#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/qos.hpp>

CameraImuNode::CameraImuNode() : 
Node("camera_imu_node"),  
camera(true, MODE_2, true),
imu()
{
    // Initialize camera
    // Initialize publishers
    rclcpp::QoS qos_cam(rclcpp::KeepLast(100));
    rclcpp::QoS qos_imu(rclcpp::KeepLast(1000));
    imagePublisher = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", qos_cam);
    imuPublisher = this->create_publisher<sensor_msgs::msg::Imu>("/imu", qos_imu);

    // Start threads
    cameraThread = new thread(&CameraImuNode::CameraThreadFunc, this);
    imuThread = new thread(&CameraImuNode::ImuThreadFunc, this);
}

CameraImuNode::~CameraImuNode()
{
    // Join threads
    cameraThread->join();
    imuThread->join();

    // Delete threads
    delete cameraThread;
    delete imuThread;
}

void CameraImuNode::CameraThreadFunc()
{
    Frame image;
    cv_bridge::CvImage cvImage;
    auto t_start = std::chrono::high_resolution_clock::now();
    int frames = 0;
    while (rclcpp::ok())
    {
        frames++;
        auto start = std::chrono::high_resolution_clock::now(); // Start the timer

        // Get image
        image = camera.getFrame();
        // cout << "Image timestamp: " << image.timestamp << endl;

        cvImage = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image.frame);
        cvImage.header.stamp = rclcpp::Time(image.timestamp);
        imagePublisher->publish(*cvImage.toImageMsg());

        auto end = std::chrono::high_resolution_clock::now(); // Stop the timer
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start); // Calculate the duration in microseconds
        // std::cout << "CameraThreadFunc duration: " << duration.count() << " microseconds" << std::endl;

    }
    // auto t_end = std::chrono::high_resolution_clock::now();
    // auto t_duration = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start);
    // std::cout << "Camera Frames per second: " << frames / (t_duration.count() / 1000000.0) << std::endl;
}

void CameraImuNode::ImuThreadFunc()
{
    float ax, ay, az, gr, gp, gy, temp;
    long long imu_ts;
    auto t_start = std::chrono::high_resolution_clock::now();
    int frames = 0;

    while (rclcpp::ok())
    {
        frames++;
        auto start = std::chrono::high_resolution_clock::now(); // Start the timer

        // Get IMU data
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

        imuMsg.orientation.x = 0.0;
        imuMsg.orientation.y = 0.0;
        imuMsg.orientation.z = 0.0;
        imuMsg.orientation.w = 1.0;

        imuMsg.angular_velocity_covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        imuMsg.linear_acceleration_covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        imuMsg.orientation_covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        imuPublisher->publish(imuMsg);

        // auto end = std::chrono::high_resolution_clock::now(); // Stop the timer
        // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start); // Calculate the duration in microseconds
        // std::cout << "IMU Msg linear acceleration: " << imuMsg.linear_acceleration.x << ", " << imuMsg.linear_acceleration.y << ", " << imuMsg.linear_acceleration.z << std::endl;
        // std::cout << "IMU Msg angular velocity: " << imuMsg.angular_velocity.x << ", " << imuMsg.angular_velocity.y << ", " << imuMsg.angular_velocity.z << std::endl;
        // std::cout << "ImuThreadFunc duration: " << duration.count() << " microseconds" << std::endl;
    }

    // auto t_end = std::chrono::high_resolution_clock::now();
    // auto t_duration = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start);
    // std::cout << "IMU Frames per second: " << frames / (t_duration.count() / 1000000.0) << std::endl;
}
