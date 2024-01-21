#include "CameraIMU_node.hpp"
#include <builtin_interfaces/msg/time.hpp>

CameraImuNode::CameraImuNode() : 
Node("camera_imu_node"),  
camera(true, MODE_2, true),
imu()
{
    // Initialize camera
    // Initialize publishers
    imagePublisher = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 100);
    imuPublisher = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 1000);

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
    while (1)
    {
        // Get image
        image = camera.getFrame();

        // Publish image
        cvImage = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image.frame);
        cvImage.header.stamp = rclcpp::Time(image.timestamp);
        imagePublisher->publish(*cvImage.toImageMsg());
    }
}

void CameraImuNode::ImuThreadFunc()
{
    float ax, ay, az, gr, gp, gy, temp;
    long long imu_ts;
    while (1)
    {
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
    }
}