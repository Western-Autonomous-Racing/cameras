#include "CameraIMU_node.hpp"
#include <builtin_interfaces/msg/time.hpp>

CameraImuNode::CameraImuNode(const rclcpp::NodeOptions & options) : 
Node("camera_imu_node", options),
enable_imu(this->declare_parameter<bool>("enable_imu", true)),
enable_camera(this->declare_parameter<bool>("enable_imu", true))
{
    // Initialize camera
    // Initialize publishers
    if (enable_camera) {
        camera = Camera(true, MODE_2, true);
        imagePublisher = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 100);
    }

    if (enable_imu) {
        imu = MPU6050();
        imuPublisher = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 1000);
    }

    // Start thread
    mThread = new thread(&CameraImuNode::SyncandPublish, this);
}

CameraImuNode::CameraImuNode() : CameraImuNode(rclcpp::NodeOptions()) {}

CameraImuNode::~CameraImuNode()
{
    mThread->join();
    delete mThread;
}

void CameraImuNode::SyncandPublish()
{
    while (rclcpp::ok())
    {
        float ax, ay, az, gr, gp, gy, temp;
        long long imu_ts;
        Frame image;
        // Get image
        mMutex.lock();

        if (enable_camera)
            Frame image = camera.getFrame();
        // Get IMU data
        if (enable_imu)
            imu.getIMU(&ax, &ay, &az, &gr, &gp, &gy, &temp, &imu_ts);

        mMutex.unlock();

        // Publish image
        if (enable_camera) {
            cv_bridge::CvImage cvImage = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image.frame);
            cvImage.header.stamp = rclcpp::Time(image.timestamp);
            imagePublisher->publish(*cvImage.toImageMsg());
        }
        
        // Publish IMU data
        if (enable_imu) {
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

}