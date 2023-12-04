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

    // Start thread
    mThread = new thread(&CameraImuNode::SyncandPublish, this);
}

CameraImuNode::~CameraImuNode()
{
    mThread->join();
    delete mThread;
}

void CameraImuNode::SyncandPublish()
{
    while (rclcpp::ok())
    {
        float ax, ay, az, gr, gp, gy;
        long long imu_ts;
        // Get image
        mMutex.lock();

        Frame image = camera.getFrame();
        // Get IMU data
        imu.getIMU(&ax, &ay, &az, &gr, &gp, &gy, &imu_ts);

        mMutex.unlock();

        // Publish image
        cv_bridge::CvImage cvImage = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image.frame);
        cvImage.header.stamp = rclcpp::Time(image.timestamp);
        
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

        imagePublisher->publish(*cvImage.toImageMsg());
        imuPublisher->publish(imuMsg);
    }    

}