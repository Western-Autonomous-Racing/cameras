import rclpy
from rclpy.node import Node
import board
import adafruit_mpu6050
from sensor_msgs.msg import Imu

class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.publisher = self.create_publisher(Imu, '/imu', 1000)
        timer_period = 0.025  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.mpu = adafruit_mpu6050.MPU6050(board.I2C())

    def timer_callback(self):
        # Create an Imu message
        imu_msg = Imu()

        # Populate the Imu message with data from the IMU sensor
        imu_msg.header.frame_id = 'imu_link'
        imu_msg.header.stamp = self.get_clock().now().to_msg()

        # Publish the Imu message
        self.publisher.publish(imu_msg)
