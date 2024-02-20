import board
import adafruit_mpu6050
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu

class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        self.imu_publisher = self.create_publisher(Imu, '/imu', 1000)
        self.imu = adafruit_mpu6050.MPU6050(board.I2C())

        self.timer = self.create_timer(0.1, self.publish_imu)

    def publish_imu(self):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_mpu6050'

        accel = self.imu.acceleration
        imu_msg.linear_acceleration.x = accel[0]
        imu_msg.linear_acceleration.y = accel[1]
        imu_msg.linear_acceleration.z = accel[2]

        gyro = self.imu.gyro
        imu_msg.angular_velocity.x = gyro[0]
        imu_msg.angular_velocity.y = gyro[1]
        imu_msg.angular_velocity.z = gyro[2]

        self.imu_publisher.publish(imu_msg)