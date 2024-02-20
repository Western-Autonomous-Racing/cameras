#!/usr/bin/env python3

import time
import board
import adafruit_mpu6050
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import threading

from sensor_msgs.msg import Imu

class IMUNode(Node):
    def __init__(self):
        super().__init__('IMUNode')

        qos = QoSProfile(depth=1000)
        self.imu_publisher = self.create_publisher(Imu, '/imu', qos)
        self.imu = adafruit_mpu6050.MPU6050(board.I2C())
        self.running_thread = True

        self.imu_thread = threading.Thread(target=self.publish_imu)
        self.imu_thread.start()

    def publish_imu(self):
        while self.running_thread:
            start_time = time.time_ns()
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

            imu_msg.angular_velocity_covariance = [0.0] * 9
            imu_msg.linear_acceleration_covariance = [0.0] * 9
            imu_msg.orientation_covariance = [0.0] * 9

            self.imu_publisher.publish(imu_msg)

            time_elapsed = time.time_ns() - start_time
            time.sleep(max(0, 0.005 - time_elapsed/1000000000))

    def on_shutdown(self):
        self.running_thread = False
        self.imu_thread.join()
        
    
if __name__ == '__main__':
    rclpy.init()
    imu_node = IMUNode()
    try:
        rclpy.spin(imu_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Shutdown ROS and cleanup
        imu_node.on_shutdown()
        # imu_node.destroy_node()
        rclpy.shutdown()
