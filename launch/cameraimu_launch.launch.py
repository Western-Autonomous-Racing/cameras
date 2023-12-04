from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    cameraimu_node = Node(
            package='camera_imu',
            executable='CameraIMU',
            name='CameraIMU'
        )

    return LaunchDescription([
        cameraimu_node
    ])