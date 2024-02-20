from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    cameraimu_node = Node(
            package='camera-imu',
            executable='CameraIMUNode',
            name='CameraIMUNode'
        )
    
    imu_node = Node(
            package='camera-imu',
            executable='IMUNode',
            name='IMUNode'
        )
    
    return LaunchDescription([
        cameraimu_node,
        imu_node
    ])