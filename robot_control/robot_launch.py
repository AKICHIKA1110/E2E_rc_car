from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_package',
            executable='controller_node',
            name='controller_node',
            output='screen'
        ),
        Node(
            package='my_robot_package',
            executable='motor_node',
            name='motor_node',
            output='screen'
        ),
        Node(
            package='my_robot_package',
            executable='camera_node',
            name='camera_node',
            output='screen'
        ),
    ])
