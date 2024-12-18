from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Controller Node
        Node(
            package='my_robot_package',
            executable='controller_node',
            name='controller_node',
            output='screen',
            parameters=[{'k': 0.1}, 
                        {'v': 0.4},
                        {'bag_dir': '/media/external_drive/rosbag_data'}]
        # Motor Node
        Node(
            package='my_robot_package',
            executable='motor_node',
            name='motor_node',
            output='screen'
        ),
        # Camera Node
        Node(
            package='my_robot_package',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[{'fps': 30}]
        ),
        # Rosbag Record
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', 'robot_data_bag', '/cmd_vel', '/wheel_speeds', '/camera/image_raw'],
            output='screen'
        ),
    ])
