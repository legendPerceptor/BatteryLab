import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='linear_rail_control',
            executable='linear_rail_server',
            name='linear_rail_server',
            output='screen'
        ),
    ])