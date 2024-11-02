import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import PythonExpression
from pathlib import Path


def generate_launch_description():

    set_ros_domain_id = SetEnvironmentVariable("ROS_DOMAIN_ID", "42")

    return LaunchDescription(
        [
            set_ros_domain_id,
            # SetEnvironmentVariable('PATH', os.path.join(venv_path, 'bin') + ':' + os.environ['PATH']),
            # # Set the PYTHONPATH to include the virtual environment's site-packages
            # SetEnvironmentVariable('PYTHONPATH', os.path.join(venv_path, 'lib', 'python3.12', 'site-packages')),
            Node(
                package="linear_rail_control",
                executable="linear_rail_server",
                name="linear_rail_server",
                output="screen",
                parameters=[{"rail_velocity_mm_s", 40}],
            ),
            Node(
                package="camera_service",
                executable="camera_server",
                name="look_up_camera_server",
                output="screen",
                parameters=[{"service_name": "/batterylab/lookup_camera"}],
            ),
            Node(
                package="sartorius",
                executable="sartorius_server",
                name="sartorius_dispensing_server",
                output="screen",
            ),
        ]
    )
