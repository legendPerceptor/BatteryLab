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
            # Node(
            #     package='camera_service',
            #     executable='camera_server',
            #     name='tower_camera_server',
            #     output='screen',
            #     parameters=[
            #         {'service_name': '/batterylab/tower_camera'}
            #     ]
            # ),
            Node(
                package="usb_cam",
                executable="usb_cam_node_exe",
                name="tower_usb_cam",
                output="screen",
                parameters=[
                    {
                        "video_device": "/dev/video0",
                        "image_width": 640,
                        "image_height": 480,
                        "framerate": 30.0,
                        "pixel_format": "yuyv",
                        "camera_name": "my_tower_cam",
                    }
                ],
                remappings=[("/image_raw", "/camera/tower_camera")],
            ),
        ]
    )
