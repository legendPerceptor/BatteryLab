import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import PythonExpression
from pathlib import Path

def generate_launch_description():

    set_ros_domain_id = SetEnvironmentVariable('ROS_DOMAIN_ID', '42')

    return LaunchDescription([
        set_ros_domain_id,
        # SetEnvironmentVariable('PATH', os.path.join(venv_path, 'bin') + ':' + os.environ['PATH']),
        # # Set the PYTHONPATH to include the virtual environment's site-packages
        # SetEnvironmentVariable('PYTHONPATH', os.path.join(venv_path, 'lib', 'python3.12', 'site-packages')), 

        Node(
            package='camera_service',
            executable='camera_server',
            name='rail_meca500_camera_server',
            output='screen',
            parameters=[
                {'service_name': '/batterylab/rail_meca500_camera'}
            ]
        ),

        Node(
            package='suction_pump',
            executable='suction_pump_server',
            name='suction_pump_server',
            output='screen'
        )
    ])