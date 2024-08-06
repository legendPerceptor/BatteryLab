import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():

    python_interpreter = "/home/yuanjian/Research/BatteryLab/lab_venv/bin/python"

    return LaunchDescription([
        # SetEnvironmentVariable('PATH', os.path.join(venv_path, 'bin') + ':' + os.environ['PATH']),
        # # Set the PYTHONPATH to include the virtual environment's site-packages
        # SetEnvironmentVariable('PYTHONPATH', os.path.join(venv_path, 'lib', 'python3.12', 'site-packages')), 
        Node(
            package='linear_rail_control',
            executable='linear_rail_server',
            name='linear_rail_server',
            output='screen',
            prefix=f'{python_interpreter} -m'
        ),
    ])