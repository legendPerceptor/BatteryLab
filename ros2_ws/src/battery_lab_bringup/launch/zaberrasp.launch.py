import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import PythonExpression

def generate_launch_description():

    set_ros_domain_id = SetEnvironmentVariable('ROS_DOMAIN_ID', '42')

    venv_path_argument = DeclareLaunchArgument(
            "venv_path",
            default_value="/home/yuanjian/Research/BatteryLab/lab_venv/",
            description='The python virtual environment path'
        ),
    # venv_path = "/home/yuanjian/Research/BatteryLab/lab_venv/"
    # python_interpreter = str(Path(venv_path) / "bin" / "python")
    # activate_program = str(Path(venv_path) / "bin" / "activate")
    # activate_venv_cmd = f"source {activate_program} &&"

    combined_prefix = PythonExpression([
        '"source ", "', venv_path_argument, '/bin/activate && ', '"'
    ])

    return LaunchDescription([
        set_ros_domain_id,
        # SetEnvironmentVariable('PATH', os.path.join(venv_path, 'bin') + ':' + os.environ['PATH']),
        # # Set the PYTHONPATH to include the virtual environment's site-packages
        # SetEnvironmentVariable('PYTHONPATH', os.path.join(venv_path, 'lib', 'python3.12', 'site-packages')), 
        Node(
            package='linear_rail_control',
            executable='linear_rail_server',
            name='linear_rail_server',
            output='screen',
            prefix=combined_prefix
        ),
    ])