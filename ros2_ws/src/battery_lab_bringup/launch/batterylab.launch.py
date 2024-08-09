import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PythonExpression
from pathlib import Path

def generate_launch_description():

    set_ros_domain_id = SetEnvironmentVariable('ROS_DOMAIN_ID', '42')

    return LaunchDescription([
        set_ros_domain_id,
        DeclareLaunchArgument(
            'remote_machine',
            default_value='yuanjian@yuanjian-rasp5.local',
            description='Remote machine user and IP address or host name'
        ),
        DeclareLaunchArgument(
            'ssh_key',
            default_value=os.path.expanduser('~/.ssh/id_ed25519'),
            description='SSH private key for remote machine'
        ),

        # Node on local machine
        # Node(
        #     package='assembly_robot',
        #     executable='assembly_robot',
        #     name='Assembly Robot Meca500 on Zaber',
        #     output='screen'
        # ),

        # Node on remote machine
        ExecuteProcess(
            cmd=[
                'ssh',
                '-i', LaunchConfiguration('ssh_key'),
                LaunchConfiguration('remote_machine'),
                "'bash -c \"source ~/.ros_env_setup.sh && ros2 launch battery_lab_bringup zaberrasp.launch.py\"'"
            ],
            shell=True,
            output='screen'
        ),

        TimerAction(
            period=5.0,  # Adjust the period to ensure the remote node starts before the local node
            actions=[
                Node(
                    package='linear_rail_control',
                    executable='linear_rail_client',
                    name='linear_rail_client',
                    output='screen'
                ),
            ],
        )
    ])
