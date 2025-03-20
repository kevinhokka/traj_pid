import os

import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnShutdown
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    return LaunchDescription([


        # 启动 traj_pid 包中的 traj_pid_node 节点
        Node(
            package='test_cmd',
            executable='test_cmd_node',
            name='test_cmd_node',
            output='screen',
            parameters=[],
            remappings=[]
        ),

        # 启动 traj_pid 包中的 traj_pid_node 节点
        Node(
            package='serial_twistctl',
            executable='serial_twistctl_node',
            name='serial_twistctl_node',
            output='screen',
            parameters=[],
            remappings=[]
        ),

    ])
