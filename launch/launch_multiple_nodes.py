import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        # 启动 planner 包中的 traj_server 节点
        Node(
            package='planner',
            executable='traj_server',
            name='traj_server_node',
            output='screen',
            parameters=[],
            remappings=[]
        ),

        # 启动 traj_pid 包中的 traj_pid_node 节点
        Node(
            package='traj_pid',
            executable='traj_pid_node',
            name='traj_pid_node',
            output='screen',
            parameters=[],
            remappings=[]
        ),
        
        TimerAction(
            period=0.0,  # 修改为浮点数
            actions=[
                Node(
                        package='planner',
                        executable='test_bspline',
                        name='test_bspline_node',
                        output='screen',
                        parameters=[],
                        remappings=[]
                    )
            ]
        ),
    ])

