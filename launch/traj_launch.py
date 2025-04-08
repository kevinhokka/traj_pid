import os

import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnShutdown
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取 livox_ros_driver2 包中 msg_MID360_launch.py 的路径
    livox_launch_file = os.path.join(
        get_package_share_directory('livox_ros_driver2'),
        'launch_ROS2',   # 根据日志提示，该文件在 launch_ROS2 目录下
        'msg_MID360_launch.py'
    )

    # 获取 fastlio2 包中 lio_launch.py 的路径
    fastlio_launch_file = os.path.join(
        get_package_share_directory('fastlio2'),
        'launch',
        'lio_launch.py'
    )

    return LaunchDescription([
        # 包含 livox_ros_driver2 的 launch 文件
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(livox_launch_file)
        ),

        # 包含 fastlio2 的 launch 文件
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fastlio_launch_file)
        ),

        # # 启动 planner 包中的 traj_server 节点
        # Node(
        #     package='planner',
        #     executable='traj_server',
        #     name='traj_server_node',
        #     output='screen',
        #     parameters=[],
        #     remappings=[]
        # ),

        # # 启动 traj_pid 包中的 traj_pid_node 节点
        # Node(
        #     package='traj_pid',
        #     executable='traj_pid_node',
        #     name='traj_pid_node',
        #     output='screen',
        #     parameters=[],
        #     remappings=[]
        # ),

        Node(
            package='serial_twistctl',
            executable='serial_twistctl_node',
            name='serial_twistctl_node',
            output='screen',
            parameters=[],
            remappings=[]
        ),

        # 延时4秒后启动 planner 包中的 test_bspline 节点
        TimerAction(
            period=0.0,
            actions=[
                Node(
                    package='traj_pid',
                    executable='traj_pid_node',
                    name='traj_pid_node',
                    output='screen',
                    parameters=[],
                    remappings=[]
                )
            ]
        ),

        # 延时4秒后启动 planner 包中的 test_bspline 节点
        TimerAction(
            period=4.0,
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


        RegisterEventHandler(
            event_handler=OnShutdown(
                on_shutdown=[
                    ExecuteProcess(
                        cmd=['pkill', '-f', 'test_bspline'],
                        output='screen'
                    )
                ]
            )
        ),

        RegisterEventHandler(
            event_handler=OnShutdown(
                on_shutdown=[
                    ExecuteProcess(
                        cmd=['pkill', '-f', 'rviz2'],
                        output='screen'
                    )
                ]
            )
        )
    ]
)
