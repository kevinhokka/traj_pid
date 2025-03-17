import launch
from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 启动前清理 test_bspline 节点残留进程（请根据实际情况调整匹配条件）
        ExecuteProcess(
            cmd=['pkill', '-f', 'test_bspline'],
            output='screen'
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
            period=0.0,  # 延迟时间，单位为秒
            actions=[
                Node(
                    package='virtual_data_publisher',
                    executable='virtual_data_publisher',
                    name='virtual_data_publisher_node',
                    output='screen',
                    parameters=[],
                    remappings=[]
                )
            ]
        ),

        TimerAction(
            period=0.0,  # 延迟时间，单位为秒
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

        # 在整个 launch 关闭时清理 test_bspline 节点残留进程
        RegisterEventHandler(
            event_handler=OnShutdown(
                on_shutdown=[
                    ExecuteProcess(
                        cmd=['pkill', '-f', 'test_bspline'],
                        output='screen'
                    )
                ]
            )
        )
    ])
