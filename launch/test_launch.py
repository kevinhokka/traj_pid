import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # source fastlio2 工作空间的 setup.bash
        ExecuteProcess(
            cmd=['bash', '-c', 'source ~/fastlio2/install/setup.bash; exec bash'],
            shell=True,
            output='screen'
        ),

        # source ws_livox 工作空间的 setup.bash
        ExecuteProcess(
            cmd=['bash', '-c', 'source ~/ws_livox/install/setup.bash; exec bash'],
            shell=True,
            output='screen'
        ),

        # 启动 fast-lio2 节点
        Node(
            package='fastlio2',
            executable='lio_launch.py',
            name='fast_lio2_node',
            output='screen',
            parameters=[],
            remappings=[]
        ),

        # 启动 planner 中的 test_bspline 节点
        Node(
            package='planner',
            executable='test_bspline',
            name='test_bspline_node',
            output='screen',
            parameters=[],
            remappings=[]
        ),

        # 启动 planner 中的 traj_server 节点
        Node(
            package='planner',
            executable='traj_server',
            name='traj_server_node',
            output='screen',
            parameters=[],
            remappings=[]
        ),

        # 启动 traj_pid_node 节点
        Node(
            package='traj_pid_node',
            executable='traj_pid_node',
            name='traj_pid_node',
            output='screen',
            parameters=[],
            remappings=[]
        )
    ])
