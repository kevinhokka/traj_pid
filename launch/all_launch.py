import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    gnss_global_planner = os.path.join(
        get_package_share_directory('gnss_global_path_planner'),
        'launch',
        'gnss_combined_launch.py'
    )
    # 获取 livox_ros_driver2 包中的 launch 文件路径
    livox_launch_file = os.path.join(
        get_package_share_directory('livox_ros_driver2'),
        'launch_ROS2',
        'msg_MID360_launch.py'
    )

    # 获取 fastlio2 包中的 launch 文件路径
    fastlio_launch_file = os.path.join(
        get_package_share_directory('fastlio2'),
        'launch',
        'lio_launch.py'
    )

    # 获取 planner 包中 all.launch.py 的路径
    all_launch_file = os.path.join(
        get_package_share_directory('planner'),
        'launch',
        'all.launch.py'
    )

    # 获取 planner 包中 test_example_target.py 的路径
    test_example_launch_file = os.path.join(
        get_package_share_directory('planner'),
        'launch',
        'test_example_target.py'
    )

    return LaunchDescription([
        # 启动 livox_ros_driver2 的 launch 文件
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(livox_launch_file)
        ),

        # 启动 fastlio2 的 launch 文件
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fastlio_launch_file)
        ),
        
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(gnss_global_planner)
        # ),

        # 启动 serial_twistctl 节点
        Node(
            package='serial_twistctl',
            executable='serial_twistctl_node',
            name='serial_twistctl_node',
            parameters=[],
            remappings=[]
        ),

        # 延时启动 all.launch.py（这里延时8秒，可以根据实际情况调整）
        TimerAction(
            period=4.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(all_launch_file)
                )
            ]
        ),

        #延时启动 test_example_target.py（这里延时12秒，同样可根据需求调整）
        #TimerAction(
        #    period=20.0,
        #    actions=[
        #        IncludeLaunchDescription(
        #            PythonLaunchDescriptionSource(test_example_launch_file)
        #        )
        #    ]
        #),
        
        TimerAction(
            period=10.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(gnss_global_planner)
                )
            ]
        ),

        # 先延时一定时间启动 traj_pid 节点（这里延时4秒）
        TimerAction(
            period=2.0,
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
        )
    ])
