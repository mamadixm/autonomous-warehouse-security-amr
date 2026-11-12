import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_amr_gazebo = get_package_share_directory('amr_gazebo')
    pkg_amr_navigation = get_package_share_directory('amr_navigation')

    rviz_flag = LaunchConfiguration('rviz')
    slam_flag = LaunchConfiguration('slam')
    guard_flag = LaunchConfiguration('guard')
    headless_flag = LaunchConfiguration('headless')

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 for 3D visualization'
    )
    declare_slam = DeclareLaunchArgument(
        'slam',
        default_value='false',
        description='Run SLAM-Toolbox online mapping'
    )
    declare_guard = DeclareLaunchArgument(
        'guard',
        default_value='true',
        description='Run autonomous warehouse security guard & patrol node'
    )
    declare_headless = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless mode without GUI window'
    )

    # 1. Gazebo Simulation (Depot World + Robot Spawn + Parameter Bridge)
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_amr_gazebo, 'launch', 'sim.launch.py')
        ),
        launch_arguments={
            'rviz': 'false',
            'headless': headless_flag,
        }.items()
    )

    # 2. Autonomous Security Guard Node
    guard_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_amr_navigation, 'launch', 'guard.launch.py')
        ),
        condition=IfCondition(guard_flag)
    )

    # 3. SLAM Toolbox (optional)
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    slam_config_file = os.path.join(pkg_amr_navigation, 'config', 'mapper_params_online_async.yaml')
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        condition=IfCondition(slam_flag),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': slam_config_file
        }.items()
    )

    # 4. RViz2 Visualizer
    rviz_config_file = os.path.join(pkg_amr_navigation, 'rviz', 'slam.rviz')
    rviz_node = Node(
        condition=IfCondition(rviz_flag),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        declare_rviz,
        declare_slam,
        declare_guard,
        declare_headless,
        sim_launch,
        guard_launch,
        slam_launch,
        rviz_node
    ])
