import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_amr_navigation = get_package_share_directory('amr_navigation')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    slam_config_file = os.path.join(pkg_amr_navigation, 'config', 'mapper_params_online_async.yaml')
    rviz_config_file = os.path.join(pkg_amr_navigation, 'rviz', 'slam.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_flag = LaunchConfiguration('rviz')
    auto_explore = LaunchConfiguration('auto_explore')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 for SLAM visualization'
    )

    declare_auto_explore = DeclareLaunchArgument(
        'auto_explore',
        default_value='false',
        description='Whether to automatically explore the warehouse without manual teleoperation'
    )

    # 1. Odom to TF Broadcaster (odom -> base_footprint)
    odom_to_tf_node = Node(
        package='amr_navigation',
        executable='odom_to_tf',
        name='odom_to_tf_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 2. SLAM Toolbox Online Async Node
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_config_file
        }.items()
    )

    # 3. Optional Auto Explorer Node
    auto_explorer_node = Node(
        condition=IfCondition(auto_explore),
        package='amr_navigation',
        executable='auto_explorer',
        name='auto_explorer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 4. RViz2 Visualizer
    rviz_node = Node(
        condition=IfCondition(rviz_flag),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_rviz,
        declare_auto_explore,
        odom_to_tf_node,
        slam_toolbox_launch,
        auto_explorer_node,
        rviz_node
    ])
