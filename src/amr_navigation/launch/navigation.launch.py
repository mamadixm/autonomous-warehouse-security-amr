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
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Default file paths
    default_map_file = os.path.join(pkg_amr_navigation, 'maps', 'warehouse_map.yaml')
    default_params_file = os.path.join(pkg_amr_navigation, 'config', 'nav2_params.yaml')

    # Launch Configurations
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    rviz_flag = LaunchConfiguration('rviz')

    # Declare Launch Arguments
    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=default_map_file,
        description='Full path to map yaml file to load'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to launch RViz2 for navigation'
    )

    # 1. Odom to TF Broadcaster (odom -> base_footprint)
    odom_to_tf_node = Node(
        package='amr_navigation',
        executable='odom_to_tf',
        name='odom_to_tf_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 2. Nav2 Bringup Launch
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart
        }.items()
    )

    # 3. RViz2 Launch for Nav2
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'rviz_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(rviz_flag)
    )

    return LaunchDescription([
        declare_map_cmd,
        declare_params_file_cmd,
        declare_use_sim_time_cmd,
        declare_autostart_cmd,
        declare_rviz_cmd,
        odom_to_tf_node,
        nav2_bringup_launch,
        rviz_launch
    ])
