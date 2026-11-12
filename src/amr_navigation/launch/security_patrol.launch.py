#!/usr/bin/env python3
"""
Unified Security Patrol Launch File (Phase 2).
Combines:
  1. Nav2 Autonomous Navigation & AMCL Localization (navigation.launch.py)
  2. 360° Dual-Camera Security Guard & Intruder Interceptor FSM (guard.launch.py)
  3. Optional RViz2 Navigation Visualizer
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_amr_navigation = get_package_share_directory('amr_navigation')

    default_map = os.path.join(pkg_amr_navigation, 'maps', 'warehouse_map.yaml')
    default_nav2_params = os.path.join(pkg_amr_navigation, 'config', 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    rviz = LaunchConfiguration('rviz')
    force_after_hours_mode = LaunchConfiguration('force_after_hours_mode')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to warehouse map YAML file'
    )
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=default_nav2_params,
        description='Full path to Nav2 parameters YAML'
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 for navigation and patrol visualization'
    )
    declare_force_after_hours = DeclareLaunchArgument(
        'force_after_hours_mode',
        default_value='true',
        description='Force restricted-hours intruder detection regardless of time'
    )

    # 1. Nav2 Bringup (AMCL + Costmaps + Planners + Controllers + odom_to_tf)
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_amr_navigation, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml,
            'params_file': params_file,
            'rviz': rviz,
            'autostart': 'true'
        }.items()
    )

    # 2. Security Guard FSM (patrol_interceptor)
    guard_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_amr_navigation, 'launch', 'guard.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'force_after_hours_mode': force_after_hours_mode
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map,
        declare_params,
        declare_rviz,
        declare_force_after_hours,
        nav_launch,
        guard_launch
    ])
