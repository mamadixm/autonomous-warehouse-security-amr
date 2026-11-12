#!/usr/bin/env python3
"""
Single-Command End-to-End Autonomous Warehouse Security AMR Demo.

Launches:
  1. Gazebo Harmonic Simulation (amr_gazebo/warehouse.launch.py)
  2. Live Web Command Center (amr_command_center/command_center_node on port 8080)
  3. Mission Orchestrator (mission_orchestrator_node) managing:
     - Stage 1: Simulation & Live Command Center online
     - Stage 2: Autonomous Frontier Mapping & Return to Base (Phase 1)
     - Stage 3: Clean Transition to Nav2 Security Patrol (Phase 2)
     - Stage 4: Automated Intruder Spawn & Interception (t + 30s into Patrol)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_amr_gazebo = get_package_share_directory('amr_gazebo')
    pkg_amr_navigation = get_package_share_directory('amr_navigation')
    pkg_amr_command_center = get_package_share_directory('amr_command_center')

    # Default file paths
    default_map_yaml = os.path.join(pkg_amr_navigation, 'maps', 'warehouse_map.yaml')
    default_map_pgm = os.path.join(pkg_amr_navigation, 'maps', 'warehouse_map.pgm')

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')
    rviz = LaunchConfiguration('rviz')
    port = LaunchConfiguration('port')
    force_after_hours_mode = LaunchConfiguration('force_after_hours_mode')
    intruder_delay_sec = LaunchConfiguration('intruder_delay_sec')
    spawn_intruder_x = LaunchConfiguration('spawn_intruder_x')
    spawn_intruder_y = LaunchConfiguration('spawn_intruder_y')
    spawn_intruder_z = LaunchConfiguration('spawn_intruder_z')
    map_yaml = LaunchConfiguration('map')

    # Launch Arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )
    declare_headless = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless mode (true for server/CI, false for full GUI)'
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 visualizer during mapping and navigation phases'
    )
    declare_port = DeclareLaunchArgument(
        'port',
        default_value='8080',
        description='Command Center Web UI / WebSocket port'
    )
    declare_force_after_hours = DeclareLaunchArgument(
        'force_after_hours_mode',
        default_value='true',
        description='Force restricted-hours intruder detection regardless of system clock'
    )
    declare_intruder_delay = DeclareLaunchArgument(
        'intruder_delay_sec',
        default_value='30.0',
        description='Simulation-time countdown seconds after entering PATROL before spawning intruder'
    )
    declare_spawn_x = DeclareLaunchArgument(
        'spawn_intruder_x',
        default_value='1.8',
        description='Intruder target spawn X coordinate'
    )
    declare_spawn_y = DeclareLaunchArgument(
        'spawn_intruder_y',
        default_value='1.2',
        description='Intruder target spawn Y coordinate'
    )
    declare_spawn_z = DeclareLaunchArgument(
        'spawn_intruder_z',
        default_value='0.35',
        description='Intruder target spawn Z coordinate'
    )
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=default_map_yaml,
        description='Warehouse map YAML file path'
    )

    # 1. Gazebo Harmonic Simulation Launch (amr_gazebo)
    gazebo_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_amr_gazebo, 'launch', 'warehouse.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'headless': headless,
            'rviz': 'false'  # RViz handled dynamically per mission phase
        }.items()
    )

    # 2. Command Center Backend Node (Active from t = 0s)
    command_center_node = Node(
        package='amr_command_center',
        executable='command_center_node',
        name='amr_command_center_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'port': port,
            'host': '0.0.0.0',
            'map_yaml': map_yaml,
            'map_pgm': default_map_pgm,
            'db_path': '/home/mmdixm/amr_project_ws/data/security_incidents.db',
            'snapshot_dir': '/home/mmdixm/amr_project_ws/data/snapshots'
        }]
    )

    # 3. Mission Orchestrator Node
    mission_orchestrator_node = Node(
        package='amr_command_center',
        executable='mission_orchestrator',
        name='mission_orchestrator_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'rviz': rviz,
            'port': port,
            'force_after_hours_mode': force_after_hours_mode,
            'intruder_delay_sec': intruder_delay_sec,
            'spawn_intruder_x': spawn_intruder_x,
            'spawn_intruder_y': spawn_intruder_y,
            'spawn_intruder_z': spawn_intruder_z,
            'map_yaml': map_yaml,
            'map_pgm': default_map_pgm,
            'max_mapping_time_sec': 300.0
        }]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_headless,
        declare_rviz,
        declare_port,
        declare_force_after_hours,
        declare_intruder_delay,
        declare_spawn_x,
        declare_spawn_y,
        declare_spawn_z,
        declare_map,
        gazebo_sim_launch,
        command_center_node,
        mission_orchestrator_node
    ])
