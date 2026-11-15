import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    port = LaunchConfiguration('port')
    host = LaunchConfiguration('host')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )

    declare_port = DeclareLaunchArgument(
        'port',
        default_value='8000',
        description='FastAPI / WebSocket HTTP server port'
    )

    declare_host = DeclareLaunchArgument(
        'host',
        default_value='0.0.0.0',
        description='FastAPI / WebSocket HTTP server host interface'
    )

    command_center_node = Node(
        package='amr_command_center',
        executable='command_center_node',
        name='amr_command_center_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'port': port,
            'host': host,
            'db_path': '/home/mmdixm/Desktop/amr_security_ws/data/security_incidents.db',
            'snapshot_dir': '/home/mmdixm/Desktop/amr_security_ws/data/snapshots',
            'map_yaml': '/home/mmdixm/Desktop/amr_security_ws/src/amr_navigation/maps/warehouse_map.yaml',
            'map_pgm': '/home/mmdixm/Desktop/amr_security_ws/src/amr_navigation/maps/warehouse_map.pgm',
        }]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_port,
        declare_host,
        command_center_node
    ])
