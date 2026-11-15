import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_amr_navigation = get_package_share_directory('amr_navigation')
    pkg_amr_command_center = get_package_share_directory('amr_command_center')

    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    port = LaunchConfiguration('port')
    rviz = LaunchConfiguration('rviz')
    map_yaml = LaunchConfiguration('map')

    default_map = os.path.join(pkg_amr_navigation, 'maps', 'warehouse_map.yaml')
    default_waypoints_file = os.path.join(pkg_amr_navigation, 'config', 'patrol_waypoints.yaml')
    default_policy_file = os.path.join(pkg_amr_navigation, 'config', 'security_policy.yaml')
    default_map_pgm = os.path.join(pkg_amr_navigation, 'maps', 'warehouse_map.pgm')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )
    declare_port = DeclareLaunchArgument(
        'port',
        default_value='8080',
        description='Command Center Web Dashboard HTTP/WebSocket port'
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Whether to launch RViz2 alongside the Command Center'
    )
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to warehouse map YAML file'
    )

    # 1. Nav2 Autonomous Navigation Stack (Costmaps, Planner, AMCL Localizer)
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_amr_navigation, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml,
            'rviz': rviz
        }.items()
    )

    # 2. 360° Patrol Interceptor Security Guard FSM Node
    guard_node = Node(
        package='amr_navigation',
        executable='patrol_interceptor',
        name='patrol_interceptor_node',
        output='screen',
        parameters=[
            default_waypoints_file,
            default_policy_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # 3. Command Center FastAPI + WebSocket + Dual MJPEG Backend Server
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
            'db_path': '/home/mmdixm/Desktop/amr_security_ws/data/security_incidents.db',
            'snapshot_dir': '/home/mmdixm/Desktop/amr_security_ws/data/snapshots'
        }]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_port,
        declare_rviz,
        declare_map,
        nav_launch,
        guard_node,
        command_center_node
    ])
