import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_amr_navigation = get_package_share_directory('amr_navigation')
    default_waypoints_file = os.path.join(pkg_amr_navigation, 'config', 'patrol_waypoints.yaml')
    default_policy_file = os.path.join(pkg_amr_navigation, 'config', 'security_policy.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    policy_file = LaunchConfiguration('policy_file')
    force_after_hours_mode = LaunchConfiguration('force_after_hours_mode')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_waypoints_file,
        description='Full path to patrol waypoints YAML file'
    )
    declare_policy_file = DeclareLaunchArgument(
        'policy_file',
        default_value=default_policy_file,
        description='Full path to security policy and vision YAML file'
    )
    declare_force_after_hours = DeclareLaunchArgument(
        'force_after_hours_mode',
        default_value='true',
        description='Force restricted-hours security mode'
    )

    guard_node = Node(
        package='amr_navigation',
        executable='patrol_interceptor',
        name='patrol_interceptor_node',
        output='screen',
        parameters=[
            params_file,
            policy_file,
            {
                'use_sim_time': use_sim_time,
                'force_after_hours_mode': force_after_hours_mode
            }
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        declare_policy_file,
        declare_force_after_hours,
        guard_node
    ])
