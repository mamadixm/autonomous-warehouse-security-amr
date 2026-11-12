import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_amr_navigation = get_package_share_directory('amr_navigation')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    # Verify if nav2_controller and navigation stack are installed
    try:
        get_package_share_directory('nav2_controller')
        get_package_share_directory('nav2_bringup')
        nav2_installed = True
    except (PackageNotFoundError, Exception):
        nav2_installed = False

    slam_config_file = os.path.join(pkg_amr_navigation, 'config', 'mapper_params_online_async.yaml')
    nav2_config_file = os.path.join(pkg_amr_navigation, 'config', 'nav2_params.yaml')
    rviz_config_file = os.path.join(pkg_amr_navigation, 'rviz', 'slam.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_flag = LaunchConfiguration('rviz')
    auto_start = LaunchConfiguration('auto_start')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 for SLAM and Nav2 visualization'
    )

    declare_auto_start = DeclareLaunchArgument(
        'auto_start',
        default_value='true',
        description='Whether exploration begins immediately upon launch'
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

    actions = [
        declare_use_sim_time,
        declare_rviz,
        declare_auto_start,
        odom_to_tf_node,
        slam_toolbox_launch,
    ]

    # 3. Nav2 Navigation Stack (only if nav2_controller package is installed)
    if nav2_installed:
        pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
        nav2_navigation_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav2_config_file,
                'autostart': 'true',
                'use_composition': 'False'
            }.items()
        )
        actions.append(nav2_navigation_launch)
    else:
        nav2_warning = LogInfo(
            msg="\n" + "=" * 78 + "\n"
            "[NOTICE] ros-jazzy-navigation2 is not installed on this machine.\n"
            "To enable full Nav2 Frontier Exploration, please run in a terminal:\n"
            "    sudo apt update && sudo apt install -y ros-jazzy-navigation2\n"
            "Running auto_explorer in Standalone Autonomous Mode in the meantime.\n"
            + "=" * 78 + "\n"
        )
        actions.append(nav2_warning)

    # 4. Auto Explorer Node
    auto_explorer_node = Node(
        package='amr_navigation',
        executable='auto_explorer',
        name='auto_explorer_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'auto_start': auto_start
        }]
    )
    actions.append(auto_explorer_node)

    # 5. RViz2 Visualizer
    rviz_node = Node(
        condition=IfCondition(rviz_flag),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    actions.append(rviz_node)

    return LaunchDescription(actions)
