import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_amr_gazebo = get_package_share_directory('amr_gazebo')
    sim_launch_path = os.path.join(pkg_amr_gazebo, 'launch', 'sim.launch.py')

    world = LaunchConfiguration('world')
    rviz = LaunchConfiguration('rviz')
    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='depot.sdf',
        description='Name of the Gazebo world file in worlds/'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Whether to launch RViz2'
    )
    declare_headless_cmd = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless / server mode (-s)'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )
    declare_x_cmd = DeclareLaunchArgument('x', default_value='0.0', description='Robot spawn X')
    declare_y_cmd = DeclareLaunchArgument('y', default_value='0.0', description='Robot spawn Y')
    declare_z_cmd = DeclareLaunchArgument('z', default_value='0.03', description='Robot spawn Z')
    declare_yaw_cmd = DeclareLaunchArgument('yaw', default_value='0.0', description='Robot spawn Yaw')

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch_path),
        launch_arguments={
            'world': world,
            'rviz': rviz,
            'headless': headless,
            'use_sim_time': use_sim_time,
            'x': x,
            'y': y,
            'z': z,
            'yaw': yaw,
        }.items()
    )

    return LaunchDescription([
        declare_world_cmd,
        declare_rviz_cmd,
        declare_headless_cmd,
        declare_use_sim_time_cmd,
        declare_x_cmd,
        declare_y_cmd,
        declare_z_cmd,
        declare_yaw_cmd,
        sim_launch
    ])
