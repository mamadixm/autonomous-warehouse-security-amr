import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    AppendEnvironmentVariable
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_andino_description = get_package_share_directory('andino_description')
    pkg_amr_description = get_package_share_directory('amr_description')
    pkg_amr_gazebo = get_package_share_directory('amr_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Resource paths so Gazebo can resolve meshes and fuel models
    andino_share_parent = os.path.dirname(pkg_andino_description)
    fuel_models_path = os.path.expanduser('~/.gz/fuel/fuel.gazebosim.org/openrobotics/models')

    env_gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        f"{andino_share_parent}:{fuel_models_path}:{pkg_amr_gazebo}/worlds"
    )
    env_ign_resource_path = AppendEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        f"{andino_share_parent}:{fuel_models_path}:{pkg_amr_gazebo}/worlds"
    )

    # Launch Configurations
    world_name = LaunchConfiguration('world')
    rviz_flag = LaunchConfiguration('rviz')
    headless_flag = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')
    spawn_x = LaunchConfiguration('x')
    spawn_y = LaunchConfiguration('y')
    spawn_z = LaunchConfiguration('z')
    spawn_yaw = LaunchConfiguration('yaw')

    # Arguments
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

    # URDF Processing
    xacro_file = os.path.join(pkg_amr_description, 'urdf', 'robot.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    world_path = PathJoinSubstitution([pkg_amr_gazebo, 'worlds', world_name])
    bridge_config_path = os.path.join(pkg_amr_gazebo, 'config', 'bridge.yaml')

    # Gazebo simulation arguments
    gz_args_expr = PythonExpression([
        "'-s -r ' + '", world_path, "' if '", headless_flag, "' == 'true' else '-r ' + '", world_path, "'"
    ])

    # 1. Gazebo Harmonic Simulator
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args_expr}.items()
    )

    # 2. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': use_sim_time
        }]
    )

    # 3. Spawn Robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'andino',
            '-string', robot_description_raw,
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
            '-Y', spawn_yaw
        ]
    )

    # 4. ROS-GZ Parameter Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_config_path,
            'use_sim_time': use_sim_time
        }]
    )

    # 5. Optional RViz2
    rviz_node = Node(
        condition=IfCondition(rviz_flag),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        env_gz_resource_path,
        env_ign_resource_path,
        declare_world_cmd,
        declare_rviz_cmd,
        declare_headless_cmd,
        declare_use_sim_time_cmd,
        declare_x_cmd,
        declare_y_cmd,
        declare_z_cmd,
        declare_yaw_cmd,
        gazebo_sim,
        robot_state_publisher,
        spawn_entity,
        bridge,
        rviz_node
    ])
