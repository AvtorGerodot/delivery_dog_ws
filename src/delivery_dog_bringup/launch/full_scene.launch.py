import os
import math
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    RegisterEventHandler
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch.conditions import IfCondition


from launch.substitutions import PythonExpression
from launch.conditions import IfCondition



def generate_launch_description():
    pkg_bringup = get_package_share_directory('delivery_dog_bringup')
    pkg_b2 = get_package_share_directory('b2_description')
    pkg_z1 = get_package_share_directory('z1_model')

    # Аргументы командной строки
    rviz_arg = DeclareLaunchArgument('rviz', default_value='false', description='Launch RViz2')
    x_arg = DeclareLaunchArgument('x', default_value='0.0', description='Spawn X position')
    y_arg = DeclareLaunchArgument('y', default_value='0.0', description='Spawn Y position')
    z_arg = DeclareLaunchArgument('z', default_value='0.8', description='Spawn Z position')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw angle')

    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    use_rviz = LaunchConfiguration('rviz')

    control_mode_arg = DeclareLaunchArgument(
        'control_mode',
        default_value='effort',
        description='Control mode for B2: effort or position'
    )

    control_mode = LaunchConfiguration('control_mode')
    use_effort = PythonExpression(["'", control_mode, "' == 'effort'"])
    use_position = PythonExpression(["'", control_mode, "' == 'position'"])

    # Генерация описания робота из xacro
    xacro_file = os.path.join(pkg_bringup, 'urdf', 'b2_z1.urdf.xacro')
    robot_description_cmd = Command(['xacro ', xacro_file])
    robot_description = ParameterValue(robot_description_cmd, value_type=str)
    robot_description_param = {'robot_description': robot_description}

    # Запуск Gazebo с вашим миром
    world_path = os.path.join(pkg_bringup, 'worlds', 'delivery_world.sdf')
    gz_launch = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    start_gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={
            'gz_args': f'-r {world_path}',
            'on_exit_shutdown': 'True'
        }.items()
    )

    # Мост Gazebo ↔ ROS2
    bridge_config = os.path.join(pkg_bringup, 'config', 'ros_gz_bridge.yaml')
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen'
    )

    # Спавн модели
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'b2_z1',
            '-topic', 'robot_description',
            '-x', x, '-y', y, '-z', z, '-Y', yaw
        ],
        output='screen'
    )

    # robot_state_publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description_param, {'use_sim_time': True}],
        output='screen'
    )

    # Контроллеры
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    effort_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['effort_controller'],
        condition=IfCondition(use_effort),
        output='screen'
    )

    position_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['position_controller'],
        condition=IfCondition(use_position),
        output='screen'
    )

    # Запуск контроллеров после появления модели
    delay_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[jsb_spawner, effort_spawner, position_spawner]
        )
    )

    # RViz2 (опционально)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_bringup, 'config', 'delivery.rviz')],
        condition=IfCondition(use_rviz),
        output='screen'
    )

    return LaunchDescription([
        control_mode_arg, rviz_arg, x_arg, y_arg, z_arg, yaw_arg,
        start_gz, bridge, spawn_robot, rsp, delay_controllers, rviz
    ])