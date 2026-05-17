"""
Standalone-запуск одной только мобильной mecanum-платформы.

Платформа управляется встроенным плагином gz_sim::MecanumDrive.
Для управления через ROS2 поднят ros_gz_bridge: /cmd_vel пробрасывается
в GZ-топик /cmd_vel.

Проверка:
    ros2 launch mobile_base_model mobile_base.launch.py
    # в другом терминале:
    ros2 topic pub --once /cmd_vel geometry_msgs/Twist '{linear: {y: 0.5}}'
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('mobile_base_model')
    xacro_path = os.path.join(pkg_share, 'urdf', 'mobile_base_standalone.urdf.xacro')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_path]),
        value_type=str,
    )

    gz_launch_file = os.path.join(
        get_package_share_directory('ros_gz_sim'),
        'launch', 'gz_sim.launch.py',
    )
    start_gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_file),
        launch_arguments={
            'gz_args': '-r empty.sdf',
            'on_exit_shutdown': 'True',
        }.items(),
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'mobile_base',
                   '-topic', 'robot_description',
                   '-z', '0.05'],
        output='screen',
    )

    # Мост: /cmd_vel и /odom + часы
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        output='screen',
    )

    return LaunchDescription([start_gz, bridge, rsp, spawn])
