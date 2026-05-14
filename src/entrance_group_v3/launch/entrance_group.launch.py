#!/usr/bin/env python3
"""
Launch-файл для запуска Gazebo Harmonic с моделью entrance_group_v3.

Использование:
    ros2 launch entrance_group_v3 entrance_group.launch.py
    ros2 launch entrance_group_v3 entrance_group.launch.py x:=2.0 y:=1.0
    ros2 launch entrance_group_v3 entrance_group.launch.py yaw:=0.0   # без поворота

ВАЖНО: по умолчанию yaw=pi (180°), чтобы наблюдатель Gazebo (камера со
стороны +X мира) оказался СНАРУЖИ подъезда. Если у вас уже есть свой
мир и нужна другая ориентация - переопределите yaw.
"""
import os
import math
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("entrance_group_v3")
    world_arg = LaunchConfiguration("world").perform(context)
    x_arg     = LaunchConfiguration("x").perform(context)
    y_arg     = LaunchConfiguration("y").perform(context)
    z_arg     = LaunchConfiguration("z").perform(context)
    yaw_arg   = LaunchConfiguration("yaw").perform(context)

    # ----------------------------------------------------------
    # Раскрытие xacro в строку URDF.
    # ----------------------------------------------------------
    import xacro
    xacro_path = os.path.join(pkg_share, "urdf", "entrance_group.urdf.xacro") # "entrance_group_forces.urdf") #
    doc = xacro.process_file(xacro_path)
    robot_description = doc.toxml()

    # ----------------------------------------------------------
    # 1) Запуск Gazebo Harmonic через ros_gz_sim
    # ----------------------------------------------------------
    world_path = os.path.join(pkg_share, 'worlds', 'entrance_world.sdf')

    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            #"gz_args": f"-r {world_arg}", #-v 4
            "gz_args": f"-r {world_path}",
            'on_exit_shutdown': 'True',
        }.items(),
    )

    # ----------------------------------------------------------
    # 2) robot_state_publisher
    # ----------------------------------------------------------
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="entrance_group_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True,
        }],
    )

    # ----------------------------------------------------------
    # 3) Спавн модели с заданным yaw (по умолчанию pi = 180°).
    # ----------------------------------------------------------
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_entrance_group",
        output="screen",
        arguments=[
            "-topic", "/robot_description",
            "-name", "entrance_group_v3",
            "-x", x_arg,
            "-y", y_arg,
            "-z", z_arg,
            "-Y", yaw_arg,
        ],
    )

    # ----------------------------------------------------------
    # 4) ROS-GZ bridge
    # ----------------------------------------------------------
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge_entrance",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/world/default/model/entrance_group_v3/joint_state@"
            "sensor_msgs/msg/JointState[gz.msgs.Model",
        ],
        remappings=[
            ("/world/default/model/entrance_group_v3/joint_state",
             "/entrance_group_v3/joint_states"),
        ],
    )

    return [gz_sim_launch, rsp, spawn, bridge]


def generate_launch_description():
    pkg_share = FindPackageShare("entrance_group_v3")

    # Добавляем share/.. в GZ_SIM_RESOURCE_PATH, чтобы Gazebo нашёл
    # текстуры по пути model://entrance_group_v3/...
    set_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            PathJoinSubstitution([pkg_share, ".."]),
            os.pathsep,
            os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
        ],
    )

    return LaunchDescription([
        set_resource_path,
        DeclareLaunchArgument(
            "world",
            default_value="empty.sdf",
            description="SDF-мир Gazebo",
        ),
        DeclareLaunchArgument("x",   default_value="0.0"),
        DeclareLaunchArgument("y",   default_value="0.0"),
        DeclareLaunchArgument("z",   default_value="0.0"),
        DeclareLaunchArgument(
            "yaw",
            default_value=str(math.pi),
            description="Поворот модели вокруг Z, рад. По умолчанию pi (180°) - "
                        "наблюдатель Gazebo окажется СНАРУЖИ подъезда.",
        ),
        OpaqueFunction(function=launch_setup),
    ])
