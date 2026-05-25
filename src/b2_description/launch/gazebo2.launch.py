from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

from launch.substitutions import Command, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue
 
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = get_package_share_directory('b2_description')
    
    # 1. Генерация robot_description из xacro
    xacro_file = os.path.join(pkg_share, 'xacro', 'robot.xacro')
    robot_description_subst = Command(['xacro ', xacro_file])

    # xacro_file = os.path.join(pkg_share, 'urdf', 'b2_one_leg.urdf')
    # # robot_description_subst = Command(['xacro ', xacro_file])
    # robot_description_subst = Command(['cat ', xacro_file])

    # xacro_file = os.path.join(pkg_share, 'xacro', 'b2_one_leg.xacro')
    # robot_description_subst = Command(['xacro ', xacro_file])
    
    
    robot_description = ParameterValue(robot_description_subst, value_type=str)
    robot_description_param = {'robot_description': robot_description}
    
    # 2. Запуск Gazebo Harmonic с пустым миром (или вашим миром)
    # Используем стандартный launch-файл ros_gz_sim
    gz_launch_file = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
    )
    # Можно использовать empty.sdf или создать свой мир
    world_file = os.path.join(pkg_share, 'worlds', 'empty.sdf')  # создайте папку worlds, если нужно
    if not os.path.exists(world_file):
        world_file = 'empty.sdf'  # тогда Gazebo возьмёт встроенный
    
    start_gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_file),
        launch_arguments={
            'gz_args': f'-r {world_file}',
            'on_exit_shutdown': 'True',
        }.items()
    )
    
    # 3. Мост Gazebo -> ROS2 (синхронизация времени и другие топики)
    bridge_config = os.path.join(pkg_share, 'config', 'gz_bridge.yaml')
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen'
    )
    
    # 4. Спавн модели робота
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'b2', '-topic', 'robot_description'],
        output='screen'
    )
    
    # 5. robot_state_publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description_param, {'use_sim_time': True}],
        output='screen'
    )
    
    # 6. Статическая трансформация base -> base_footprint
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'base', '--child-frame-id', 'base_footprint'
        ],
        output='screen'
    )
    
     # 7. Спавн joint_state_broadcaster
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # 8. Спавн единого effort_controller
    effort_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['effort_controller'],
        output='screen'
    )

    return LaunchDescription([
        start_gz_sim,
        gz_bridge,
        spawn_robot,
        rsp,
        static_tf,
        jsb_spawner,
        effort_spawner,
    ])