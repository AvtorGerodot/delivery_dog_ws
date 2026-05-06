from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('z1_model')

    # Создание urdf робота z1 для дальнейшей работы
    # 0) указываем путь к файлу
    z1_xacro_file = os.path.join(pkg_share, 'urdf', 'z1_model.urdf.xacro')   
    # 1) Генерируем Substitution для xacro -> urdf
    robot_description_substitution = Command(['xacro ', z1_xacro_file])
    # 2) Явно указываем, что это строка
    robot_description = ParameterValue(robot_description_substitution, value_type=str)
    # 3) Готовим словарь
    robot_description_param = {'robot_description': robot_description} 

    
    # настройки для gazebo
    world_path = os.path.join(pkg_share, 'worlds', 'z1_world.sdf')
    gz_launch_file = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    gz_bridge_params_file = os.path.join(pkg_share, 'config', 'gz_bridge_param.yaml')



    # запуск симулятора Gazebo Harmonic
    start_gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_file),
         launch_arguments={
            'gz_args': f'-r {world_path}', # флаг -r необходим для запуска симуляции
            'on_exit_shutdown': 'True',
        }.items()
    )

    # Спавн модели (ros_gz_sim вместо gazebo_ros)
    z1_spawner = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'z1', 
            '-topic', '/z1/robot_description'
            ],
        output='screen'
    )

    # Публикация описания робота
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='/z1', 
        parameters=[
            robot_description_param,
            {'use_sim_time': True}
        ]
    )

    # Запуск joint_state_broadcaster (публикует /joint_states)
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/z1/controller_manager'],
        output='screen',
        # parameters=[{'use_sim_time': True}]
    )

    # # Запуск контроллера управления
    # joint_trajectory_controller = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['joint_trajectory_controller', '--controller-manager', '/z1/controller_manager'],
    #     output='screen',
    #     # parameters=[{'use_sim_time': True}]
    # )

    # Запуск контроллера прямого управления моментами
    effort_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['effort_controller', '--controller-manager', '/z1/controller_manager'],
        output='screen'
    )

    # Мост Gazebo - ROS2
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gazebo_bridge',
        parameters=[{'config_file': gz_bridge_params_file}],
        output='screen'
    )

    return LaunchDescription([
        start_gz_sim,
        gz_bridge,
        
        z1_spawner, 
        robot_state_publisher, 
        joint_state_broadcaster,
        # joint_trajectory_controller,
        effort_controller,

        ])