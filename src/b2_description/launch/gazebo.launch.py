from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'b2_description'
    # urdf = PathJoinSubstitution([
    #     FindPackageShare(pkg_name), 'urdf', 'b2_description.urdf'
    # ])
    # Используем готовый URDF или генерируем через xacro
    # robot_description = Command([FindExecutable(name='xacro'), ' ', urdf])
    
    urdf_xacro = PathJoinSubstitution([
        FindPackageShare(pkg_name), 'xacro', 'robot.xacro'
    ])
    robot_description = Command([FindExecutable(name='xacro'), ' ', urdf_xacro])
    

    return LaunchDescription([
        # 1. Запуск Gazebo
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', 'empty.sdf'],
            output='screen'
        ),
        
        # 2. Мост для синхронизации времени (как в Z1)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen'
        ),
        
        # 3. Спавн робота
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', 'b2', '-topic', 'robot_description'],
            output='screen'
        ),
        
        # 4. Статическая трансформация
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'base', '--child-frame-id', 'base_footprint'
            ],
            output='screen'
        ),
        
        # 5. robot_state_publisher (с симуляционным временем)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
        )
    ])

