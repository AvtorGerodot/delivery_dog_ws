from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'b2_description'
    urdf_xacro = PathJoinSubstitution([
        FindPackageShare(pkg_name), 'xacro', 'robot.xacro'
    ])
    
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', urdf_xacro
    ])

    rviz_config = PathJoinSubstitution([
        FindPackageShare(pkg_name), 'config', 'config.rviz'
    ])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description_content}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config]
        )
    ])