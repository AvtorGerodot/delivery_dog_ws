"""
Главный composite-launch.
ИЗМЕНЕНО: mecanum-кинематика больше не нужна (используется встроенный
плагин gz_sim::MecanumDrive внутри SDF). Платформа управляется через
GZ-топик /cmd_vel, который прокинут в ROS2 через ros_gz_bridge.
"""
import os
import math
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
    SetEnvironmentVariable, RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    pkg_bringup  = get_package_share_directory('delivery_dog_bringup')
    pkg_z1       = get_package_share_directory('z1_model')
    pkg_mobile   = get_package_share_directory('mobile_base_model')
    pkg_entrance = get_package_share_directory('entrance_group_v3')

    ex  = LaunchConfiguration('entrance_x').perform(context)
    ey  = LaunchConfiguration('entrance_y').perform(context)
    ez  = LaunchConfiguration('entrance_z').perform(context)
    eyaw= LaunchConfiguration('entrance_yaw').perform(context)
    rx  = LaunchConfiguration('robot_x').perform(context)
    ry  = LaunchConfiguration('robot_y').perform(context)
    rz  = LaunchConfiguration('robot_z').perform(context)
    use_rviz = LaunchConfiguration('rviz').perform(context).lower() in ('1','true','yes')

    import xacro
    composite_xacro = os.path.join(pkg_bringup,  'urdf', 'mobile_z1.urdf.xacro')
    entrance_xacro  = os.path.join(pkg_entrance, 'urdf', 'entrance_group.urdf.xacro')
    controllers_yaml = os.path.join(pkg_bringup, 'config', 'mobile_z1_controllers.yaml')

    composite_doc = xacro.process_file(
        composite_xacro,
        mappings={'controllers_yaml': controllers_yaml,
                  'namespace': '/robot',
                  'cmd_vel_topic': 'cmd_vel'},
    )
    composite_desc = composite_doc.toxml()
    entrance_desc  = xacro.process_file(entrance_xacro).toxml()

    # ---- Gazebo ----
    world_path = os.path.join(pkg_bringup, 'worlds', 'delivery_world.sdf')
    gz_launch  = os.path.join(get_package_share_directory('ros_gz_sim'),
                              'launch', 'gz_sim.launch.py')
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={
            'gz_args': f'-r {world_path}',
            'on_exit_shutdown': 'True',
        }.items(),
    )

    rsp_robot = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='rsp_robot', namespace='robot',
        parameters=[{'robot_description': composite_desc, 'use_sim_time': True}],
        output='screen',
    )

    spawn_robot = Node(
        package='ros_gz_sim', executable='create',
        name='spawn_robot',
        arguments=['-name', 'mobile_z1',
                   '-topic', '/robot/robot_description',
                   '-x', rx, '-y', ry, '-z', rz],
        output='screen',
    )

    rsp_entrance = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='rsp_entrance', namespace='entrance',
        parameters=[{'robot_description': entrance_desc, 'use_sim_time': True}],
        output='screen',
    )

    spawn_entrance = Node(
        package='ros_gz_sim', executable='create',
        name='spawn_entrance',
        arguments=['-name', 'entrance_group_v3',
                   '-topic', '/entrance/robot_description',
                   '-x', ex, '-y', ey, '-z', ez, '-Y', eyaw],
        output='screen',
    )

    # Контроллеры Z1 - после спавна модели
    spawn_jsb = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/robot/controller_manager'],
        output='screen',
    )
    spawn_effort = Node(
        package='controller_manager', executable='spawner',
        arguments=['effort_controller',
                   '--controller-manager', '/robot/controller_manager'],
        output='screen',
    )

    bridge_yaml = os.path.join(pkg_bringup, 'config', 'ros_gz_bridge.yaml')
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{'config_file': bridge_yaml}],
        output='screen',
    )

    delay_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[spawn_jsb, spawn_effort],
        )
    )

    actions = [gz, rsp_robot, spawn_robot, rsp_entrance, spawn_entrance,
               delay_after_spawn, bridge]

    if use_rviz:
        rviz = Node(
            package='rviz2', executable='rviz2',
            name='rviz2', output='screen',
            arguments=['-d', os.path.join(pkg_bringup, 'config', 'delivery.rviz')],
        )
        actions.append(rviz)

    return actions


def generate_launch_description():
    pkg_bringup  = get_package_share_directory('delivery_dog_bringup')
    pkg_z1       = get_package_share_directory('z1_model')
    pkg_mobile   = get_package_share_directory('mobile_base_model')
    pkg_entrance = get_package_share_directory('entrance_group_v3')

    extra_resource_paths = os.pathsep.join([
        os.path.dirname(pkg_entrance),
        os.path.dirname(pkg_z1),
        os.path.dirname(pkg_mobile),
    ])
    set_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=extra_resource_paths + os.pathsep
              + os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    )

    return LaunchDescription([
        set_resource_path,
        DeclareLaunchArgument('robot_x',    default_value='0.0'),
        DeclareLaunchArgument('robot_y',    default_value='0.0'),
        # DeclareLaunchArgument('robot_z',    default_value='0.05'),
        DeclareLaunchArgument('robot_z',    default_value='0.01'),
        DeclareLaunchArgument('entrance_x', default_value='2.5'),
        DeclareLaunchArgument('entrance_y', default_value='0.0'),
        DeclareLaunchArgument('entrance_z', default_value='0.0'),
        DeclareLaunchArgument('entrance_yaw', default_value=str(math.pi)),
        DeclareLaunchArgument('rviz', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
