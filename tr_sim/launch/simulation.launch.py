# Neobotix GmbH
# Author: Pradheep Padmanabhan
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
    ExecuteProcess, RegisterEventHandler
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.event_handlers import OnShutdown
from launch_ros.substitutions import FindPackageShare
import os
import xacro


def launch_setup(context: LaunchContext, my_neo_env_arg):
    launch_actions = []

    my_neo_environment = my_neo_env_arg.perform(context)
    use_sim_time = True

    # ---------- Gazebo world ----------------------------------------------
    if my_neo_environment in ("neo_workshop", "neo_track1"):
        world_path = os.path.join(
            get_package_share_directory('tr_sim'),
            'worlds',
            f'{my_neo_environment}.world')
    else:
        world_path = my_neo_environment

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py')),
        launch_arguments={
            'world':   world_path,
            'verbose': 'true',
        }.items())

    # ---------- Robot description / spawn ---------------------------------
    robot_description_xacro = os.path.join(
        get_package_share_directory('tr_sim'),
        'robots',
        'amr.urdf.xacro')

    robot_description = xacro.process_file(
        robot_description_xacro,
        mappings={'use_gazebo': 'true'}).toxml()

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'amr', '-topic', '/robot_description'],
        output='screen')

    state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description}],
        output='screen')

    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        prefix='xterm -e',
        name='teleop',
        output='screen')

    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare('tr_sim'), 'configs', 'jtc_params.yaml'])

    load_jsb = Node(                                         # joint_state_broadcaster
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '-c', '/controller_manager',
            '--param-file', controllers_yaml],
        output='screen')

    load_jtc = Node(                                         # joint_trajectory_controller
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_controller',
            '-c', '/controller_manager',
            '--param-file', controllers_yaml],
        output='screen')

    # ---------- Append all actions ----------------------------------------
    launch_actions += [
        state_pub,
        gazebo,
        spawn_entity,
        teleop,
        load_jsb,
        load_jtc
    ]
    return launch_actions


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'world',
        default_value='neo_workshop',
        description='Available worlds: "neo_track1", "neo_workshop"'))

    ld.add_action(OpaqueFunction(
        function=launch_setup,
        args=[LaunchConfiguration('world')]))

    return ld
