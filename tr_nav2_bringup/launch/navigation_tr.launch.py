# Copyright (c) 2022 MY_KJJ_ROBOTbotix GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('tr_nav2_bringup')
    speed_mask_yaml_file = LaunchConfiguration('speed_mask')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_multi_robots = LaunchConfiguration('use_multi_robots')
    use_rviz = LaunchConfiguration('use_rviz')

    # --- [수정 1] lifecycle_nodes 리스트에 새로 추가할 노드들의 이름을 추가합니다. ---
    lifecycle_nodes = ['controller_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower',
                       'map_server_speed_mask',          # 추가
                       'costmap_filter_info_server']   # 추가

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static'),
                  ('/cmd_vel', '/cmd_vel_nav')]

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart}

    configured_params = RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,

            convert_types=True)

    # ... (DeclareLaunchArgument 부분은 변경 없음) ...
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'config', 'navigation.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_multi_robots_cmd =  DeclareLaunchArgument(
        'use_multi_robots', default_value='False',
        description='A flag to remove the remappings')

    declare_speed_mask_yaml_cmd = DeclareLaunchArgument(
        'speed_mask',
        default_value='',
        description='Full path to speed mask yaml file to load',
    )

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_multi_robots])),
        actions=[
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                parameters=[configured_params],
                remappings=remappings),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[configured_params],
                remappings=remappings),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[configured_params],
                remappings=remappings),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[configured_params],
                remappings=remappings),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                parameters=[configured_params],
                remappings=remappings),

            # --- [수정 2] 여기에 Speed Filter 관련 노드 2개를 추가합니다. ---
            Node(
              package='nav2_map_server',
              executable='map_server',
              name='map_server_speed_mask',
              output='screen',
              parameters=[configured_params],
              remappings=remappings),
          Node(
              package='nav2_map_server',
              executable='costmap_filter_info_server',
              name='costmap_filter_info_server',
              output='screen',
              parameters=[configured_params],
              remappings=remappings),
            # ----------------------------------------------------------------

            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}]), # 수정된 lifecycle_nodes 리스트가 전달됨
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir, 'launch', 'rviz_launch.py')
                ),
                condition = IfCondition(use_rviz),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'namespace': namespace
                }.items(),
            ),
        ]
    )

    # ... (load_nodes_multi_robot 부분은 multi-robot을 사용하지 않으신다면 수정할 필요 없습니다) ...
    
    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_multi_robots_cmd)
    ld.add_action(declare_speed_mask_yaml_cmd)


    # Add the actions to launch all of the navigation nodes
    ld.add_action(load_nodes)
    # ld.add_action(load_nodes_multi_robot) # 필요하다면 이쪽에도 동일하게 추가해야 합니다.

    return ld