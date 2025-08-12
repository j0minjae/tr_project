import launch
import launch.actions
import launch.substitutions
import os
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    config = os.path.join(get_package_share_directory('tr_localization2'),'launch','test_setup.yaml')

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='tr_localization2', executable='tr_localization_node', output='screen',
            name='tr_localization2_node', parameters = [config])
    ])