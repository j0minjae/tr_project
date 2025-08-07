import launch
import launch.actions
import launch.substitutions
import os
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions

def generate_launch_description():
    config = os.path.join(get_package_share_directory('tr_kinematics_differential2'),'launch','test_setup.yaml')
    return launch.LaunchDescription([
        launch_ros.actions.Node(package='tr_kinematics_differential2', executable='tr_differential_node', output='screen',
            name='tr_differential_node', parameters = [config])
    ])