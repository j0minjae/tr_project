import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # 각 패키지의 공유 디렉토리 경로를 찾습니다.
    tr_description_dir = get_package_share_directory('tr_description')
    tr_driver_dir = get_package_share_directory('tr_driver')
    tr_kinematics_dir = get_package_share_directory('tr_kinematics_differential2')
    sick_safetyscanners_dir = get_package_share_directory('sick_safetyscanners2')
    teleop_twist_joy_dir = get_package_share_directory('teleop_twist_joy')

    # IncludeLaunchDescription을 사용하여 다른 런치 파일을 포함합니다.
    launch_tr_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tr_description_dir, 'launch', 'tr_description.launch.py')
        )
    )

    launch_tr_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tr_driver_dir, 'launch', 'driving_motor_launch.py')
        )
    )

    launch_tr_kinematics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tr_kinematics_dir, 'launch', 'kinematics.launch.py')
        )
    )
    
    launch_sick_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sick_safetyscanners_dir, 'launch', 'sick_safetyscanners2_launch.py')
        )
    )

    # 파라미터(Launch Argument)를 전달하는 올바른 방법
    launch_teleop_joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(teleop_twist_joy_dir, 'launch', 'teleop-launch.py')
        ),
        launch_arguments={'joy_config': 'ps5'}.items()
    )

    # LaunchDescription을 생성하고 action들을 추가합니다.
    ld = LaunchDescription()

    ld.add_action(launch_tr_description)
    ld.add_action(launch_tr_driver)
    ld.add_action(launch_tr_kinematics)
    ld.add_action(launch_sick_lidar)
    ld.add_action(launch_teleop_joy)

    return ld