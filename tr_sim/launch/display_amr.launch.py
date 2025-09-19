import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def robot_state_publisher_spawner(context: LaunchContext, use_gazebo, use_arm):
    use_gazebo_str = context.perform_substitution(use_gazebo)
    use_arm_str = context.perform_substitution(use_arm)

    xacro_path = os.path.join(
        get_package_share_directory("tr_description"),
        "urdf", "amr_body.urdf.xacro"
    )

    robot_description = xacro.process_file(
        xacro_path,
        mappings={
            "use_gazebo": use_gazebo_str,
            "use_arm": use_arm_str,
            
        }
    ).toprettyxml(indent="  ")

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        )
    ]


def rviz_spawner(context: LaunchContext):
    rviz_config_path = os.path.join(
        get_package_share_directory("tr_description"),
        "rviz", "tr_description.rviz"
    )

    return [
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["--display-config", rviz_config_path],
            output="screen"
        ),
    ]


def generate_launch_description():
    use_gazebo_arg = DeclareLaunchArgument(
        "use_gazebo",
        default_value="true",
        description="Whether to use gazebo simulation"
    )
    use_arm_arg = DeclareLaunchArgument(
        "use_arm",
        default_value="false",
        description="Whether to use dual-arm"
    )
    use_gazebo = LaunchConfiguration("use_gazebo")
    use_arm = LaunchConfiguration('use_arm')

    robot_state_publisher_loader = OpaqueFunction(
        function=robot_state_publisher_spawner,
        args=[use_gazebo, use_arm]
    )

    rviz_loader = OpaqueFunction(
        function=rviz_spawner,
        args=[]
    )

    return LaunchDescription([
        use_gazebo_arg,
        use_arm_arg,
        robot_state_publisher_loader,
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui"
        ),
        rviz_loader,
    ])