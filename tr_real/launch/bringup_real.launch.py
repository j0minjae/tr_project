# Neobotix GmbH
# Author: Pradheep Padmanabhan

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
from launch_ros.actions import Node
import os
from pathlib import Path

def generate_launch_description():

    launch_tr_description = ExecuteProcess(cmd=["ros2", "launch", "tr_description", "tr_description.launch.py"])
    launch_tr_driver = ExecuteProcess(cmd=["ros2", "launch", "tr_driver", "driving_motor_launch.py"])
    launch_tr_kinematics = ExecuteProcess(cmd=["ros2", "launch", "tr_kinematics_differential2", "kinematics.launch.py"])
    launch_sick_lidar = ExecuteProcess(cmd=["ros2", "launch", "sick_safetyscanners2", "sick_safetyscanners2_launch.py"], shell=True)
    launch_teleop_joy = ExecuteProcess(cmd=["ros2", "launch", "teleop_twist_joy", "teleop-launch.py", "joy_config:='ps5'"])

    ld = LaunchDescription()

    ld.add_action(launch_tr_description)
    ld.add_action(launch_tr_driver)
    ld.add_action(launch_tr_kinematics)
    ld.add_action(launch_sick_lidar)
    ld.add_action(launch_teleop_joy)

    return ld