sudo apt install ros-$ROS_DISTRO-imu-tools

ros2 launch tr_real bringup_real.launch.py

ros2 launch tr_nav2_bringup rviz_launch.py

ros2 launch tr_nav2_bringup mapping.launch.py

ros2 run nav2_map_server map_saver_cli -f ~/tr_ws/src/tr_real/maps/corridor

ros2 launch tr_real tr_navigation2.launch.py