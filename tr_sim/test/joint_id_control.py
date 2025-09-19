import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import cmd
import numpy as np
import time

# 1. 표준 액션 타입과 메시지 타입을 임포트합니다.
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class DualArmShell(cmd.Cmd):
    intro = "Welcome to the Dual Arm Simulation Shell.\nType 'help' or '?' to list commands."
    prompt = "(dual_arm_sim) "

    def __init__(self):
        super().__init__()
        # ROS 2 노드를 초기화합니다.
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node('dual_arm_action_client')

        # --- 로봇 팔 자세를 미리 정의합니다. ---
        # [joint1, joint2, joint3, joint4, joint5, joint6, joint7]
        self.poses = {
            'home': [0.0, 0.0, 0.0, 1.57, 0.0, 0.0, 0.0],
            'ready': [0.0, 0.5, 0.0, -1.5, 0.0, 2.0, 0.785],
            'stretch': [1.5, -0.2, 0.0, -1.0, 0.0, 1.2, 0.785]
        }
        
        # 2. 확인된 실제 액션 서버 이름과 타입을 사용합니다.
        action_server_name_left = '/left_arm_controller/follow_joint_trajectory'
        action_server_name_right = '/right_arm_controller/follow_joint_trajectory'
        
        self.left_arm_client = ActionClient(self.node, FollowJointTrajectory, action_server_name_left)
        self.right_arm_client = ActionClient(self.node, FollowJointTrajectory, action_server_name_right)
        
        # 각 팔의 관절 이름을 URDF와 정확히 일치시켜야 합니다.
        self.left_joint_names = [f'openarm_left_joint{i}' for i in range(1, 8)]
        self.right_joint_names = [f'openarm_right_joint{i}' for i in range(1, 8)]

        print("Waiting for action servers...")
        if not self.left_arm_client.wait_for_server(timeout_sec=5.0) or \
           not self.right_arm_client.wait_for_server(timeout_sec=5.0):
            print("\nAction servers not available! Please check if the simulation and controllers are running.")
            self.node.destroy_node()
            rclpy.shutdown()
            # cmdloop()가 시작되기 전에 exit()를 호출합니다.
            raise SystemExit("Could not connect to action servers.")
        
        print("Dual arm action clients initialized successfully.")

        
if __name__ == '__main__':
    try:
        shell = DualArmShell()
        shell.cmdloop()
    except (KeyboardInterrupt, SystemExit) as e:
        print(f"\nExiting shell: {e}")
        # 프로그램이 비정상적으로 종료될 때도 노드를 정리합니다.
        if 'shell' in locals() and shell.node and rclpy.ok():
            shell.node.destroy_node()
            rclpy.shutdown()