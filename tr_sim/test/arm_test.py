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

class ControlSuiteShell(cmd.Cmd):
    intro = "Welcome to the Dual Arm Simulation Shell.\nType 'help' or '?' to list commands."
    prompt = "(dual_arm_sim) "

    def __init__(self):
        cmd.Cmd.__init__(self)
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node('dual_arm_action_client')
        self.left_arm_client = ActionClient(self.node, FollowJointTrajectory, '/left_arm_controller/follow_joint_trajectory')
        self.right_arm_client = ActionClient(self.node, FollowJointTrajectory, '/right_arm_controller/follow_joint_trajectory')
        
        # 각 팔의 관절 이름을 URDF와 정확히 일치시켜야 합니다.
        self.left_joint_names = [f'openarm_left_joint{i}' for i in range(1, 8)]
        self.right_joint_names = [f'openarm_right_joint{i}' for i in range(1, 8)]

        self.poses = {
                    'home': [0.0, 0.0, 0.0, 1.57, 0.0, 0.0, 0.0],
                    'move': [0.0, 0.0, 0.0, 1.57, 1.57, 0.0, 0.0],
        }

        print("Waiting for action servers...")
        if not self.left_arm_client.wait_for_server(timeout_sec=5.0) or \
           not self.right_arm_client.wait_for_server(timeout_sec=5.0):
            print("\nAction servers not available! Please check if the simulation and controllers are running.")
            self.node.destroy_node()
            rclpy.shutdown()
            raise SystemExit("Could not connect to action servers.")
        
        print("Dual arm action clients initialized successfully.")

    # 4. _send_goal 호출 방식에 joint_names 인자를 추가합니다.
    def do_home(self, arg):
        """Move both arms to the 'home' position."""
        print("--- Moving both arms to HOME ---")
        goal = FollowJointTrajectory.Goal()

        trajectory = JointTrajectory()
        trajectory.joint_names = self.left_joint_names
        joint_positions = self.poses.get('home')

        point = JointTrajectoryPoint()
        duration = 5.0
        point.positions = np.array(joint_positions, dtype=np.float64).tolist()
        point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
        
        trajectory.points.append(point)
        goal.trajectory = trajectory

        self.left_arm_client.send_goal_async(goal)

    def do_move(self, arg):
        """Move both arms to the 'move' position."""
        print("--- Moving both arms to HOME ---")
        joint_positions = self.poses.get('move')
        duration = 5.0

        point = JointTrajectoryPoint()
        point.positions = np.array(joint_positions, dtype=np.float64).tolist()
        point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))

        trajectory = JointTrajectory()
        trajectory.joint_names = self.left_joint_names
        trajectory.points.append(point)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        
        self.left_arm_client.send_goal_async(goal)

    def do_quit(self, arg):
        return True

if __name__ == '__main__':
    try:
        shell = ControlSuiteShell()
        shell.cmdloop()
    except (KeyboardInterrupt, SystemExit) as e:
        print(f"\nExiting shell: {e}")
        # 프로그램이 비정상적으로 종료될 때도 노드를 정리합니다.
        if 'shell' in locals() and shell.node and rclpy.ok():
            shell.node.destroy_node()
            rclpy.shutdown()
