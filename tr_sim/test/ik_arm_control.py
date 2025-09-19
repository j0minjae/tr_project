
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import pinocchio as pin
import numpy as np
import sys
import threading
import time
import cmd

class DualArmIKController(cmd.Cmd):
    intro = "Welcome to the Dual Arm Simulation Shell.\nType 'help' or '?' to list commands."
    prompt = "(dual_arm_sim) "

    def __init__(self, urdf_path):
        super().__init__()
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node('dual_arm_action_client')
        self.left_arm_client = ActionClient(self.node, FollowJointTrajectory, '/left_arm_controller/follow_joint_trajectory')
        # self.right_arm_client = ActionClient(self.node, FollowJointTrajectory, '/right_arm_controller/follow_joint_trajectory')
        self.subscription = self.node.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # --- Pinocchio Setup ---
        try:
            self.model = pin.buildModelFromUrdf(urdf_path)  #urdf를 읽음
        except Exception as e:
            self.node.get_logger().error(f"Failed to load URDF: {e}")
            sys.exit(1)
        self.data = self.model.createData() #urdf를 읽고 생성한 데이터를 저장
        self.full_joint_positions = pin.neutral(self.model) #관절의 기본 자세값을 가져옴 
        self.joint_state_lock = threading.Lock()    #관절 상태를 가져올떄 잠시lock을 걸어서 정확한 계산을 위함

        ee_frame_name= 'openarm_left_hand'
        self.ee_frame_id = self.model.getFrameId(ee_frame_name)
        self.joint_names = [f'openarm_left_joint{i}' for i in range(1, 8)]

        joint_ids = [self.model.getJointId(name) for name in self.joint_names]
        self.q_indices = [self.model.joints[jid].idx_q for jid in joint_ids]
        self.v_indices = [self.model.joints[jid].idx_v for jid in joint_ids]
    
        print("Waiting for action servers...")
        if not self.left_arm_client.wait_for_server(timeout_sec=5.0):
            print("\nAction servers not available! Please check if the simulation and controllers are running.")
            self.node.destroy_node()
            rclpy.shutdown()
            raise SystemExit("Could not connect to action servers.")
        
        self.node.get_logger().info("Dual Arm IK Controller initialized successfully.")

    # def setup_arm(self,):
    #     # Pinocchio Frame and Joint Info
    #     joint_indices = [self.model.getJointId(name) for name in self.joint_names]
    #     # Poses
    #     home_pose = np.array([0.0, 0.0, 0.0, 1.57, 0.0, 0.0, 0.0])

    def joint_state_callback(self, msg):
        """
        Callback function for the /joint_states subscriber.
        Updates the full configuration vector.
        """
        with self.joint_state_lock:
            for i, name in enumerate(msg.name):
                try:
                    joint_id = self.model.getJointId(name)
                    if joint_id < len(self.model.joints):
                        q_index = self.model.joints[joint_id].idx_q
                        self.full_joint_positions[q_index] = msg.position[i]
                except ValueError:
                    pass

    def get_jacobian(self, q):
        """
        Computes the Jacobian for the specified arm's joints.
        """
        full_J = pin.computeFrameJacobian(self.model, self.data, q, self.ee_frame_id, pin.ReferenceFrame.LOCAL)
        arm_J = full_J[:, self.v_indices]
        return arm_J

    def ik_solver(self, target_placement, initial_q_full):
        """
        A simple inverse kinematics solver for a specific arm.
        """
        q_full = initial_q_full.copy()
        max_iterations, damping, tolerance, gain = 100, 1e-4, 1e-3, 0.5

        for i in range(max_iterations):
            # 1. 현재 EE의 전체 placement(위치+회전)를 계산
            pin.forwardKinematics(self.model, self.data, q_full)
            pin.updateFramePlacements(self.model, self.data)
            ee_placement = self.data.oMf[self.ee_frame_id]

            # 2. (핵심) 6차원 오차(error) 계산
            # 목표와 현재 사이의 변환 차이를 계산하고, 이를 6차원 벡터로 변환합니다.
            error = pin.log(ee_placement.inverse() * target_placement).vector

            if np.linalg.norm(error) < tolerance:
                self.node.get_logger().info(f"Target for arm reached in {i} iterations.")
                return q_full[self.q_indices]

            J_arm = self.get_jacobian(q_full)

            # pesudo inverse
            J_pinv = J_arm.T @ np.linalg.inv(J_arm @ J_arm.T + damping * np.eye(6))
            dq_arm = J_pinv @ error

            dq_full = np.zeros(self.model.nv)
            for j, v_idx in enumerate(self.v_indices):
                dq_full[v_idx] = dq_arm[j]

            q_full = pin.integrate(self.model, q_full, gain * dq_full)

        self.node.get_logger().warn(f"IK solver for arm could not reach the target.")
        return q_full[self.q_indices]

    def do_move(self, arg):
        """Moves the end-effector to a predefined target pose inside the code."""
        # --- DEFINE YOUR TARGET POSE HERE ---
        # Target position (x, y, z) in meters
        target_position = np.array([0.616, 0.154, 0.570])

        # Target orientation as Roll, Pitch, Yaw in radians
        # For example, (0, pi, 0) is an orientation facing downwards
        roll, pitch, yaw = -1.306, -1.568, -1.836
        target_rotation = pin.utils.rpyToMatrix(roll, pitch, yaw)
        # -------------------------------------

        # Optional: Get duration from argument, e.g., "move_predefined 3.0"
        try:
            duration = float(arg) if arg else 2.0
        except ValueError:
            print("Invalid duration. Using default.")
            duration = 2.0
        
        with self.joint_state_lock:
            initial_q_full = self.full_joint_positions.copy()

        target_placement = pin.SE3(target_rotation, target_position)

        # Calculate joint positions using IK
        joint_positions = self.ik_solver(target_placement, initial_q_full)

        if joint_positions is not None:
            self.node.get_logger().info(f"IK solution found: {joint_positions}")
            self.send_goal(joint_positions, duration)
            self.node.get_logger().info("Goal sent to the arm.")
        else:
            self.node.get_logger().warn("IK solution not found. Arm will not move.")

    def do_home(self, arg):
        """Move both arms to the 'home' position."""
        print("--- Moving both arms to HOME ---")
        goal = FollowJointTrajectory.Goal()

        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        joint_positions = [0.0, 0.0, 0.0, 1.57, 0.0, 0.0, 0.0]

        point = JointTrajectoryPoint()
        duration = 5.0
        point.positions = np.array(joint_positions, dtype=np.float64).tolist()
        point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
        
        trajectory.points.append(point)
        goal.trajectory = trajectory

        self.left_arm_client.send_goal_async(goal)

    def do_quit(self, arg):
        """Exit the application."""
        return True

    def do_exit(self, arg):
        """Exit the application."""
        return True

    def send_goal(self, positions, duration=3.0):
        """
        Sends a goal to the specified arm's FollowJointTrajectory action server.
        """
        goal_msg = FollowJointTrajectory.Goal()

        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions.tolist()    #파이싼 기본 리스트로 변환
        point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
        trajectory.points.append(point)
        goal_msg.trajectory = trajectory

        self.left_arm_client.send_goal_async(goal_msg)

def main(args=None):
    # 1. ROS 2 초기화
    rclpy.init(args=args)

    # 2. URDF 경로 설정 및 컨트롤러 클래스 생성
    #    (기존 코드의 DualArmIKController(cmd.Cmd, urdf_path)는 잘못된 호출 방식입니다.)
    urdf_path = "/home/home/tr_project/src/tr_description/urdf/amr_sim.urdf"
    ik_controller = DualArmIKController(urdf_path)

    # 3. ROS 2 노드를 별도 스레드에서 실행 (콜백 함수 처리를 위함)
    #    rclpy.spin의 인자로 노드 객체(ik_controller.node)를 명확히 전달합니다.
    thread = threading.Thread(target=rclpy.spin, args=(ik_controller.node,))
    thread.start()

    # 4. cmdloop()를 사용하여 사용자 명령어 입력을 받음 (이 함수가 종료될 때까지 프로그램이 여기서 대기함)
    try:
        ik_controller.cmdloop()
    except (KeyboardInterrupt, SystemExit):
        # 사용자가 Ctrl+C 등으로 종료했을 때 메시지 출력
        print("\nExiting shell.")
    finally:
        # 5. 프로그램 종료 시 ROS 2 노드 및 스레드를 안전하게 종료
        print("Shutting down...")
        ik_controller.node.destroy_node()
        rclpy.shutdown()
        thread.join() # 스레드가 완전히 끝날 때까지 기다림

if __name__ == '__main__':
    main()