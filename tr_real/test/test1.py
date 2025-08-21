#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

# PoseStamped 메시지를 생성하는 헬퍼 함수
def make_pose(x: float, y: float, yaw: float, frame: str = "map") -> PoseStamped:
    p = PoseStamped()
    p.header.frame_id = frame
    p.header.stamp = rclpy.clock.Clock().now().to_msg()
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.orientation.z = math.sin(yaw * 0.5)
    p.pose.orientation.w = math.cos(yaw * 0.5)
    return p

class SegmentedMissionController(Node):
    def __init__(self):
        super().__init__("segmented_mission_controller")
        self._ac_navigate = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.param_client_controller = self.create_client(SetParameters, '/controller_server/set_parameters')
        
        self.param_client_controller.wait_for_service()
        self._ac_navigate.wait_for_server()
        self.get_logger().info("액션 및 파라미터 서버가 준비되었습니다.")

    def set_segment_parameters(self, desired_vel, xy_tol, yaw_tol):
        """한 구간의 주행 파라미터를 한 번에 설정합니다."""
        self.get_logger().info(f"주행 파라미터 설정: 속도={desired_vel} m/s, XY오차={xy_tol} m, Yaw오차={yaw_tol} rad")
        
        py_params = [
            Parameter(name='FollowPath.desired_linear_vel', value=float(desired_vel)),
            Parameter(name='general_goal_checker.xy_goal_tolerance', value=float(xy_tol)),
            Parameter(name='general_goal_checker.yaw_goal_tolerance', value=float(yaw_tol))
        ]

        req = SetParameters.Request()
        req.parameters = [p.to_parameter_msg() for p in py_params]
        
        future = self.param_client_controller.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        all_successful = True
        for result in future.result().results:
            if not result.successful:
                all_successful = False
                self.get_logger().error(f"파라미터 변경 실패: {result.reason}")
        
        return all_successful

    def go_to_pose(self, pose: PoseStamped) -> bool:
        """NavigateToPose 액션을 실행하고 결과를 반환합니다."""
        self.get_logger().info(f"목표 지점 [{pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}] (으)로 이동 시작...")
        goal = NavigateToPose.Goal()
        goal.pose = pose
        
        future = self._ac_navigate.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("목표가 거부되었습니다.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        status = result_future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("목표 지점 도착 성공!")
            return True
        else:
            self.get_logger().error(f"목표 실패 (상태: {status})")
            return False

def main():
    rclpy.init()
    node = SegmentedMissionController()

    # 웨이포인트 정의
    waypoint_0 = make_pose(1.952, -0.053, -1.595)
    waypoint_1 = make_pose(1.876, -7.021,  -3.135)
    waypoint_2 = make_pose(-0.158, -7.05, 0.021)
    waypoint_3 = make_pose(1.876, -7.021, 1.564)
    waypoint_4 = make_pose(1.952, -0.053, -3.166)
    waypoint_5 = make_pose(0.0, 0.0, 0.0)

    # --- 미션 시작 ---
    
    # 🎯 단계 1: 시작 -> WP0
    node.get_logger().info("====== [단계 1] 웨이포인트 0으로 주행 시작 ======")
    params_ok = node.set_segment_parameters(desired_vel=0.3, xy_tol=0.05, yaw_tol=0.25)
    if not params_ok:
        node.get_logger().error("단계 1 파라미터 설정 실패. 임무 중단.")
    else:
        success_step1 = node.go_to_pose(waypoint_0)
        if not success_step1:
            node.get_logger().error("단계 1 실패. 임무를 중단합니다.")
        else:
            # 🚀 단계 2: WP0 -> WP1
            node.get_logger().info("====== [단계 2] 웨이포인트 1으로 주행 시작 ======")
            params_ok = node.set_segment_parameters(desired_vel=1.0, xy_tol=0.05, yaw_tol=0.25)
            if not params_ok:
                node.get_logger().error("단계 2 파라미터 설정 실패. 임무 중단.")
            else:
                success_step2 = node.go_to_pose(waypoint_1)
                if not success_step2:
                    node.get_logger().error("단계 2 실패. 임무를 중단합니다.")
                else:
                    # 🎯 단계 3: WP1 -> WP2
                    node.get_logger().info("====== [단계 3] 웨이포인트 2로 주행 시작 ======")
                    params_ok = node.set_segment_parameters(desired_vel=0.3, xy_tol=0.03, yaw_tol=0.1)
                    if not params_ok:
                        node.get_logger().error("단계 3 파라미터 설정 실패. 임무 중단.")
                    else:
                        success_step3 = node.go_to_pose(waypoint_2)
                        if not success_step3:
                            node.get_logger().error("단계 3 실패. 임무를 중단합니다.")
                        else:
                            # ➡️ 단계 4: WP2 -> WP3
                            node.get_logger().info("====== [단계 4] 웨이포인트 3으로 주행 시작 ======")
                            params_ok = node.set_segment_parameters(desired_vel=0.3, xy_tol=0.05, yaw_tol=0.25)
                            if not params_ok:
                                node.get_logger().error("단계 4 파라미터 설정 실패. 임무 중단.")
                            else:
                                success_step4 = node.go_to_pose(waypoint_3)
                                if not success_step4:
                                    node.get_logger().error("단계 4 실패. 임무를 중단합니다.")
                                else:
                                    # ➡️ 단계 5: WP3 -> WP4
                                    node.get_logger().info("====== [단계 5] 웨이포인트 4로 주행 시작 ======")
                                    params_ok = node.set_segment_parameters(desired_vel=1.0, xy_tol=0.05, yaw_tol=0.25)
                                    if not params_ok:
                                        node.get_logger().error("단계 5 파라미터 설정 실패. 임무 중단.")
                                    else:
                                        success_step5 = node.go_to_pose(waypoint_4)
                                        if not success_step5:
                                            node.get_logger().error("단계 5 실패. 임무를 중단합니다.")
                                        else:
                                            # 🏠 단계 6: WP4 -> WP5 (원점 복귀)
                                            node.get_logger().info("====== [단계 6] 원점(웨이포인트 5)으로 복귀 시작 ======")
                                            params_ok = node.set_segment_parameters(desired_vel=0.3, xy_tol=0.03, yaw_tol=0.1)
                                            if not params_ok:
                                                node.get_logger().error("단계 6 파라미터 설정 실패. 임무 중단.")
                                            else:
                                                success_step6 = node.go_to_pose(waypoint_5)
                                                if success_step6:
                                                    node.get_logger().info("★★★★★ 모든 미션을 성공적으로 완료했습니다! ★★★★★")

    # 임무 종료 후 원래 파라미터로 복원 (선택 사항이지만 권장)
    node.get_logger().info("임무 종료. 파라미터를 기본값으로 복원합니다.")
    node.set_segment_parameters(desired_vel=1.0, xy_tol=0.03, yaw_tol=0.1)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()