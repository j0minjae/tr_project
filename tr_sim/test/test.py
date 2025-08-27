#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

# PoseStamped 메시지를 생성하는 헬퍼 함수
def make_pose(x: float, y: float, yaw: float, frame: str = "map") -> PoseStamped:
    """지정된 x, y, yaw 값으로 PoseStamped 객체를 생성합니다."""
    p = PoseStamped()
    p.header.frame_id = frame
    # 최신 시간을 사용하기 위해 노드의 시계를 사용하도록 수정
    # p.header.stamp = rclpy.clock.Clock().now().to_msg() # 노드 외부에서는 사용 불가
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.orientation.z = math.sin(yaw * 0.5)
    p.pose.orientation.w = math.cos(yaw * 0.5)
    return p

class WaypointFollower(Node):
    """지정된 웨이포인트 목록을 따라 순차적으로 이동하는 노드."""
    def __init__(self):
        super().__init__("waypoint_follower")
        # Nav2 네비게이션 액션 클라이언트
        self._ac_navigate = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        # 액션 서버가 준비될 때까지 대기
        self.get_logger().info("Nav2 액션 서버('/navigate_to_pose')를 기다리는 중...")
        self._ac_navigate.wait_for_server()
        self.get_logger().info("✅ Nav2 액션 서버가 준비되었습니다.")

    def go_to_pose(self, pose: PoseStamped, wp_idx: int) -> bool:
        """단일 웨이포인트로 이동을 요청하고 결과를 기다립니다."""
        self.get_logger().info(f"WP-{wp_idx} [{pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}] (으)로 이동 시작...")
        
        goal = NavigateToPose.Goal()
        # make_pose 함수에서 생성된 pose에 현재 시간을 스탬프로 추가
        pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose = pose
        
        # 비동기 방식으로 목표 전송
        future = self._ac_navigate.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error(f"❌ WP-{wp_idx} 목표가 거부되었습니다.")
            return False

        # 목표 실행 결과를 기다림
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        status = result_future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"✅ WP-{wp_idx} 도착 성공!")
            return True
        else:
            self.get_logger().error(f"❌ WP-{wp_idx} 도착 실패 (상태 코드: {status})")
            return False

def main():
    rclpy.init()
    node = WaypointFollower()

    # 왕복 주행에 사용할 웨이포인트 목록
    waypoints = [
        make_pose(2.0, 0.0, -1.571),    # 0
        make_pose(2.0, -7.0, -3.141),   # 1
        make_pose(0.0, -7.0, 3.141),    # 2 (방향 수정)
        make_pose(2.0, -7.0, 1.571),    # 3
        make_pose(2.0, 0.0, 3.141),     # 4
        make_pose(0.0, 0.0, 0.0)        # 5 (원점 복귀)
    ]

    # 사용자로부터 왕복 횟수 입력받기
    try:
        repeat_count = int(input(">> 왕복 주행 횟수를 입력하세요: "))
        if repeat_count <= 0: raise ValueError
    except ValueError:
        node.get_logger().error("잘못된 입력입니다. 1 이상의 정수를 입력해야 합니다.")
        rclpy.shutdown()
        return

    # 메인 로직 실행
    try:
        for i in range(repeat_count):
            node.get_logger().info(f"\n★★★★★ 왕복 {i+1}/{repeat_count} 시작 ★★★★★")
            success = True
            for wp_idx, pose in enumerate(waypoints):
                # go_to_pose 함수를 호출하여 각 웨이포인트로 이동
                success = node.go_to_pose(pose, wp_idx) # 인덱스를 1부터 시작하도록
                if not success:
                    node.get_logger().error(f"WP-{wp_idx} 이동 실패! 현재 주행을 중단합니다.")
                    break
            
            if not success:
                break # 실패 시 전체 루프 중단
            
            node.get_logger().info(f"🎉 왕복 {i+1}회 완료! 🎉")

    except KeyboardInterrupt:
        node.get_logger().info("\n사용자에 의해 실행이 중단되었습니다.")
    except Exception as e:
        node.get_actor().error(f"실행 중 예외 발생: {e}")
    finally:
        # 노드 및 rclpy 종료
        node.get_logger().info("스크립트를 종료합니다.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()