#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from nav_msgs.msg import Odometry  # Odometry 메시지 임포트
import time
import threading # 스레딩 라이브러리 임포트

# 저장할 파일 이름
LOG_FILE_PATH = 'waypoint_speed_log.csv'

def make_pose(x: float, y: float, yaw: float, frame: str = "map") -> PoseStamped:
    p = PoseStamped()
    p.header.frame_id = frame
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.position.z = 0.0
    p.pose.orientation.z = math.sin(yaw * 0.5)
    p.pose.orientation.w = math.cos(yaw * 0.5)
    return p

# Action Client와 Subscriber를 모두 갖는 통합 노드
class WaypointSpeedLogger(Node):
    def __init__(self):
        super().__init__("waypoint_speed_logger")
        
        # 1. Action Client 초기화
        self._ac = ActionClient(self, FollowWaypoints, "/follow_waypoints")
        
        # 2. /odom Subscriber 초기화
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # 3. 로깅 제어를 위한 상태 변수
        self.is_logging = False
        
        # 4. CSV 파일 헤더 작성
        with open(LOG_FILE_PATH, 'w') as f:
            f.write('timestamp, linear_x, angular_z\n')
        self.get_logger().info(f"'{LOG_FILE_PATH}' 파일에 로그 저장을 준비합니다.")

    # [수정] 피드백 콜백: 현재 웨이포인트 인덱스에 따라 로깅 플래그 제어
    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(f"[피드백] 현재 목표 웨이포인트: {fb.current_waypoint}")
        
        # 목표가 두 번째 웨이포인트(인덱스 1)일 때만 로깅 시작
        if fb.current_waypoint == 0 and not self.is_logging:
            self.get_logger().info(">>>>> 목표: 웨이포인트 1. 속도 측정을 시작합니다. <<<<<")
            self.is_logging = True
        # 목표가 다른 웨이포인트로 바뀌고, 로깅 중이었다면 로깅 중지
        elif fb.current_waypoint != 0 and self.is_logging:
            self.get_logger().info(">>>>> 웨이포인트 1 구간 통과. 속도 측정을 중지합니다. <<<<<")
            self.is_logging = False

    # [추가] /odom 콜백: is_logging이 True일 때만 파일에 데이터 저장
    def odom_callback(self, msg):
        if self.is_logging:
            timestamp = time.time()
            linear_x = msg.twist.twist.linear.x
            angular_z = msg.twist.twist.angular.z
            
            with open(LOG_FILE_PATH, 'a') as f:
                f.write(f'{timestamp}, {linear_x}, {angular_z}\n')

    # 웨이포인트 목표를 전송하고 결과를 기다리는 함수 (기존 send_goal)
    def run_mission(self, poses):
        goal = FollowWaypoints.Goal()
        goal.poses = poses
        
        self.get_logger().info("'/follow_waypoints' 액션 서버를 기다리는 중...")
        if not self._ac.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("액션 서버를 찾을 수 없습니다!")
            return
            
        self.get_logger().info(f"{len(poses)}개의 웨이포인트를 전송합니다.")
        send_future = self._ac.send_goal_async(goal, feedback_callback=self._feedback_cb)
        rclpy.spin_until_future_complete(self, send_future)
        
        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("목표가 거부되었습니다.")
            return

        self.get_logger().info("목표가 수락되었습니다. 주행을 시작합니다...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result_msg = result_future.result()
        result = result_msg.result
        self.get_logger().info(f"주행 완료: 놓친 웨이포인트 {list(result.missed_waypoints)}")
        
        # 모든 임무 완료 후 ROS 종료
        self.get_logger().info("모든 임무 완료. 3초 후 노드를 종료합니다.")
        time.sleep(3)
        self.destroy_node()
        rclpy.shutdown()


def main():
    rclpy.init()
    node = WaypointSpeedLogger()
    
    # 여기에 원하는 좌표 찍기 (map 좌표계 기준)
    poses = [
        make_pose(1.956, -0.029, -1.592), # 0번 웨이포인트
        make_pose(1.906, -7.050,  -3.135), # 1번 웨이포인트
        make_pose(-0.119, -7.086, -3.123), # 2번 웨이포인트
    ]
    
    now = node.get_clock().now().to_msg()
    for p in poses:
        p.header.stamp = now

    # 중요: 미션 실행은 별도의 스레드에서 실행
    # 메인 스레드는 rclpy.spin()으로 콜백 처리를 계속해야 하기 때문
    mission_thread = threading.Thread(target=node.run_mission, args=(poses,))
    mission_thread.start()

    # 메인 스레드는 노드가 종료될 때까지 콜백을 계속 처리
    rclpy.spin(node)

    mission_thread.join()


if __name__ == "__main__":
    main()