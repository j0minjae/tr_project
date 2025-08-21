#!/usr/bin/env python3
import math
import time
import rclpy
import numpy as np
import csv
import matplotlib.pyplot as plt
from datetime import datetime
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from tr_driver_msgs.msg import PgvData # PGV 메시지 타입

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

class AgvPerformanceTester(Node):
    def __init__(self):
        super().__init__("agv_performance_tester")
        # 액션, 서비스 클라이언트
        self._ac_navigate = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.param_client_controller = self.create_client(SetParameters, '/controller_server/set_parameters')

        # 구독, 발행
        self.pgv_sub = self.create_subscription(PgvData, '/pgv', self.pgv_callback, 10)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 테스트 설정값 및 결과 변수
        self.STOP_TARGET_WPS = {2, 5}
        self.WP_TO_TAG_MAP = {
            2: 13,
            5: 0
        }
        self.STOP_TOLERANCE_MM = 10.0
        self.PATH_TARGET_TAG_IDS = {1, 3, 4, 5, 6, 7, 8, 10}
        self.PATH_Y_TOLERANCE_MM = 30.0
        self.DEFAULT_XY_TOL = 0.03
        self.PRECISE_XY_TOL = 0.03
        self.is_monitoring_path = False
        self.last_pgv_msg = None

        # 데이터 기반 오프셋 값 (미터 단위)
        self.POSITION_OFFSETS_M = {
            0: {'x': -44.55 / 1000, 'y': 36.60 / 1000},
            13: {'x': -21.11 / 1000, 'y': -34.43 / 1000}
        }

        # 데이터 저장을 위한 변수 초기화
        self.stop_position_data = []
        self.path_y_position_data = []

        # 서버 연결 대기
        self._ac_navigate.wait_for_server()
        if not self.param_client_controller.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Controller Server의 파라미터 서비스에 연결할 수 없습니다.")
            raise RuntimeError("Parameter service not available")
        self.get_logger().info("✅ 액션 및 파라미터 서버가 준비되었습니다.")

    def pgv_callback(self, msg: PgvData):
        self.last_pgv_msg = msg
        if not msg.tag_detected or not self.is_monitoring_path:
            return

        if msg.tag_id in self.PATH_TARGET_TAG_IDS:
            y_abs_mm = abs(msg.y_pos)
            is_within_tolerance = y_abs_mm <= self.PATH_Y_TOLERANCE_MM
            self.path_y_position_data.append({
                "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f"),
                "tag_id": msg.tag_id,
                "y_pos_mm": msg.y_pos
            })
            if not is_within_tolerance:
                self.get_logger().warn(f"🚩 경로 이탈 감지! Tag ID: {msg.tag_id}, Y-Offset: {msg.y_pos:.2f} mm")

    def perform_origin_alignment(self) -> bool:
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("🚦 원점 정렬을 시작합니다. 로봇을 Tag ID 0 위에 위치시켜주세요.")
        self.get_logger().info(f"목표: 태그 중심으로부터 거리 {self.STOP_TOLERANCE_MM:.1f} mm 이내")
        self.get_logger().info("정렬이 완료되면 'y'를 입력하여 원점을 설정합니다. (중단: Ctrl+C)")
        self.get_logger().info("="*60)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.last_pgv_msg is None:
                self.get_logger().warn("PGV 데이터를 수신 대기중...", throttle_duration_sec=2)
                continue
            if not self.last_pgv_msg.tag_detected or self.last_pgv_msg.tag_id != 0:
                print(f"\r\033[K[❌] Tag 0을 찾고 있습니다... (감지 플래그: {self.last_pgv_msg.tag_detected}, ID: {self.last_pgv_msg.tag_id})", end="")
                continue
            x_mm, y_mm = self.last_pgv_msg.x_pos, self.last_pgv_msg.y_pos
            distance_mm = math.sqrt(x_mm**2 + y_mm**2)
            if distance_mm <= self.STOP_TOLERANCE_MM:
                print(f"\r\033[K[✅] 정렬 완료! X: {x_mm:6.2f} mm, Y: {y_mm:6.2f} mm, 거리: {distance_mm:6.2f} mm. 이 위치를 원점으로 설정합니까? (y/n): ", end="")
                try:
                    answer = input()
                    if answer.lower() == 'y':
                        self.get_logger().info("\n원점 설정이 확정되었습니다.")
                        return True
                except (EOFError, KeyboardInterrupt): return False
            else:
                print(f"\r\033[K[ ] 정렬 중... X: {x_mm:6.2f} mm, Y: {y_mm:6.2f} mm, 거리: {distance_mm:6.2f} mm", end="")
        return False

    def measure_stopping_accuracy(self, wp_idx: int):
        if wp_idx not in self.WP_TO_TAG_MAP:
            return
        target_tag_id = self.WP_TO_TAG_MAP[wp_idx]
        self.get_logger().info(f"--- WP-{wp_idx} (Tag ID: {target_tag_id}) 최종 위치 측정을 시작합니다 ---")

        # 1. 측정 시작 전, 이전의 '쓰레기값'을 처리하기 위한 로직
        self.get_logger().info("  - 안정적인 측정을 위해 잠시 대기하며 이전 센서 데이터를 비웁니다...")
        flush_start_time = time.time()
        while time.time() - flush_start_time < 0.3: # 0.3초간 메시지 큐에 쌓인 데이터 소진
            rclpy.spin_once(self, timeout_sec=0.05)
        
        # 2. 로봇이 물리적으로 안정화되고 센서가 새 값을 발행할 때까지 잠시 대기
        time.sleep(0.5)
        self.get_logger().info("  - 최신 데이터 수신 대기를 시작합니다.")

        # 3. 안정적인 최신 데이터 수신을 위한 대기 루프 (최대 2초)
        start_time = self.get_clock().now()
        timeout_sec = 2.0
        valid_msg_received = False
        
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.05)
            if (self.last_pgv_msg and 
                self.last_pgv_msg.tag_detected and
                self.last_pgv_msg.tag_id == target_tag_id):
                self.get_logger().info(f"  - 유효한 PGV 데이터 수신 (Tag ID: {target_tag_id})")
                valid_msg_received = True
                break
            time.sleep(0.1)

        if not valid_msg_received:
            self.get_logger().error(f"WP-{wp_idx}에서 기대한 Tag ID({target_tag_id})를 시간 내에 찾지 못했습니다.")
            if self.last_pgv_msg:
                 self.get_logger().error(f"  - 마지막으로 감지된 태그: ID={self.last_pgv_msg.tag_id}, 감지됨={self.last_pgv_msg.tag_detected}")
            return

        # 4. 유효한 최신 메시지를 받은 후 데이터 기록
        x_mm, y_mm = self.last_pgv_msg.x_pos, self.last_pgv_msg.y_pos
        distance_error_mm = math.sqrt(x_mm**2 + y_mm**2)
        is_within_tolerance = distance_error_mm <= self.STOP_TOLERANCE_MM
        self.stop_position_data.append({
            "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f"),
            "wp": wp_idx,
            "tag_id": target_tag_id,
            "x_pos_mm": x_mm,
            "y_pos_mm": y_mm
        })
        result_str = "PASS" if is_within_tolerance else "FAIL"
        self.get_logger().info(f"  - 태그 중심 오차: {distance_error_mm:.3f} mm -> [{result_str}]")

    def perform_correction_maneuver(self, dx: float, dy: float):
        self.get_logger().info(f"  - 정밀 보정 기동 시작: dx={dx*1000:.1f}mm, dy={dy*1000:.1f}mm")
        
        twist_msg = Twist()
        linear_speed = 0.05
        angular_speed = 0.3
        
        if abs(dy) > 0.001:
            self.get_logger().info(f"    - Y축 보정 (거리: {dy*1000:.1f}mm)")
            turn_time_90_deg = (math.pi / 2) / angular_speed
            
            twist_msg.angular.z = angular_speed
            self.cmd_vel_pub.publish(twist_msg)
            time.sleep(turn_time_90_deg)
            
            twist_msg.angular.z = 0.0
            move_time_y = abs(dy) / linear_speed
            twist_msg.linear.x = math.copysign(linear_speed, dy)
            self.cmd_vel_pub.publish(twist_msg)
            time.sleep(move_time_y)

            twist_msg.linear.x = 0.0
            twist_msg.angular.z = -angular_speed
            self.cmd_vel_pub.publish(twist_msg)
            time.sleep(turn_time_90_deg)

            twist_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(twist_msg)
            time.sleep(0.3)

        if abs(dx) > 0.001:
            self.get_logger().info(f"    - X축 보정 (거리: {dx*1000:.1f}mm)")
            move_time_x = abs(dx) / linear_speed
            twist_msg.linear.x = math.copysign(linear_speed, dx)
            self.cmd_vel_pub.publish(twist_msg)
            time.sleep(move_time_x)
            
            twist_msg.linear.x = 0.0
            self.cmd_vel_pub.publish(twist_msg)
            time.sleep(0.3)

        self.get_logger().info("  - 정밀 보정 기동 완료.")

    def go_to_pose(self, pose: PoseStamped, wp_idx: int) -> bool:
        is_precise_target = wp_idx in self.STOP_TARGET_WPS
        tolerance = self.PRECISE_XY_TOL if is_precise_target else self.DEFAULT_XY_TOL
        if not self.set_goal_tolerance(tolerance): return False

        self.get_logger().info(f"WP-{wp_idx} [{pose.pose.position.x:.3f}, {pose.pose.position.y:.3f}] (으)로 1차 이동 시작...")
        goal = NavigateToPose.Goal(); goal.pose = pose
        self.is_monitoring_path = True
        
        future = self._ac_navigate.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("1차 목표가 거부되었습니다."); return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        status = result_future.result().status
        self.is_monitoring_path = False
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"✅ WP-{wp_idx} 1차 도착 성공!")

            if is_precise_target:
                self.get_logger().info("  - Nav2 목표를 취소하여 /cmd_vel 제어권을 확보합니다.")
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future)
                time.sleep(0.5)

                target_tag_id = self.WP_TO_TAG_MAP.get(wp_idx)
                offset = self.POSITION_OFFSETS_M.get(target_tag_id)
                if offset:
                    dx_correction = offset['x']
                    dy_correction = offset['y']
                    self.perform_correction_maneuver(dx_correction, dy_correction)
            
            self.get_logger().info(f"✅ WP-{wp_idx} 최종 위치 도달!")
            if is_precise_target:
                self.measure_stopping_accuracy(wp_idx)
            return True
        else:
            self.get_logger().error(f"❌ WP-{wp_idx} 도착 실패 (상태: {status})"); return False

    def reset_localization_to_origin(self):
        self.get_logger().info("초기 위치를 (0, 0)으로 재설정합니다...")
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.pose.pose.position.x = 0.0
        initial_pose.pose.pose.position.y = 0.0
        initial_pose.pose.pose.orientation.w = 1.0
        self.initial_pose_pub.publish(initial_pose)
        self.get_logger().info("`/initialpose` 토픽으로 초기 위치 발행 완료.")
        time.sleep(1.0)

    def set_goal_tolerance(self, tolerance: float) -> bool:
        self.get_logger().info(f"목표 허용 오차를 {tolerance * 1000:.1f}mm ({tolerance}m)로 설정합니다.")
        param = Parameter(name='general_goal_checker.xy_goal_tolerance', value=tolerance)
        req = SetParameters.Request(); req.parameters = [param.to_parameter_msg()]
        future = self.param_client_controller.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and all(res.successful for res in future.result().results): return True
        else: self.get_logger().error("파라미터 변경 실패!"); return False

    def save_and_plot_results(self):
        self.get_logger().info("\n" + "="*60 + "\n📊 데이터 저장 및 시각화 중... 📊\n" + "="*60)
        if self.stop_position_data:
            with open('stop_positions.csv', 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=self.stop_position_data[0].keys())
                writer.writeheader()
                writer.writerows(self.stop_position_data)
            self.get_logger().info("✅ 'stop_positions.csv' 파일이 저장되었습니다.")
            
            plt.figure(figsize=(10, 10))
            wp2_data = [d for d in self.stop_position_data if d['wp'] == 2]
            if wp2_data:
                plt.scatter([d['x_pos_mm'] for d in wp2_data], [d['y_pos_mm'] for d in wp2_data],
                            marker='o', color='blue', label='WP2 (Tag 13)', alpha=0.7, edgecolors='k')
            wp5_data = [d for d in self.stop_position_data if d['wp'] == 5]
            if wp5_data:
                plt.scatter([d['x_pos_mm'] for d in wp5_data], [d['y_pos_mm'] for d in wp5_data],
                            marker='^', color='green', label='WP5 (Tag 0)', alpha=0.7, edgecolors='k')
            
            plt.title('Stop Position Accuracy at WP2 & WP5')
            plt.xlabel('X Position (mm)')
            plt.ylabel('Y Position (mm)')
            plt.axhline(0, color='grey', lw=0.5)
            plt.axvline(0, color='grey', lw=0.5)
            plt.grid(True, linestyle='--', alpha=0.6)
            plt.gca().set_aspect('equal', adjustable='box')
            all_x = [d['x_pos_mm'] for d in self.stop_position_data]
            all_y = [d['y_pos_mm'] for d in self.stop_position_data]
            max_val = self.STOP_TOLERANCE_MM
            if all_x and all_y:
                max_val = max(max(map(abs, all_x)), max(map(abs, all_y)), self.STOP_TOLERANCE_MM)
            plt.xlim(-max_val * 1.2, max_val * 1.2)
            plt.ylim(-max_val * 1.2, max_val * 1.2)
            circle = plt.Circle((0, 0), self.STOP_TOLERANCE_MM, color='r', fill=False, linestyle='--', label=f'Tolerance ({self.STOP_TOLERANCE_MM}mm)')
            plt.gca().add_artist(circle)
            plt.legend()
            plt.savefig('stop_positions_scatter.png')
            plt.close()
            self.get_logger().info("✅ 'stop_positions_scatter.png' 파일이 저장되었습니다.")

        if self.path_y_position_data:
            with open('path_y_positions.csv', 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=self.path_y_position_data[0].keys())
                writer.writeheader()
                writer.writerows(self.path_y_position_data)
            self.get_logger().info("✅ 'path_y_positions.csv' 파일이 저장되었습니다.")
            
            tag_ids = [d['tag_id'] for d in self.path_y_position_data]
            y_vals = [d['y_pos_mm'] for d in self.path_y_position_data]
            colors = np.arange(len(self.path_y_position_data))
            plt.figure(figsize=(12, 8))
            scatter = plt.scatter(tag_ids, y_vals, c=colors, cmap='cividis', alpha=0.7, edgecolors='k')
            plt.colorbar(scatter, label='Time Progression (Early to Late)')
            plt.title('Path Following Accuracy (Y-position at each Tag)')
            plt.xlabel('Tag ID')
            plt.ylabel('Y Position (mm)')
            plt.axhline(0, color='black', lw=1)
            plt.axhline(self.PATH_Y_TOLERANCE_MM, color='r', linestyle='--', label=f'Tolerance (+{self.PATH_Y_TOLERANCE_MM}mm)')
            plt.axhline(-self.PATH_Y_TOLERANCE_MM, color='r', linestyle='--', label=f'Tolerance (-{self.PATH_Y_TOLERANCE_MM}mm)')
            plt.xticks(sorted(list(self.PATH_TARGET_TAG_IDS)))
            plt.grid(True, linestyle='--', alpha=0.6, axis='y')
            plt.legend()
            plt.savefig('path_y_positions_scatter.png')
            plt.close()
            self.get_logger().info("✅ 'path_y_positions_scatter.png' 파일이 저장되었습니다.")

def main():
    rclpy.init()
    node = AgvPerformanceTester()

    waypoints = [
        make_pose(1.947, 0.008, -1.639),
        make_pose(1.846, -6.978, 3.130),
        make_pose(-0.010, -6.948, 0.029),
        make_pose(1.846, -6.978, 1.559),
        make_pose(1.947, 0.008, 3.074),
        make_pose(0.0, 0.0, 0.0)
    ]

    try:
        repeat_count = int(input(">> 왕복 실험 횟수를 입력하세요: "))
        if repeat_count <= 0: raise ValueError
    except ValueError:
        node.get_logger().error("잘못된 입력입니다. 1 이상의 정수를 입력해야 합니다.")
        rclpy.shutdown()
        return

    try:
        if not node.perform_origin_alignment():
            raise KeyboardInterrupt
        node.reset_localization_to_origin()

        for i in range(repeat_count):
            node.get_logger().info(f"\n★★★★★ 왕복 {i+1}/{repeat_count} 시작 ★★★★★")
            success = True
            for wp_idx, pose in enumerate(waypoints):
                success = node.go_to_pose(pose, wp_idx)
                if not success:
                    node.get_logger().error(f"WP-{wp_idx} 이동 실패! 현재 랩을 중단합니다.")
                    break
            if not success:
                break
            node.get_logger().info(f"🎉 왕복 {i+1}회 완료! 🎉")

    except KeyboardInterrupt:
        node.get_logger().info("\n사용자에 의해 테스트가 중단되었습니다.")
    except Exception as e:
        node.get_logger().error(f"테스트 중 예외 발생: {e}")
    finally:
        node.set_goal_tolerance(node.DEFAULT_XY_TOL)
        node.save_and_plot_results()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()