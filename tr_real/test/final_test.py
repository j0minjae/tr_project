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
        # 액션 클라이언트
        self._ac_navigate = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        # 구독, 발행
        self.pgv_sub = self.create_subscription(PgvData, '/pgv', self.pgv_callback, 10)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 테스트 설정값 및 결과 변수
        # [수정] 측정할 웨이포인트와 해당 태그 ID를 매핑
        self.WP_TO_TAG_MAP = {
            0: 16,
            1: 17,
            2: 18,
            3: 17,
            4: 16,
            5: 15
        }
        self.STOP_TOLERANCE_MM = 10.0
        self.PATH_TARGET_TAG_IDS = {19}
        self.PATH_Y_TOLERANCE_MM = 30.0
        self.is_monitoring_path = False
        self.last_pgv_msg = None
        
        # [제거] 오프셋 관련 변수 제거
        # self.POSITION_OFFSETS_M = ...

        # 데이터 저장을 위한 변수 초기화
        self.stop_position_data = []
        self.path_y_position_data = []
        self.current_path_readings = {} # 주행 중 y 값 임시 저장소

        # 서버 연결 대기
        self._ac_navigate.wait_for_server()
        self.get_logger().info("✅ 액션 서버가 준비되었습니다.")

    def pgv_callback(self, msg: PgvData):
        self.last_pgv_msg = msg
        if not msg.tag_detected or not self.is_monitoring_path:
            return

        if msg.tag_id in self.PATH_TARGET_TAG_IDS:
            if msg.tag_id not in self.current_path_readings:
                self.current_path_readings[msg.tag_id] = []
            self.current_path_readings[msg.tag_id].append(msg.y_pos)
            
            y_abs_mm = abs(msg.y_pos)
            is_within_tolerance = y_abs_mm <= self.PATH_Y_TOLERANCE_MM
            if not is_within_tolerance:
                self.get_logger().warn(f"🚩 경로 이탈 감지! Tag ID: {msg.tag_id}, Y-Offset: {msg.y_pos:.2f} mm")

    def perform_origin_alignment(self) -> bool:
        self.get_logger().info("\n" + "="*60)
        # 시작점 정렬은 Tag 15를 기준으로 하므로 이 부분은 유지합니다.
        self.get_logger().info("🚦 원점 정렬을 시작합니다. 로봇을 Tag ID 15 위에 위치시켜주세요.")
        self.get_logger().info(f"목표: 태그 중심으로부터 거리 {self.STOP_TOLERANCE_MM:.1f} mm 이내")
        self.get_logger().info("정렬이 완료되면 'y'를 입력하여 원점을 설정합니다. (중단: Ctrl+C)")
        self.get_logger().info("="*60)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.last_pgv_msg is None:
                self.get_logger().warn("PGV 데이터를 수신 대기중...", throttle_duration_sec=2)
                continue
            # Tag 15 (WP 5)를 기준으로 원점을 잡습니다.
            if not self.last_pgv_msg.tag_detected or self.last_pgv_msg.tag_id != self.WP_TO_TAG_MAP[5]:
                print(f"\r\033[K[❌] Tag 15을 찾고 있습니다... (감지 플래그: {self.last_pgv_msg.tag_detected}, ID: {self.last_pgv_msg.tag_id})", end="")
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

        # 안정적인 측정을 위해 잠시 대기
        time.sleep(0.8) # 기존보다 약간 늘려 안정성 확보
        self.get_logger().info("  - 최신 데이터 수신 대기를 시작합니다.")

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

    # [제거] 정밀 보정 기동 함수 제거
    # def perform_correction_maneuver(...): ...

    def go_to_pose(self, pose: PoseStamped, wp_idx: int) -> bool:
        self.get_logger().info(f"WP-{wp_idx} [{pose.pose.position.x:.3f}, {pose.pose.position.y:.3f}] (으)로 이동 시작...")
        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.current_path_readings.clear()
        self.is_monitoring_path = True
        
        future = self._ac_navigate.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("목표가 거부되었습니다.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        status = result_future.result().status
        self.is_monitoring_path = False
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"✅ WP-{wp_idx} 도착 성공!")

            for tag_id, y_readings in self.current_path_readings.items():
                if y_readings:
                    avg_y_pos = np.mean(y_readings)
                    self.path_y_position_data.append({
                        "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f"),
                        "tag_id": tag_id,
                        "y_pos_mm": avg_y_pos
                    })

            # [수정] 정밀 정렬 로직을 제거하고, WP_TO_TAG_MAP에 있는 모든 지점에서 측정
            if wp_idx in self.WP_TO_TAG_MAP:
                self.get_logger().info(f"  - WP-{wp_idx}는 측정 지점입니다. 정지 정확도를 측정합니다.")
                self.measure_stopping_accuracy(wp_idx)
            
            return True
        else:
            self.get_logger().error(f"❌ WP-{wp_idx} 도착 실패 (상태: {status})")
            return False

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

    def save_and_plot_results(self):
        self.get_logger().info("\n" + "="*60 + "\n📊 데이터 저장 및 시각화 중... 📊\n" + "="*60)
        if self.stop_position_data:
            with open('stop_positions.csv', 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=self.stop_position_data[0].keys())
                writer.writeheader()
                writer.writerows(self.stop_position_data)
            self.get_logger().info("✅ 'stop_positions.csv' 파일이 저장되었습니다.")
            
            # [수정] 모든 측정 지점을 그래프에 동적으로 표시
            plt.figure(figsize=(10, 10))
            
            plot_styles = {
                0: {'marker': 'o', 'color': '#ff7f0e', 'label': f'WP0 (Tag 16)'},
                1: {'marker': 's', 'color': '#2ca02c', 'label': f'WP1 (Tag 17)'},
                2: {'marker': 'D', 'color': '#d62728', 'label': f'WP2 (Tag 18)'},
                3: {'marker': 's', 'color': '#9467bd', 'label': f'WP3 (Tag 17)'},
                4: {'marker': 'o', 'color': '#8c564b', 'label': f'WP4 (Tag 16)'},
                5: {'marker': '^', 'color': '#1f77b4', 'label': f'WP5 (Tag 15)'}
            }
            
            waypoints_in_data = sorted(list(set(d['wp'] for d in self.stop_position_data)))

            for wp_idx in waypoints_in_data:
                wp_data = [d for d in self.stop_position_data if d['wp'] == wp_idx]
                if wp_data:
                    style = plot_styles.get(wp_idx, {'marker': 'x', 'color': 'black', 'label': f'WP{wp_idx}'})
                    plt.scatter(
                        [d['x_pos_mm'] for d in wp_data],
                        [d['y_pos_mm'] for d in wp_data],
                        marker=style['marker'],
                        color=style['color'],
                        label=style['label'],
                        alpha=0.7,
                        edgecolors='k',
                        s=80 # 마커 크기
                    )

            plt.title('Stop Position Accuracy at Waypoints')
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
            
            y_vals = [d['y_pos_mm'] for d in self.path_y_position_data]
            plt.figure(figsize=(12, 8))
            plt.scatter(range(len(y_vals)), y_vals, alpha=0.7, edgecolors='k')
            plt.title('Path Following Accuracy (Y-position at each Tag)')
            plt.xlabel('Measurement Sequence')
            plt.ylabel('Average Y Position (mm)')
            plt.axhline(0, color='black', lw=1)
            plt.axhline(self.PATH_Y_TOLERANCE_MM, color='r', linestyle='--', label=f'Tolerance (+{self.PATH_Y_TOLERANCE_MM}mm)')
            plt.axhline(-self.PATH_Y_TOLERANCE_MM, color='r', linestyle='--', label=f'Tolerance (-{self.PATH_Y_TOLERANCE_MM}mm)')
            plt.grid(True, linestyle='--', alpha=0.6, axis='y')
            plt.legend()
            plt.savefig('path_y_positions_scatter.png')
            plt.close()
            self.get_logger().info("✅ 'path_y_positions_scatter.png' 파일이 저장되었습니다.")

def main():
    rclpy.init()
    node = AgvPerformanceTester()

    waypoints = [
        make_pose(2.0+0.04882, 0.0-0.05571+0.03884, 0.0),
        make_pose(2.0+0.04882, -7.0-0.04464+0.03588, -1.571),
        make_pose(0.0-0.03495, -7.0-0.04464, -3.141),
        make_pose(2.0-0.0069, -7.0-0.04464+0.07255, 0.0),
        make_pose(2.0-0.0069, 0.0-0.05571-0.013, 1.571),
        make_pose(0.0+0.01235, 0.0-0.05571, 3.141)
    ]
    
    lap_times = []

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
            lap_start_time = time.time()
            success = True
            
            for wp_idx, pose in enumerate(waypoints):
                success = node.go_to_pose(pose, wp_idx)
                if not success:
                    node.get_logger().error(f"WP-{wp_idx} 이동 실패! 현재 랩을 중단합니다.")
                    break
            
            if not success:
                break
            
            lap_end_time = time.time()
            lap_duration = lap_end_time - lap_start_time
            lap_times.append(lap_duration)
            node.get_logger().info(f"🎉 왕복 {i+1}회 완료! (소요 시간: {lap_duration:.2f}초) 🎉")

    except KeyboardInterrupt:
        node.get_logger().info("\n사용자에 의해 테스트가 중단되었습니다.")
    except Exception as e:
        node.get_logger().error(f"테스트 중 예외 발생: {e}")
    finally:
        if lap_times:
            avg_lap_time = np.mean(lap_times)
            node.get_logger().info("\n" + "="*60)
            node.get_logger().info(f"📊 최종 결과: 평균 왕복 소요 시간: {avg_lap_time:.2f}초")
            node.get_logger().info("="*60)
            
        node.save_and_plot_results()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()