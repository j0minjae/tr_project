import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf_transformations

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        # 1초 후에 publish_pose 함수를 한 번 실행
        self.timer = self.create_timer(1.0, self.publish_pose)

    def publish_pose(self):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        
        # 원하는 초기 위치 설정
        pose_msg.pose.pose.position.x = 0.0
        pose_msg.pose.pose.position.y = 0.0
        
        # Yaw (radian) -> Quaternion 변환
        yaw_angle = 0.0 
        quat = tf_transformations.quaternion_from_euler(0, 0, yaw_angle)
        pose_msg.pose.pose.orientation.x = quat[0]
        pose_msg.pose.pose.orientation.y = quat[1]
        pose_msg.pose.pose.orientation.z = quat[2]
        pose_msg.pose.pose.orientation.w = quat[3]

        # 분산(Covariance) 설정 (값이 작을수록 해당 위치를 더 확신)
        pose_msg.pose.covariance[0] = 0.25  # x-x variance
        pose_msg.pose.covariance[7] = 0.25  # y-y variance
        pose_msg.pose.covariance[35] = 0.0685  # yaw-yaw variance

        self.get_logger().info('Publishing initial pose...')
        self.publisher_.publish(pose_msg)
        
        # 메시지를 보낸 후 노드를 종료
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()