/*********************************************************************
 * (Copyright 내용은 기존과 동일하여 생략)
 *********************************************************************/

#ifndef tr_kinematics_h_
#define tr_kinematics_h_

#include "rclcpp/rclcpp.hpp"

// Forward Declarations
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class Kinematics
{
public:
	// 포인터 타입을 const std::shared_ptr<const ...>& 로 통일합니다.
	virtual void execForwKin(const std::shared_ptr<const sensor_msgs::msg::JointState>& js, nav_msgs::msg::Odometry& odom) = 0;
	virtual void execInvKin(const std::shared_ptr<const geometry_msgs::msg::Twist>& Twist, trajectory_msgs::msg::JointTrajectory& Jt) = 0;
	virtual void execInvKin(const std::shared_ptr<const geometry_msgs::msg::Twist>& Twist, std_msgs::msg::Float64MultiArray& VelCmd) = 0;
};

#endif //tr_kinematics_h_