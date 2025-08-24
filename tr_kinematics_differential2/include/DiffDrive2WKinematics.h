/*********************************************************************
 * (Copyright 내용은 기존과 동일하여 생략)
 *********************************************************************/

#ifndef tr_diffdrivekinematics_h_
#define tr_diffdrivekinematics_h_

#include "Kinematics.h"

// 필요한 메시지 헤더들을 모두 포함시킵니다.
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class DiffDrive2WKinematics : public Kinematics
{
public:
	DiffDrive2WKinematics();

	// 부모 클래스의 인터페이스와 정확히 일치하도록 수정합니다.
	void execForwKin(const std::shared_ptr<const sensor_msgs::msg::JointState>& js, nav_msgs::msg::Odometry& odom) override;
	void execInvKin(const std::shared_ptr<const geometry_msgs::msg::Twist>& twist, trajectory_msgs::msg::JointTrajectory& traj) override;
	void execInvKin(const std::shared_ptr<const geometry_msgs::msg::Twist>& twist, std_msgs::msg::Float64MultiArray& VelCmd) override;

	void setAxisLength(double dLength);
	void setWheelDiameter(double dDiam);

private:
	double m_phiAbs = 0;
	nav_msgs::msg::Odometry m_curr_odom;

	double m_dAxisLength = 0;
	double m_dDiam = 0;
};

#endif //tr_diffdrivekinematics_h_