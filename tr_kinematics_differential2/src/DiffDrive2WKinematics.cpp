/*********************************************************************
 * (Copyright 내용은 기존과 동일하여 생략)
 *********************************************************************/

#include "DiffDrive2WKinematics.h"
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

DiffDrive2WKinematics::DiffDrive2WKinematics() {}

// 함수 정의를 헤더파일과 일치시킵니다.
void DiffDrive2WKinematics::execForwKin(const std::shared_ptr<const sensor_msgs::msg::JointState>& js, nav_msgs::msg::Odometry& odom)
{
	const double vel_x = 0.5 * (js->velocity[0] + js->velocity[1]) * (m_dDiam * 0.5);
	const double yaw_rate = -(js->velocity[0] - js->velocity[1]) * (m_dDiam * 0.5) / m_dAxisLength;
	if(m_curr_odom.header.stamp.sec != 0) // 초기값이 0이 아닐 때만 계산하도록 조건 수정
	{
		const double t_js = rclcpp::Time(js->header.stamp).seconds();
		const double t_odom = rclcpp::Time(m_curr_odom.header.stamp).seconds();
		const double dt = t_js - t_odom;

		const double vel_x_mid = 0.5 * (vel_x + m_curr_odom.twist.twist.linear.x);
		const double yaw_rate_mid = 0.5 * (yaw_rate + m_curr_odom.twist.twist.angular.z);
		const double phi_mid = m_phiAbs + yaw_rate_mid * dt * 0.5;
		m_curr_odom.pose.pose.position.x += vel_x_mid * dt * cos(phi_mid);
		m_curr_odom.pose.pose.position.y += vel_x_mid * dt * sin(phi_mid);
		m_curr_odom.pose.pose.position.z = 0;
		m_phiAbs += yaw_rate_mid * dt;
		tf2::Quaternion q;
		q.setRPY(0, 0, m_phiAbs);
		m_curr_odom.pose.pose.orientation = tf2::toMsg(q);
	}
	m_curr_odom.header.stamp = js->header.stamp;
	m_curr_odom.header.frame_id = odom.header.frame_id;
	m_curr_odom.child_frame_id = odom.child_frame_id;
	m_curr_odom.twist.twist.linear.x = vel_x;
	m_curr_odom.twist.twist.angular.z = yaw_rate;
	odom = m_curr_odom;
}

// 함수 정의를 헤더파일과 일치시킵니다.
void DiffDrive2WKinematics::execInvKin(const std::shared_ptr<const geometry_msgs::msg::Twist>& twist, trajectory_msgs::msg::JointTrajectory& traj)
{
    const double w_left  = (twist->linear.x - 0.5 * twist->angular.z * m_dAxisLength) * 2.0 / m_dDiam;
    const double w_right = (twist->linear.x + 0.5 * twist->angular.z * m_dAxisLength) * 2.0 / m_dDiam;
    
    traj.points.resize(1); // Point 배열 크기를 1로 설정
    traj.points[0].velocities.resize(2);
    traj.points[0].velocities[0] = w_left;
    traj.points[0].velocities[1] = w_right;
    traj.points[0].time_from_start = rclcpp::Duration::from_seconds(0.1);
}

// 함수 정의를 헤더파일과 일치시킵니다.
void DiffDrive2WKinematics::execInvKin(const std::shared_ptr<const geometry_msgs::msg::Twist>& twist, std_msgs::msg::Float64MultiArray& VelCmd)
{
    const double w_left  = (twist->linear.x - 0.5 * twist->angular.z * m_dAxisLength) * 2.0 / m_dDiam;
    const double w_right = (twist->linear.x + 0.5 * twist->angular.z * m_dAxisLength) * 2.0 / m_dDiam;

    VelCmd.data.clear();
    VelCmd.data.push_back(w_left);
    VelCmd.data.push_back(w_right);
}

void DiffDrive2WKinematics::setAxisLength(double dLength) { m_dAxisLength = dLength; }
void DiffDrive2WKinematics::setWheelDiameter(double dDiam) { m_dDiam = dDiam; }