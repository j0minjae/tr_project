/*********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Neobotix GmbH
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * * Neither the name of the Neobotix nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "Kinematics.h"
#include "DiffDrive2WKinematics.h"
#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/buffer.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <mutex>

using std::placeholders::_1;

class PlatformCtrlNode: public rclcpp::Node
{
public:
	PlatformCtrlNode(): Node("tr_differential_node"), last_cmd_vel_time_(0, 0, this->get_clock()->get_clock_type())
    {
        current_joint_state_.name = {"wheel_left_joint", "wheel_right_joint"};
        current_joint_state_.position = {0.0, 0.0};
        current_joint_state_.velocity = {0.0, 0.0};
    }

	int init() {
		this->declare_parameter<double>("wheelDiameter", 0.3);
		this->declare_parameter<double>("wheelSeperation", 0.5);
		this->declare_parameter<std::string>("odomFrame", "odom");
		this->declare_parameter<std::string>("robotBaseFrame", "base_footprint");
		this->declare_parameter<std::string>("joint_states_topic", "drives/joint_states");
        this->declare_parameter<std::string>("joint_trajectory_topic", "drives/joint_trajectory");
        this->declare_parameter<bool>("use_for_simulation", false);

		this->get_parameter("wheelDiameter", wheelDiameter);
		this->get_parameter("wheelSeperation", axisLength);
		this->get_parameter("odomFrame", odomFrame);
		this->get_parameter("robotBaseFrame", robotBaseFrame);
		this->get_parameter("joint_states_topic", joint_states_topic);
        this->get_parameter("use_for_simulation", use_for_simulation_);

		topicPub_Odometry = this->create_publisher<nav_msgs::msg::Odometry>("odom1", 10);
        
        if (use_for_simulation_) {
            topicPub_VelocityCommands = this->create_publisher<std_msgs::msg::Float64MultiArray>("velocity_controller/commands", 10);
        } else {
            this->get_parameter("joint_trajectory_topic", joint_trajectory_topic);
            topicPub_TrajectoryCommands = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(joint_trajectory_topic, 10);
        }

		topicSub_ComVel = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1, std::bind(&PlatformCtrlNode::receiveCmd, this, _1));
		topicSub_DriveState = this->create_subscription<sensor_msgs::msg::JointState>(joint_states_topic, 10, std::bind(&PlatformCtrlNode::receiveOdo, this, _1));
		odom_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

		DiffDrive2WKinematics* diffKin = new DiffDrive2WKinematics();
		diffKin->setWheelDiameter(wheelDiameter);
		diffKin->setAxisLength(axisLength);
		kin = diffKin;

        drive_command_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&PlatformCtrlNode::publishDriveCommand, this)
        );

		RCLCPP_INFO(this->get_logger(), "Kinematics node initialized.");
		RCLCPP_INFO(this->get_logger(), "Simulation mode: %s", use_for_simulation_ ? "true" : "false");
		return 0;
	}

	void receiveCmd(const geometry_msgs::msg::Twist::SharedPtr twist) {
        std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
		last_twist_ = *twist;
        last_cmd_vel_time_ = this->get_clock()->now();
	}

	void receiveOdo(const sensor_msgs::msg::JointState::SharedPtr js) {
        {
            std::lock_guard<std::mutex> lock(joint_state_mutex_);
            current_joint_state_ = *js;
        }

		nav_msgs::msg::Odometry odom;
		odom.header.frame_id = odomFrame;
		odom.child_frame_id = robotBaseFrame;
		kin->execForwKin(js, odom);
		topicPub_Odometry->publish(odom);

		if(sendTransform) {
			geometry_msgs::msg::TransformStamped odom_trans;
			odom_trans.header.stamp = odom.header.stamp;
            odom_trans.header.frame_id = odomFrame;
            odom_trans.child_frame_id = robotBaseFrame;
			odom_trans.transform.translation.x = odom.pose.pose.position.x;
			odom_trans.transform.translation.y = odom.pose.pose.position.y;
			odom_trans.transform.translation.z = odom.pose.pose.position.z;
			odom_trans.transform.rotation = odom.pose.pose.orientation;
			// odom_broadcaster->sendTransform(odom_trans);
		}
	}

private:
    void publishDriveCommand() {
        if (last_cmd_vel_time_.seconds() == 0.0) {
            return;
        }
        if ((this->get_clock()->now() - last_cmd_vel_time_).seconds() > 0.2) {
            std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
            last_twist_.linear.x = 0.0;
            last_twist_.angular.z = 0.0;
        }

        auto twist_ptr = std::make_shared<const geometry_msgs::msg::Twist>(last_twist_);

        if (use_for_simulation_) {
            std_msgs::msg::Float64MultiArray vel_cmd;
            kin->execInvKin(twist_ptr, vel_cmd);
            topicPub_VelocityCommands->publish(vel_cmd);
        } else {
            trajectory_msgs::msg::JointTrajectory traj;
            traj.header.stamp = this->get_clock()->now();
            
            // execInvKin 함수에 현재 조인트 상태를 함께 전달합니다.
            {
                std::lock_guard<std::mutex> lock(joint_state_mutex_);
                kin->execInvKin(twist_ptr, traj);
            }
            
            topicPub_TrajectoryCommands->publish(traj);
        }
    }

	Kinematics* kin = 0;
	bool sendTransform = true;
    bool use_for_simulation_ = false;

	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr topicPub_Odometry;
	rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr topicPub_TrajectoryCommands;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr topicPub_VelocityCommands;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr topicSub_ComVel;
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topicSub_DriveState;
	std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster;

	double wheelDiameter = 0.0;
	double axisLength = 0.0;

	std::string odomFrame;
	std::string robotBaseFrame;
    std::string joint_trajectory_topic;
    std::string joint_states_topic;

    rclcpp::TimerBase::SharedPtr drive_command_timer_;
    geometry_msgs::msg::Twist last_twist_;
    rclcpp::Time last_cmd_vel_time_;
    sensor_msgs::msg::JointState current_joint_state_;
    std::mutex joint_state_mutex_;
    std::mutex cmd_vel_mutex_;
};

int main (int argc, char** argv)
{
	rclcpp::init(argc, argv);
	auto nh = std::make_shared<PlatformCtrlNode>();
	if(nh->init() != 0) {
		RCLCPP_ERROR_STREAM(nh->get_logger(),"tr_kinematics_differential_node: init failed!");
	}
	rclcpp::spin(nh);
	return 0;
}