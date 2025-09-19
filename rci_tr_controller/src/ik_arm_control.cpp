#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <Eigen/Dense>
#include <iostream>
#include <thread>
#include <vector>
#include <string>

class DualArmIKController : public rclcpp::Node
{
public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

    DualArmIKController(const std::string& urdf_path)
    : Node("dual_arm_action_client")
    {
        left_arm_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this, "/left_arm_controller/follow_joint_trajectory");

        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&DualArmIKController::joint_state_callback, this, std::placeholders::_1));

        pinocchio::urdf::buildModel(urdf_path, model_);
        data_ = pinocchio::Data(model_);
        full_joint_positions_ = pinocchio::neutral(model_);

        const std::string ee_frame_name = "openarm_left_hand";
        ee_frame_id_ = model_.getFrameId(ee_frame_name);

        for (int i = 1; i <= 7; ++i) {
            joint_names_.push_back("openarm_left_joint" + std::to_string(i));
        }

        for (const auto& name : joint_names_) {
            const auto joint_id = model_.getJointId(name);
            q_indices_.push_back(model_.joints[joint_id].idx_q());
            v_indices_.push_back(model_.joints[joint_id].idx_v());
        }

        RCLCPP_INFO(this->get_logger(), "Waiting for action servers...");
        if (!left_arm_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Action servers not available!");
            throw std::runtime_error("Could not connect to action servers.");
        }
        RCLCPP_INFO(this->get_logger(), "Dual Arm IK Controller initialized successfully.");
    }

    void do_move(double duration = 2.0)
    {
        Eigen::Vector3d target_position(0.616, 0.154, 0.570);
        double roll = -1.306, pitch = -1.568, yaw = -1.836;
        Eigen::Matrix3d target_rotation = pinocchio::rpy::rpyToMatrix(roll, pitch, yaw);

        pinocchio::SE3 target_placement(target_rotation, target_position);

        Eigen::VectorXd joint_positions = ik_solver(target_placement, full_joint_positions_);

        if (joint_positions.size() > 0) {
            RCLCPP_INFO(this->get_logger(), "IK solution found.");
            send_goal(joint_positions, duration);
            RCLCPP_INFO(this->get_logger(), "Goal sent to the arm.");
        } else {
            RCLCPP_WARN(this->get_logger(), "IK solution not found. Arm will not move.");
        }
    }

    void do_home()
    {
        RCLCPP_INFO(this->get_logger(), "--- Moving arm to HOME ---");
        Eigen::VectorXd joint_positions(7);
        joint_positions << 0.0, 0.0, 0.0, 1.57, 0.0, 0.0, 0.0;
        send_goal(joint_positions, 5.0);
    }

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (model_.existJointName(msg->name[i])) {
                auto joint_id = model_.getJointId(msg->name[i]);
                if (joint_id < model_.joints.size()) {
                    auto q_index = model_.joints[joint_id].idx_q();
                    full_joint_positions_[q_index] = msg->position[i];
                }
            }
        }
    }

    Eigen::MatrixXd get_jacobian(const Eigen::VectorXd& q)
    {
        pinocchio::computeFrameJacobian(model_, data_, q, ee_frame_id_, pinocchio::LOCAL, jacobian_);
        Eigen::MatrixXd arm_J(6, v_indices_.size());
        for (size_t i = 0; i < v_indices_.size(); ++i) {
            arm_J.col(i) = jacobian_.col(v_indices_[i]);
        }
        return arm_J;
    }

    Eigen::VectorXd ik_solver(const pinocchio::SE3& target_placement, const Eigen::VectorXd& initial_q_full)
    {
        Eigen::VectorXd q_full = initial_q_full;
        const int max_iterations = 100;
        const double damping = 1e-4;
        const double tolerance = 1e-3;
        const double gain = 0.5;

        for (int i = 0; i < max_iterations; ++i) {
            pinocchio::forwardKinematics(model_, data_, q_full);
            pinocchio::updateFramePlacements(model_, data_);
            const pinocchio::SE3& ee_placement = data_.oMf[ee_frame_id_];

            Eigen::Matrix<double, 6, 1> error = pinocchio::log6(ee_placement.inverse() * target_placement).toVector();

            if (error.norm() < tolerance) {
                RCLCPP_INFO(this->get_logger(), "Target for arm reached in %d iterations.", i);
                Eigen::VectorXd result(q_indices_.size());
                for(size_t j = 0; j < q_indices_.size(); ++j) {
                    result[j] = q_full[q_indices_[j]];
                }
                return result;
            }

            Eigen::MatrixXd J_arm = get_jacobian(q_full);
            Eigen::MatrixXd J_pinv = J_arm.transpose() * (J_arm * J_arm.transpose() + damping * Eigen::MatrixXd::Identity(6, 6)).inverse();
            Eigen::VectorXd dq_arm = J_pinv * error;

            Eigen::VectorXd dq_full = Eigen::VectorXd::Zero(model_.nv);
            for (size_t j = 0; j < v_indices_.size(); ++j) {
                dq_full[v_indices_[j]] = dq_arm[j];
            }

            q_full = pinocchio::integrate(model_, q_full, gain * dq_full);
        }

        RCLCPP_WARN(this->get_logger(), "IK solver for arm could not reach the target.");
        return Eigen::VectorXd();
    }

    void send_goal(const Eigen::VectorXd& positions, double duration)
    {
        auto goal_msg = FollowJointTrajectory::Goal();
        trajectory_msgs::msg::JointTrajectory trajectory;
        trajectory.joint_names = joint_names_;

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions.assign(positions.data(), positions.data() + positions.size());
        point.time_from_start = rclcpp::Duration::from_seconds(duration);

        trajectory.points.push_back(point);
        goal_msg.trajectory = trajectory;

        left_arm_client_->async_send_goal(goal_msg);
    }

    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr left_arm_client_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    pinocchio::Model model_;
    pinocchio::Data data_;
    Eigen::VectorXd full_joint_positions_;
    std::mutex joint_state_mutex_;
    pinocchio::FrameIndex ee_frame_id_;
    std::vector<std::string> joint_names_;
    std::vector<pinocchio::JointIndex> q_indices_, v_indices_;
    Eigen::MatrixXd jacobian_;
};

void print_usage() {
    std::cout << "Usage: " << std::endl;
    std::cout << " move - Move the arm to a predefined pose" << std::endl;
    std::cout << " home - Move the arm to the home position" << std::endl;
    std::cout << " quit - Exit the program" << std::endl;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    const std::string urdf_path = "/home/home/tr_project/src/tr_description/urdf/amr_sim.urdf";
    auto ik_controller = std::make_shared<DualArmIKController>(urdf_path);

    std::thread ros_thread([&]() {
        rclcpp::spin(ik_controller);
    });

    print_usage();
    std::string command;
    while (rclcpp::ok()) {
        std::cout << "> ";
        std::cin >> command;
        if (command == "move") {
            ik_controller->do_move();
        } else if (command == "home") {
            ik_controller->do_home();
        } else if (command == "quit") {
            break;
        } else {
            print_usage();
        }
    }

    rclcpp::shutdown();
    ros_thread.join();
    return 0;
}
