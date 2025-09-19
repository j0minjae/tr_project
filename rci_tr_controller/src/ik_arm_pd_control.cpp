// To use this controller, you need to switch the controller in 'control.yaml' to an effort-based controller.
// For example, change 'joint_trajectory_controller/JointTrajectoryController' to 'effort_controllers/JointEffortController'.
// You will also need to configure the gains for the effort controller in your URDF or a separate YAML file.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
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

class DualArmPDController : public rclcpp::Node
{
public:
    DualArmPDController(const std::string& urdf_path)
    : Node("dual_arm_pd_controller")
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/left_arm_effort_controller/command", 10);

        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&DualArmPDController::joint_state_callback, this, std::placeholders::_1));

        pinocchio::urdf::buildModel(urdf_path, model_);
        data_ = pinocchio::Data(model_);
        full_joint_positions_ = pinocchio::neutral(model_);
        current_q_.resize(7);
        current_v_.resize(7);

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

        RCLCPP_INFO(this->get_logger(), "Dual Arm PD Controller initialized successfully.");
    }

    void do_move()
    {
        //1. define x_des, x_dot_des
        Eigen::Vector3d target_position(0.616, 0.154, 0.570);
        double roll = -1.306, pitch = -1.568, yaw = -1.836;
        Eigen::Matrix3d target_rotation = pinocchio::rpy::rpyToMatrix(roll, pitch, yaw);
        pinocchio::SE3 target_placement(target_rotation, target_position);

        //2. get M_dot, C_dot, G_dot, J_ee with Pinocchio
        M, C, G = get_MCG
        J_arm, J_adot = get_jacobian

        //  2-1 get x_now, x_dot_now from /joint_states
        x_now vel_now, q_fulldot= get_fk()
        error_x, error_vel

        //3. calculate x_twodot_des with PD제어기
        accel_des = pd_controller(error_x, error_vel)

        //4. generate Torque 
        effort = generate_effort(accel_des, J_arm, J_adot, q_fulldot)





        Eigen::VectorXd target_q = ik_solver(target_placement, full_joint_positions_);

        if (target_q.size() == 0) {
            RCLCPP_WARN(this->get_logger(), "IK solution not found. Arm will not move.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "IK solution found. Starting PD control.");
        
        //PD control
        double Kp = 10.0, Kd = 2 * sqrt(Kp);
        rclcpp::Rate rate(100); // 100 Hz
        auto start_time = this->get_clock()->now();
        while (rclcpp::ok() && (this->get_clock()->now() - start_time).seconds() < 5.0) {
            std_msgs::msg::Float64MultiArray effort_msg;
            effort_msg.data.resize(7);

            Eigen::VectorXd error_q = target_q - current_q_;
            Eigen::VectorXd effort = Kp * error_q - Kd * current_v_;

            for (int i = 0; i < 7; ++i) {
                effort_msg.data[i] = effort[i];
            }
            publisher_->publish(effort_msg);
            rate.sleep();
        }
        RCLCPP_INFO(this->get_logger(), "PD control finished.");
    }

    void do_home()
    {
        RCLCPP_INFO(this->get_logger(), "--- Moving arm to HOME ---");
        Eigen::VectorXd target_q(7);
        target_q << 0.0, 0.0, 0.0, 1.57, 0.0, 0.0, 0.0;

        double Kp = 10.0, Kd = 0.1;
        rclcpp::Rate rate(100);
        auto start_time = this->get_clock()->now();
        while (rclcpp::ok() && (this->get_clock()->now() - start_time).seconds() < 5.0) {
            std_msgs::msg::Float64MultiArray effort_msg;
            effort_msg.data.resize(7);

            Eigen::VectorXd error_q = target_q - current_q_;
            Eigen::VectorXd effort = Kp * error_q - Kd * current_v_;

            for (int i = 0; i < 7; ++i) {
                effort_msg.data[i] = effort[i];
            }
            publisher_->publish(effort_msg);
            rate.sleep();
        }
        RCLCPP_INFO(this->get_logger(), "PD control finished.");
    }

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        for (size_t i = 0; i < msg->name.size(); ++i) {
            for(size_t j = 0; j < joint_names_.size(); ++j) {
                if (msg->name[i] == joint_names_[j]) {
                    current_q_[j] = msg->position[i];
                    current_v_[j] = msg->velocity[i];
                    break;
                }
            }

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

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    pinocchio::Model model_;
    pinocchio::Data data_;
    Eigen::VectorXd full_joint_positions_;
    Eigen::VectorXd current_q_, current_v_;
    std::mutex joint_state_mutex_;
    pinocchio::FrameIndex ee_frame_id_;
    std::vector<std::string> joint_names_;
    std::vector<pinocchio::JointIndex> q_indices_, v_indices_;
    Eigen::MatrixXd jacobian_;
};

void print_usage() {
    std::cout << "Usage: " << std::endl;
    std::cout << " move - Move the arm to a predefined pose with PD control" << std::endl;
    std::cout << " home - Move the arm to the home position with PD control" << std::endl;
    std::cout << " quit - Exit the program" << std::endl;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    const std::string urdf_path = "/home/home/tr_project/src/tr_description/urdf/amr_sim.urdf";
    auto pd_controller = std::make_shared<DualArmPDController>(urdf_path);

    std::thread ros_thread([&]() {
        rclcpp::spin(pd_controller);
    });

    print_usage();
    std::string command;
    while (rclcpp::ok()) {
        std::cout << "> ";
        std::cin >> command;
        if (command == "move") {
            pd_controller->do_move();
        } else if (command == "home") {
            pd_controller->do_home();
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
