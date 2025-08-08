#include "rclcpp/rclcpp.hpp"
#include "can_open.hpp"
#include "std_msgs/msg/string.hpp"
#include "tr_driver_msgs/msg/motor_status.hpp"
#include "tr_driver_msgs/msg/motor_driver.hpp"
#include "tr_driver_msgs/msg/channel_status.hpp"
#include "tr_driver_msgs/msg/safety_status.hpp"

#include "tr_driver_msgs/srv/motor_run_position.hpp"
#include "tr_driver_msgs/srv/motor_run_velocity.hpp"
#include "tr_driver_msgs/srv/motor_stop_homing.hpp"
#include "tr_driver_msgs/srv/motor_send_var.hpp"
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include "tr_driver_msgs/msg/wheel_control.hpp"
#include "data_type/constants.h"
#include "data_type/alarm_data.hpp"
#include "helper/thread_safety_queue.hpp"
#include "helper/converter.hpp"
#include "helper/amr_logger.hpp"
#include "helper/file_io.hpp"
#include "nlohmann/json.hpp"
#include "motor_define.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <memory>
#include <iostream>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <string>
#include <signal.h>
#include <cmath>
#include <thread>
#include <chrono>
#include <bitset>
#include <sys/time.h>
#include <fstream>
#include <mutex> // For std::mutex and std::lock_guard

using nlohmann::json;
using namespace std::placeholders;
using namespace std::chrono_literals;
using std::placeholders::_1;

const double PI = 3.14159266;

struct Error
{
	std::string error_name;
	int error_code;
};

class DrivingMotorsCanOpen : public rclcpp::Node
{
public:
	DrivingMotorsCanOpen();
	~DrivingMotorsCanOpen();

	void declareParameters();
	void getParameters();
	void canOpenSysn();
	void init();
	void publishControlFeedbackCallback();
	void publishStatusFeedbackCallback();
	void publishJointStateCallback();
	void sendCommandCallback();

	void safetyStatusCallback(const tr_driver_msgs::msg::SafetyStatus::SharedPtr msg);
	void wheelMotorCallback(const tr_driver_msgs::msg::WheelControl::SharedPtr msg);
	void jointDesiredCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
	void logWrite(LogLevel level, std::string log_string);
	void findErrorMessage(int code);
	std::vector<bool> intToBinaryVector(uint16_t num, uint8_t length);
	std::string axis_state(uint16_t fault_code, bool axis_busy, bool homing_complete, bool homing, bool upper_limit, bool lower_limit, int32_t position);

	// service - server
	void motorRunPositionServerCallback(const std::shared_ptr<tr_driver_msgs::srv::MotorRunPosition::Request> request,
										std::shared_ptr<tr_driver_msgs::srv::MotorRunPosition::Response> response);
	void motorRunVelocityServerCallback(const std::shared_ptr<tr_driver_msgs::srv::MotorRunVelocity::Request> request,
										std::shared_ptr<tr_driver_msgs::srv::MotorRunVelocity::Response> response);
	void motorStopServerCallback(const std::shared_ptr<tr_driver_msgs::srv::MotorStopHoming::Request> request,
								 std::shared_ptr<tr_driver_msgs::srv::MotorStopHoming::Response> response);
	void motorHomingServerCallback(const std::shared_ptr<tr_driver_msgs::srv::MotorStopHoming::Request> request,
								   std::shared_ptr<tr_driver_msgs::srv::MotorStopHoming::Response> response);

	// Time Function
	unsigned int getTickCount();
	unsigned long elapseTime(unsigned int dw_start_tick_count);
	bool getTimeOut(unsigned int dw_start_tick_count, unsigned millisecound);

public: // var
		// Create a vector of Error structs to store the error messages and codes
	std::vector<Error> errors = {
		{"AlignmentFail", 3601},
		{"CargoIsOnEquipment", 3602},
		{"CargoIsNotOnEquipment", 3603},
		{"CargoIsOnTray", 3604},
		{"CargoIsNotOnTray", 3605},
		{"CargoIsOnStorage", 3606},
		{"CargoIsNotOnStorage", 3607},
		{"EquipmentIndexWrong", 3608},
		{"StorageIndexWrong", 3609},
		{"XAxisMovingFail", 3610},
		{"AAxisMovingFail", 3611},
		{"TAxisMovingFail", 3612},
		{"YAxisMovingFail", 3613},
		{"ZAxisMovingFail", 3614},
		{"XAxisCommandFail", 3615},
		{"AAxisCommandFail", 3616},
		{"TAxisCommandFail", 3617},
		{"YAxisCommandFail", 3618},
		{"ZAxisCommandFail", 3619},
		{"AxisCommandFail", 3620},
		{"XAxisBeltCut", 2601},
		{"TAxisBeltCut", 2602},
		{"AAxisBeltCut", 2603},
		{"XAMotorOverHeat", 2604},	  // f1
		{"XAMotorOverVolt", 2605},	  // f2
		{"XAMotorUnderVolt", 2606},	  // f3
		{"XAMotorShort", 2607},		  // f4
		{"XAMotorEstop", 2608},		  // f5
		{"XAMotorSensorFault", 2609}, // f6
		{"XAMotorMosFail", 2610},	  // f7
		{"XAMotorDefConfig", 2611},	  // f8
		{"XAMotorSTOFault", 2612},	  // f9
		{"TYMotorOverHeat", 2613},	  // f1
		{"TYMotorOverVolt", 2614},	  // f2
		{"TYMotorUnderVolt", 2615},	  // f3
		{"TYMotorShort", 2616},		  // f4
		{"TYMotorEstop", 2617},		  // f5
		{"TYMotorSensorFault", 2618}, // f6
		{"TYMotorMosFail", 2619},	  // f7
		{"TYMotorDefConfig", 2620},	  // f8
		{"TYMotorSTOFault", 2621},	  // f9
		{"ZMotorOverHeat", 2622},	  // f1
		{"ZMotorOverVolt", 2623},	  // f2
		{"ZMotorUnderVolt", 2624},	  // f3
		{"ZMotorShort", 2625},		  // f4
		{"ZMotorEstop", 2626},		  // f5
		{"ZMotorSensorFault", 2627},  // f6
		{"ZMotorMosFail", 2628},	  // f7
		{"ZMotorDefConfig", 2629},	  // f8
		{"ZMotorSTOFault", 2630},	  // f9
		{"MotorAmpLim", 340},
		{"MotorStall", 341},
		{"MotorLoopError", 342},
		{"MotorSafeStop", 343},
		{"MotorFwdLimit", 344},
		{"MotorRevLimit", 345},
		{"MotorAmpTrig", 346},
		{"MotorFETsOff", 347}};
	int walk_front_dir = 0;
	int walk_rear_dir = 0;
	int steer_front_dir = 0;
	int steer_rear_dir = 0;

	std::string can_interface_;
	std::string dcf_path_;

	double pre_motor_left_pos_ = 0;
	double pre_motor_right_pos_ = 0;
	double motor_left_pos_ = 0;
	double motor_right_pos_ = 0;
	int vel_timer = 0;
	int enc_pulse = 65536;
	double sample_time_ = 0.01;
	double motor_initial_ = false;
	double motor_freq_ = 100.0;

	bool init_done_ = false;
	double gear_walk_ = 20;
	double radius_walk_left_ = 0.07525;	 // meter
	double radius_walk_right_ = 0.07535; // meter


	bool go_home_ = false;
	bool home_finish_sub = false;
	bool st_comm_fail_to_ok = false; // status from CAN communication fail to ok => go home
	bool motor_stop_ = false;
	// walking
	int32_t walk_left_vel_cmd_;
	int32_t walk_rear_vel_cmd_;

	int32_t pre_fb_vel_left_ = 0;
	int32_t pre_fb_vel_right_ = 0;
	int32_t walk_rear_vel_resp_;

	uint16_t count_timer = 0;
	uint8_t count_sub_odom_ = 0;

	float v_wheel_sub_[4] = {0, 0, 0, 0};

	bool obstacle_detect_ = false;
	uint8_t raw_data_safety_st_ = 0;
	uint8_t raw_lidar_front_st_ = 0;
	uint8_t raw_lidar_rear_st_ = 0;
	uint8_t pre_raw_data_safety_st_ = 0;
	uint8_t pre_raw_lidar_front_st_ = 0;
	uint8_t pre_raw_lidar_rear_st_ = 0;
	uint8_t driving_motor_safety_ = 0;
	uint8_t pre_driving_motor_safety_ = 0;
	uint16_t count_safety_ = 0;
	unsigned int timeout_ = 0;

	std::thread canopen_thread_;
	std::thread SDO_thread_;

	std::mutex data_mutex; // Mutex for thread safety

	rclcpp::TimerBase::SharedPtr send_control_data_;
	rclcpp::TimerBase::SharedPtr send_status_data_;
	rclcpp::TimerBase::SharedPtr send_command_;
	rclcpp::TimerBase::SharedPtr joint_state_publisher_timer_;

	rclcpp::Subscription<tr_driver_msgs::msg::WheelControl>::SharedPtr wheel_motor_sub_;
	rclcpp::Subscription<tr_driver_msgs::msg::SafetyStatus>::SharedPtr safety_st_sub_;
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_desired_sub_;

	rclcpp::Publisher<tr_driver_msgs::msg::WheelControl>::SharedPtr wheel_motor_pub_;
	rclcpp::Publisher<tr_driver_msgs::msg::WheelControl>::SharedPtr control_data_pub_;
	rclcpp::Publisher<tr_driver_msgs::msg::MotorStatus>::SharedPtr status_data_pub_;
	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;

	// Create a client to send Roboteq command
	rclcpp::Service<tr_driver_msgs::srv::MotorRunPosition>::SharedPtr motor_run_position_server_;
	rclcpp::Service<tr_driver_msgs::srv::MotorRunVelocity>::SharedPtr motor_run_velocity_server_;
	rclcpp::Service<tr_driver_msgs::srv::MotorStopHoming>::SharedPtr motor_stop_server_;
	rclcpp::Service<tr_driver_msgs::srv::MotorStopHoming>::SharedPtr motor_homing_server_;

private:
	int num_driver = 1;
	// Functions
	// VelocityControl *drivers_1;
	// Create an array of pointers to VelocityControl
	// VelocityControl *drivers[1];
	std::vector<std::unique_ptr<VelocityControl>> drivers_unique_;
	std::vector<VelocityControl*> drivers;
	Tcon_Master *master;
};
