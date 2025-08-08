#include "driving_motor.hpp"
#include <lely/util/diag.h>


void lelyLogHandler(void * /*handle*/, enum diag_severity severity, int /*errc*/,
                    const char *format, va_list ap)
{
    if (severity < DIAG_DEBUG)
        return;

    char buffer[1024];
    vsnprintf(buffer, sizeof(buffer), format, ap);

    std::cerr << "[LELY][" << severity << "] " << buffer << std::endl;
}


DrivingMotorsCanOpen::DrivingMotorsCanOpen() : Node("driving_motor")
{
    logWrite(LOG_INFO, "driving_motor node is started");
    declareParameters();
    getParameters();

    canopen_thread_ = std::thread(&DrivingMotorsCanOpen::canOpenSysn, this);
    logWrite(LOG_INFO, "creating a thread is done.");

    sample_time_ = 1.0 / motor_freq_;

    joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    joint_state_publisher_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&DrivingMotorsCanOpen::publishJointStateCallback, this)
    );

    joint_desired_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_desired", 10, std::bind(&DrivingMotorsCanOpen::jointDesiredCallback, this, _1));
}

DrivingMotorsCanOpen::~DrivingMotorsCanOpen()
{
    if (canopen_thread_.joinable())
    {
        canopen_thread_.join();
    }
}
void DrivingMotorsCanOpen::declareParameters()
{
    RCLCPP_INFO(get_logger(), "driving_motor_node declareParameters starts! ");
    this->declare_parameter("motor_freq", 100.0);
    this->declare_parameter("can_interface", "can0");
    this->declare_parameter("dcf_path", "");
    this->declare_parameter("num_driver", 1);
    RCLCPP_INFO(get_logger(), "driving_motor_node declareParameters finished! ");
}
void DrivingMotorsCanOpen::getParameters()
{
    RCLCPP_INFO(get_logger(), "driving_motor_node getParameters starts! ");
    this->get_parameter<double>("motor_freq", motor_freq_);
    this->get_parameter<std::string>("can_interface", can_interface_);
    this->get_parameter<std::string>("dcf_path", dcf_path_);
    this->get_parameter<int>("num_driver", num_driver);

    RCLCPP_INFO(get_logger(), "Motor Freq: %f", motor_freq_);
    RCLCPP_INFO(get_logger(), "CAN Interface: %s", can_interface_.c_str());
    RCLCPP_INFO(get_logger(), "DCF Path: %s", dcf_path_.c_str());
    RCLCPP_INFO(get_logger(), "Num Driver: %d", num_driver);

    RCLCPP_INFO(get_logger(), "driving_motor_node getParameters finished! ");
}
void DrivingMotorsCanOpen::init()
{
    for (int j = 0; j < this->num_driver; j++)
    {
        for (int i = 0; i < 2; i++)
        {
            drivers[j]->motor_control_.set_acc_dec[i] = false;
            drivers[j]->motor_control_.set_vel_pos[i] = false;
        }
        drivers[j]->init_canopen = true;
    }
    logWrite(LOG_INFO, "init is done.");
    init_done_ = true;
}

void DrivingMotorsCanOpen::canOpenSysn()
{

    diag_set_handler(lelyLogHandler, nullptr);

    logWrite(LOG_INFO, "DrivingMotorsCanOpen::canOpenSysn starts.");
    io::IoGuard io_guard;
    io::Context ctx;
    io::Poll poll(ctx);
    ev::Loop loop(poll.get_poll());
    // I/O devices only need access to the executor interface of the event loop.
    logWrite(LOG_INFO, "loop.get_executor()");
    auto exec = loop.get_executor();
    // Create a timer using a monotonic clock, a clock that is not affected
    // by discontinuous jumps in the system time.
    io::Timer timer(poll, exec, CLOCK_MONOTONIC);
    // Create a SocketCAN CAN controller and channel, and do not modify CAN bus state or bitrate.
    logWrite(LOG_INFO, "io::Timer timer(poll, exec, CLOCK_MONOTONIC);");
    io::CanController ctrl(can_interface_.c_str());
    // io::CanController ctrl("can3");
    io::CanChannel chan(poll, exec);
    chan.open(ctrl);

    logWrite(LOG_INFO, "Tcon_Master _master(timer,...");
    // dcf file is ok
    Tcon_Master _master(timer, chan, dcf_path_.c_str(), "", 111);
    master = &_master;

    // Create a driver for the slave with node-ID 1-2-3-4-...
    // Create a vector of VelocityControl objects
    std::stringstream ss;
    for (int i = 0; i < num_driver; ++i)
    {
        drivers_unique_.push_back(std::make_unique<VelocityControl>(exec, _master, i + 1));
        ss << "Initialized VelocityControl driver " << (i + 1);
    }
    logWrite(LOG_INFO, ss.str());

    for (int i = 0; i < num_driver; ++i)
    {
        drivers.push_back(drivers_unique_[i].get());
    }


    // Create a signal handler.[%d](exec, _master, %d);", i, i + 1
    io::SignalSet sigset(poll, exec);
    // Watch for Ctrl+C or process termination.
    sigset.insert(SIGHUP);
    sigset.insert(SIGINT);
    sigset.insert(SIGTERM);

    // Submit a task to be executed when a signal is raised. We don't care which.
    sigset.submit_wait([&](int /*signo*/)
                       {
                            cout << "deconfiguration process:" << endl;
                            logWrite(LOG_INFO, "deconfiguration process start.");
                            // If the signal is raised again, terminate immediately.
                            sigset.clear();
                            master->AsyncDeconfig().submit(exec, [&]() {
                            // Perform a clean shutdown.
                            ctx.shutdown();
                        }); });
    init();
    // Start the NMT service of the master by pretending to receive a 'reset node' command.
    master->Reset();
    logWrite(LOG_INFO, "Master reset end.");

    loop.run();
}
// Set the time, enter the desired time, and check the time.
bool DrivingMotorsCanOpen::getTimeOut(unsigned int dw_start_tick_count, unsigned int millisecound)
{
    if (elapseTime(dw_start_tick_count) >= millisecound)
        return true;
    else
        return false;
}

unsigned int DrivingMotorsCanOpen::getTickCount()
{
    struct timeval tv;
    gettimeofday(&tv, 0);
    return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

unsigned long DrivingMotorsCanOpen::elapseTime(unsigned int dw_start_tick_count)
{
    return getTickCount() - dw_start_tick_count;
}

// function to write log file
void DrivingMotorsCanOpen::logWrite(LogLevel level, std::string log_string)
{
    auto &logger = AmrLogger::getInstance();
    logger.logWrite(LOG_MOTOR_DRIVER, level, log_string);
}

// convert int to binary vector
std::vector<bool> DrivingMotorsCanOpen::intToBinaryVector(uint16_t num, uint8_t length)
{
    std::vector<bool> binaryVector(length, 0);
    // Start from the end of the vector
    int index = length - 1;
    // Continue shifting the bits until num becomes 0
    while (num > 0 && index >= 0)
    {
        // Extract the least significant bit and assign it to the vector
        binaryVector[index] = num & 1;
        // Right shift num by 1 bit
        num >>= 1;
        // Move to the previous position in the vector
        index--;
    }
    // Reverse the vector to get the correct binary representation
    std::reverse(binaryVector.begin(), binaryVector.end());
    return binaryVector;
}

// Function to find and print the string corresponding to a given number
void DrivingMotorsCanOpen::findErrorMessage(int code)
{
    for (const auto &error : errors)
    {
        if (error.error_code == code)
        {
            logWrite(LOG_INFO, "Error: " + error.error_name);
            return;
        }
    }
}

void DrivingMotorsCanOpen::publishJointStateCallback()
{
    if (!init_done_) return;

    auto joint_state_msg = std::make_unique<sensor_msgs::msg::JointState>();
    joint_state_msg->header.stamp = this->get_clock()->now();

    for (int i = 0; i < num_driver; ++i)
    {
        // Assuming each driver has two motors
        for (int j = 0; j < 2; ++j)
        {
            std::string joint_name = "driver" + std::to_string(i + 1) + "_motor" + std::to_string(j + 1);
            joint_state_msg->name.push_back(joint_name);

            // Position data might need conversion from ticks to radians
            // Velocity data might need conversion from RPM or other units to rad/s
            // For now, using raw values.
            joint_state_msg->position.push_back(drivers[i]->motor_status_.feedback_pos[j]);
            joint_state_msg->velocity.push_back(drivers[i]->motor_status_.feedback_vel[j]);
            joint_state_msg->effort.push_back(0.0); // Effort is not available
        }
    }

    joint_state_publisher_->publish(std::move(joint_state_msg));
}

void DrivingMotorsCanOpen::jointDesiredCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (!init_done_) return;

    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        for (int driver_idx = 0; driver_idx < num_driver; ++driver_idx)
        {
            for (int motor_idx = 0; motor_idx < 2; ++motor_idx)
            {
                std::string joint_name = "driver" + std::to_string(driver_idx + 1) + "_motor" + std::to_string(motor_idx + 1);
                if (msg->name[i] == joint_name)
                {
                    if (i < msg->velocity.size())
                    {
                        drivers[driver_idx]->motor_control_.target_vel[motor_idx] = static_cast<int32_t>(msg->velocity[i]);
                        drivers[driver_idx]->motor_control_.set_vel_pos[motor_idx] = true;
                    }
                }
            }
        }
    }
}
