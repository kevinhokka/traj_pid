#include <memory>
#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <fstream>
#include <iomanip>
#include <ctime>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "planner/msg/bspline.hpp"  // 引入Bspline消息
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <sys/stat.h>
#include <filesystem>  // C++17文件系统库

// 获取当前时间字符串
std::string get_current_time_str() {
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    std::tm tm = *std::localtime(&now_time_t);
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    std::stringstream ss;
    ss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S") << "." << std::setw(3) << std::setfill('0') << milliseconds.count();
    return ss.str();
}


class PIDController {
public:
    PIDController(double p, double i, double d)
    : p_gain(p), i_gain(i), d_gain(d), prev_error(0), integral(0) {}

    double compute(double setpoint, double actual) {
        double error = setpoint - actual;
        integral += error;
        double derivative = error - prev_error;
        prev_error = error;

        double output = p_gain * error + i_gain * integral + d_gain * derivative;

        return output;
    }

    void print_parameters() const {
        std::cout << "PID Parameters - P: " << p_gain << ", I: " << i_gain << ", D: " << d_gain << std::endl;
    }

private:
    double p_gain, i_gain, d_gain;
    double prev_error, integral;
};

class TrajPidNode : public rclcpp::Node {
public:
    TrajPidNode()
    : Node("traj_pid_node"),
      pid_control_(1.0, 0.0, 0.1),  // 初步设置PID参数
      pid_orientation_control_(1.0, 0.0, 0.1) // 朝向PID控制器

    {   
        // 创建日志文件夹
        std::string log_dir = "/home/jetson/ros2_ws/src/traj_pid/traj_pid_log";
        if (!std::filesystem::exists(log_dir)) {
            std::filesystem::create_directory(log_dir);
        }

        // 获取当前时间并创建日志文件
        std::string log_filename = log_dir + "/" + get_current_time_str() + ".txt";
        log_file_.open(log_filename, std::ios::out);

        if (!log_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "无法打开日志文件: %s", log_filename.c_str());
            throw std::runtime_error("无法打开日志文件");
        }

        RCLCPP_INFO(this->get_logger(), "Logging to file: %s", log_filename.c_str());

        traj_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/traj", 10, std::bind(&TrajPidNode::traj_callback, this, std::placeholders::_1));

        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10, std::bind(&TrajPidNode::odom_callback, this, std::placeholders::_1));

        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/livox/imu", 10, std::bind(&TrajPidNode::imu_callback, this, std::placeholders::_1));

        bspline_subscription_ = this->create_subscription<planner::msg::Bspline>(
            "/bspline", 10, std::bind(&TrajPidNode::bspline_callback, this, std::placeholders::_1));

        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "PID 轨迹跟踪节点已启动");

        // 输出PID控制器参数
        pid_control_.print_parameters();
        pid_orientation_control_.print_parameters();

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 每100ms调用一次
            std::bind(&TrajPidNode::control_loop, this));
    }

    ~TrajPidNode() {
        if (log_file_.is_open()) {
            log_file_.close();  // 确保文件在节点退出时被关闭
        }
    }

private:

    rclcpp::TimerBase::SharedPtr control_timer_;

    bool bspline_received_ = false;
    bool odom_received_ = false;
    bool imu_received_ = false;

    void traj_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        target_linear_velocity_ = msg->linear.x;
        target_angular_velocity_ = msg->angular.z;

        log_file_ << "Received trajectory - Linear velocity: " << target_linear_velocity_
                  << ", Angular velocity: " << target_angular_velocity_ << std::endl;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {

        if (msg == nullptr) {
            odom_received_ = false;
            return;
        }
    
        odom_received_ = true;

        current_linear_velocity_ = msg->twist.twist.linear.x;
        current_angular_velocity_ = msg->twist.twist.angular.z;
        current_position_x_ = msg->pose.pose.position.x;
        current_position_y_ = msg->pose.pose.position.y;

        log_file_ << "Received Odometry - Position: x=" << current_position_x_
                  << ", y=" << current_position_y_
                  << ", Linear Velocity: " << current_linear_velocity_
                  << ", Angular Velocity: " << current_angular_velocity_ << std::endl;

        tf2::Quaternion quat;
        tf2::fromMsg(msg->pose.pose.orientation, quat);

        double roll, pitch;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, current_yaw_);

        log_file_ << "Current yaw: " << current_yaw_ << " (roll: " << roll << ", pitch: " << pitch << ")" << std::endl;
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {

        if (msg == nullptr) {
            imu_received_ = false;
            return;
        }

        imu_received_ = true;

        double angular_velocity = msg->angular_velocity.z;
        double linear_acceleration = msg->linear_acceleration.x;

        log_file_ << "IMU - Angular Velocity: " << angular_velocity
                  << ", Linear Acceleration: " << linear_acceleration << std::endl;
    }

    void bspline_callback(const planner::msg::Bspline::SharedPtr msg) {
        auto current_time = get_current_time_str(); // 获取当前时间戳
        if (msg == nullptr || msg->pos_pts.empty() || msg->yaw_pts.empty()) {
            log_file_ << "[" << current_time << "] Received Bspline message - Data is missing" << std::endl;
            bspline_received_ = false;
            return;
        }

        bspline_received_ = true;
        log_file_ << "[" << current_time << "] Received Bspline message - pos_pts size: " << msg->pos_pts.size()
                << ", yaw_pts size: " << msg->yaw_pts.size() << std::endl;

        for (size_t i = 0; i < msg->pos_pts.size(); ++i) {
            target_x_ = msg->pos_pts[i].x;
            target_y_ = msg->pos_pts[i].y;

            if (i < msg->yaw_pts.size()) {
                target_yaw_ = msg->yaw_pts[i];
            }

            log_file_ << "[" << current_time << "] Target position " << i << ": x=" << target_x_
                    << ", y=" << target_y_ << ", target yaw=" << target_yaw_ << std::endl;
        }
    }
    

    void control_loop() {
        // 获取当前时间戳
        std::string current_time = get_current_time_str();
    
        // 计算当前位置和目标位置的误差
        double position_error = std::sqrt(std::pow(target_x_ - current_position_x_, 2) + std::pow(target_y_ - current_position_y_, 2));
        
        // 使用PID控制器计算线速度输出
        double pid_linear_output = pid_control_.compute(position_error, 0.0);
        
        // 计算当前航向与目标航向之间的误差
        double orientation_error = target_yaw_ - current_yaw_;
        
        // 确保误差在 -π 到 π 之间
        if (orientation_error > M_PI) {
            orientation_error -= 2 * M_PI;
        } else if (orientation_error < -M_PI) {
            orientation_error += 2 * M_PI;
        }
        
        // 使用PID控制器计算角速度输出
        double pid_angular_output = pid_orientation_control_.compute(orientation_error, 0.0);
        
        // 输出限制（避免过大控制量）
        const double max_linear_velocity = 2.0;  // 最大线速度
        const double max_angular_velocity = 0.5; // 最大角速度
        
        pid_linear_output = std::clamp(pid_linear_output, -max_linear_velocity, max_linear_velocity);
        pid_angular_output = std::clamp(pid_angular_output, -max_angular_velocity, max_angular_velocity);
        
        // 判断数据是否已接收到，如果没有，输出 'NA'
        std::string position_str = odom_received_ ? "x: " + std::to_string(current_position_x_) + ", y: " + std::to_string(current_position_y_) : "NA";
        std::string yaw_str = odom_received_ ? "Current Yaw: " + std::to_string(current_yaw_) : "NA";
        std::string target_position_str = bspline_received_ ? "Target Position - x: " + std::to_string(target_x_) + ", y: " + std::to_string(target_y_) : "NA";
        std::string target_yaw_str = bspline_received_ ? "Target Yaw: " + std::to_string(target_yaw_) : "NA";
        std::string actual_linear_velocity_str = imu_received_ ? std::to_string(current_linear_velocity_) : "NA";
        std::string actual_angular_velocity_str = imu_received_ ? std::to_string(current_angular_velocity_) : "NA";

        // 日志记录PID控制输出和目标与当前状态
        log_file_ << "-------------------------" << std::endl;
        log_file_ << "-------------------------" << std::endl;
        log_file_ << "Timestamp: " << current_time << std::endl;
        log_file_ << "Control Loop - PID Output: Linear Velocity: " << pid_linear_output
                  << ", Angular Velocity: " << pid_angular_output << std::endl;
        
        log_file_ << "PID Control - Linear Velocity Target: " << target_linear_velocity_
                  << ", Angular Velocity Target: " << target_angular_velocity_ << std::endl;
        
        log_file_ << target_position_str << ", " << target_yaw_str << std::endl;
        log_file_ << "Current Position - " << position_str << ", " << yaw_str << std::endl;

        // 记录实际的线速度和角速度，若未接收到IMU数据则显示NA
        log_file_ << "Current Linear Velocity: " << actual_linear_velocity_str << std::endl;
        log_file_ << "Current Angular Velocity: " << actual_angular_velocity_str << std::endl;

        log_file_ << "-------------------------" << std::endl;
        log_file_ << "-------------------------" << std::endl;
        
        // 发布控制指令
        geometry_msgs::msg::Twist cmd_msg;
        cmd_msg.linear.x = pid_linear_output;
        cmd_msg.angular.z = pid_angular_output;
        
        cmd_publisher_->publish(cmd_msg);
    }
    
    

    PIDController pid_control_;
    PIDController pid_orientation_control_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr traj_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<planner::msg::Bspline>::SharedPtr bspline_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;

    double target_linear_velocity_ = 0.0;
    double target_angular_velocity_ = 0.0;
    double current_linear_velocity_ = 0.0;
    double current_angular_velocity_ = 0.0;
    double target_x_ = 0.0;
    double target_y_ = 0.0;
    double current_position_x_ = 0.0;
    double current_position_y_ = 0.0;
    double current_yaw_ = 0.0;
    double target_yaw_ = 0.0;

    std::ofstream log_file_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajPidNode>());
    rclcpp::shutdown();
    return 0;
}