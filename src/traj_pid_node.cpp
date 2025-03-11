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
#include <cmath> 

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
          pid_control_(1.0, 0.0, 0.1),  // 初始化 PID 控制器（位置）
          pid_orientation_control_(1.0, 0.0, 0.1),  // 初始化朝向 PID 控制器
          pid_velocity_control_(1.0, 0.0, 0.1),  // 初始化线速度 PID 控制器
          pid_angular_velocity_control_(1.0, 0.0, 0.1),  // 初始化角速度 PID 控制器
          target_linear_velocity_(0.0), target_angular_velocity_(0.0),  // 初始化目标线速度和角速度
          current_linear_velocity_(0.0), current_angular_velocity_(0.0),  // 初始化当前线速度和角速度
          current_position_x_(0.0), current_position_y_(0.0), current_yaw_(0.0),  // 初始化位置和航向
          target_x_(0.0), target_y_(0.0), target_yaw_(0.0)  // 初始化目标位置和航向
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
    
            bspline_subscription_ = this->create_subscription<planner::msg::Bspline>(
                "/bspline", 10, std::bind(&TrajPidNode::bspline_callback, this, std::placeholders::_1));
    
            cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    
            RCLCPP_INFO(this->get_logger(), "PID 轨迹跟踪节点已启动");
    
            // 输出PID控制器参数
            pid_control_.print_parameters();
            pid_orientation_control_.print_parameters();
            pid_velocity_control_.print_parameters();
            pid_angular_velocity_control_.print_parameters();
    
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
        // 控制点结构体
        struct ControlPoint {
            double x;
            double y;
            double yaw;
            double time;
        };
    
        // 重载输出流操作符，输出控制点信息
        friend std::ostream& operator<<(std::ostream& os, const ControlPoint& cp) {
            os << "(x: " << cp.x << ", y: " << cp.y << ", yaw: " << cp.yaw << ", time: " << cp.time << ")";
            return os;
        }
    
        // 声明成员变量
        PIDController pid_control_;  // PID 控制器（位置）
        PIDController pid_orientation_control_;  // PID 控制器（朝向）
        PIDController pid_velocity_control_;  // PID 控制器（线速度）
        PIDController pid_angular_velocity_control_;  // PID 控制器（角速度）
    
        std::ofstream log_file_;  // 日志文件
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr traj_subscription_;  // Trajectory 订阅者
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;  // Odometry 订阅者
        rclcpp::Subscription<planner::msg::Bspline>::SharedPtr bspline_subscription_;  // Bspline 订阅者
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;  // 速度控制指令发布者
        rclcpp::TimerBase::SharedPtr control_timer_;  // 定时器
    
        // 控制点存储
        std::vector<ControlPoint> control_points_;  
    
        // 状态变量
        bool bspline_received_ = false;
        bool odom_received_ = false;
        double target_linear_velocity_;
        double target_angular_velocity_;
        double current_linear_velocity_;
        double current_angular_velocity_;
        double current_position_x_;
        double current_position_y_;
        double current_yaw_;
        double target_x_;
        double target_y_;
        double target_yaw_;
    
        // 回调函数处理
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
    
        void bspline_callback(const planner::msg::Bspline::SharedPtr msg) {
            double current_time_sec = this->now().seconds();  // 获取当前时间（单位：秒）
            std::string current_time = get_current_time_str();  // 用于日志
    
            if (msg == nullptr || msg->pos_pts.empty() || msg->yaw_pts.empty()) {
                log_file_ << "[" << current_time << "] Received Bspline message - Data is missing" << std::endl;
                bspline_received_ = false;
                return;
            }
    
            bspline_received_ = true;
            log_file_ << "[" << current_time << "] Received Bspline message - pos_pts size: " << msg->pos_pts.size()
                        << ", yaw_pts size: " << msg->yaw_pts.size() << std::endl;
    
            control_points_.clear();
            double yaw_dt = msg->yaw_dt; // 获取 yaw_dt
    
            for (size_t i = 0; i < msg->pos_pts.size(); ++i) {
                ControlPoint cp;
                cp.x = msg->pos_pts[i].x;
                cp.y = msg->pos_pts[i].y;
                cp.yaw = (i < msg->yaw_pts.size()) ? msg->yaw_pts[i] : 0.0;
                cp.time = current_time_sec + i * yaw_dt;  // 使用 yaw_dt 来计算控制点时间
                control_points_.push_back(cp);
    
                log_file_ << "[" << current_time << "] Control point" << i << ": x=" << cp.x
                            << ", y=" << cp.y << ", yaw=" << cp.yaw << ", time=" << std::setprecision(6) << i * yaw_dt << std::endl;
            }
    
            if (!control_points_.empty()) {
                target_x_ = control_points_[0].x;
                target_y_ = control_points_[0].y;
                target_yaw_ = control_points_[0].yaw;
            }
        }
    
        void control_loop() {

            double current_time_sec = this->now().seconds();  // 获取当前时间（单位：秒）
            std::string current_time = get_current_time_str();  // 用于日志

            if (control_points_.empty()) {
                log_file_ << "[" << current_time << "] No control points available" << std::endl;
                return;
            }

            ControlPoint target_control_point;
            bool target_found = false;

            for (size_t i = 0; i < control_points_.size(); ++i) {
                const auto& cp = control_points_[i];
                if (cp.time > current_time_sec) {
                    target_control_point = cp;
                    target_found = true;
                    break;
                }
            }

            if (target_found) {
                target_x_ = target_control_point.x;
                target_y_ = target_control_point.y;
                target_yaw_ = target_control_point.yaw;
            }

            // 计算位置误差（目标位置与当前位置的欧几里得距离）
            double position_error = std::sqrt(std::pow(target_x_ - current_position_x_, 2) + std::pow(target_y_ - current_position_y_, 2));
            
            // 计算朝向误差（目标航向与当前航向的误差）
            double orientation_error = target_yaw_ - current_yaw_;
            if (orientation_error > M_PI) {
                orientation_error -= 2 * M_PI;
            } else if (orientation_error < -M_PI) {
                orientation_error += 2 * M_PI;
            }
        
            // 计算线速度误差（目标线速度与当前线速度的误差）
            double velocity_error = target_linear_velocity_ - current_linear_velocity_;
        
            // 计算角速度误差（目标角速度与当前角速度的误差）
            double angular_velocity_error = target_angular_velocity_ - current_angular_velocity_;
        
            // 使用 PID 控制器计算位置控制输出
            double pid_linear_output = pid_control_.compute(position_error, 0.0);
            
            // 使用 PID 控制器计算朝向控制输出
            double pid_angular_output = pid_orientation_control_.compute(orientation_error, 0.0);
        
            // 使用 PID 控制器计算线速度控制输出
            double pid_velocity_output = pid_velocity_control_.compute(velocity_error, 0.0);
            
            // 使用 PID 控制器计算角速度控制输出
            double pid_angular_velocity_output = pid_angular_velocity_control_.compute(angular_velocity_error, 0.0);
        
            // 通过加权来组合各个控制输出
            // 调整加权系数，可以根据实际需要调整
            const double position_weight = 1.0;
            const double velocity_weight = 1.0;
            const double orientation_weight = 1.0;
            const double angular_velocity_weight = 1.0;
        
            pid_linear_output = pid_linear_output * position_weight + pid_velocity_output * velocity_weight;
            pid_angular_output = pid_angular_output * orientation_weight + pid_angular_velocity_output * angular_velocity_weight;
        
            // 限制输出，避免过大控制量
            const double max_linear_velocity = 2.0;  // 最大线速度
            const double max_angular_velocity = 0.5; // 最大角速度
            pid_linear_output = std::clamp(pid_linear_output, -max_linear_velocity, max_linear_velocity);
            pid_angular_output = std::clamp(pid_angular_output, -max_angular_velocity, max_angular_velocity);
        
            // 判断数据是否已接收到，如果没有，输出 'NA'
            std::string position_str = odom_received_ ? "x: " + std::to_string(current_position_x_) + ", y: " + std::to_string(current_position_y_) : "NA";
            std::string yaw_str = odom_received_ ? "Current Yaw: " + std::to_string(current_yaw_) : "NA";
            std::string target_position_str = bspline_received_ ? "Target Position - x: " + std::to_string(target_x_) + ", y: " + std::to_string(target_y_) : "NA";
            std::string target_yaw_str = bspline_received_ ? "Target Yaw: " + std::to_string(target_yaw_) : "NA";
        
            // 定义实际线速度和角速度
            std::string actual_linear_velocity_str = odom_received_ ? std::to_string(current_linear_velocity_) : "NA";
            std::string actual_angular_velocity_str = odom_received_ ? std::to_string(current_angular_velocity_) : "NA";
        
            // 日志记录 PID 控制输出和目标与当前状态
            log_file_ << "-------------------------" << std::endl;
            log_file_ << "-------------------------" << std::endl;
        
            log_file_ << "[" << get_current_time_str() << "] Selected control point: "
                      << "(x: " << target_x_ << ", y: " << target_y_ << ", yaw: " << target_yaw_ << ")" << std::endl;
        
            log_file_ << "&&&&&& TIMESTAMP: &&&&&&&" << get_current_time_str() << std::endl;
            log_file_ << "Control Loop - PID Output: Linear Velocity: " << pid_linear_output
                      << ", Angular Velocity: " << pid_angular_output << std::endl;
        
            log_file_ << "PID Control - Linear Velocity Target: " << target_linear_velocity_
                      << ", Angular Velocity Target: " << target_angular_velocity_ << std::endl;
        
            log_file_ << target_position_str << ", " << target_yaw_str << std::endl;
            log_file_ << "Current Position - " << position_str << ", " << yaw_str << std::endl;
        
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

};
    

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajPidNode>());
    rclcpp::shutdown();
    return 0;
}