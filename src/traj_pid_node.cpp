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
#include "plugin/non_uniform_bspline.hpp"


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
        double p_gain, i_gain , d_gain;
        double prev_error, integral;
};
    
class TrajPidNode : public rclcpp::Node {
    public:
        TrajPidNode()
        : Node("traj_pid_node"),
        pid_control_(4.9371, 0.0019, 0.9322),  // 初始化 PID 控制器（位置）
        pid_orientation_control_(7, 0.0083, 0.6812),  // 初始化朝向 PID 控制器

        //   pid_velocity_control_(2, 0.5, 0.1),  // 初始化线速度 PID 控制器
        //   pid_angular_velocity_control_(3, 0.8, 0.2),  // 初始化角速度 PID 控制器

        pid_velocity_control_(1, 0, 1),  // 初始化线速度 PID 控制器
        pid_angular_velocity_control_(9.9905, 9.5899, 3.1715),  // 初始化角速度 PID 控制器


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
    
        
        void bspline_callback(const planner::msg::Bspline::SharedPtr msg)
        {
            double current_time_sec = this->now().seconds();
            std::string current_time = get_current_time_str();

            // 1. 检查消息有效性
            if (!msg) {
                log_file_ << "[" << current_time << "] B-spline message is null." << std::endl;
                bspline_received_ = false;
                return;
            }
            if (msg->pos_pts.empty() || msg->yaw_pts.empty()) {
                log_file_ << "[" << current_time << "] B-spline message missing pos_pts or yaw_pts." << std::endl;
                bspline_received_ = false;
                return;
            }
            bspline_received_ = true;
            log_file_ << "[" << current_time << "] Received Bspline message - pos_pts size: " 
                    << msg->pos_pts.size() << ", yaw_pts size: " << msg->yaw_pts.size() 
                    << ", order=" << msg->order << std::endl;

            // 2. 获取 yaw_dt、pos_pts 和 yaw_pts 的数量
            int n_yaw = static_cast<int>(msg->yaw_pts.size());
            int n_pos = static_cast<int>(msg->pos_pts.size());
            double yaw_dt = msg->yaw_dt;

            // 总时间 T 按照 yaw 数据计算：T = (n_yaw - 1) * yaw_dt
            double total_time = (n_yaw - 1) * yaw_dt;

            // 3. 清空旧的控制点
            control_points_.clear();

            // 4. 遍历每个 yaw 点，生成控制点
            //    对于每个 yaw 索引 i，对应时间 t = i * yaw_dt
            //    位置通过对 pos_pts 进行线性插值获得，下标：
            //         pos_index = (t / total_time) * (n_pos - 1)
            //    航向直接使用 msg->yaw_pts[i]
            for (int i = 0; i < n_yaw; i++) {
                double t = i * yaw_dt;
                double pos_index = 0.0;
                if (total_time > 0 && n_pos > 1) {
                    pos_index = (t / total_time) * (n_pos - 1);
                }
                int j = static_cast<int>(std::floor(pos_index));
                double fraction = pos_index - j;
                Eigen::Vector3d pos_xyz;
                if (j >= n_pos - 1) {
                    // 超出插值范围，直接使用最后一个点
                    pos_xyz << msg->pos_pts[n_pos - 1].x,
                            msg->pos_pts[n_pos - 1].y,
                            msg->pos_pts[n_pos - 1].z;
                } else {
                    // 线性插值：p = (1 - fraction) * pos_pts[j] + fraction * pos_pts[j+1]
                    pos_xyz.x() = (1.0 - fraction) * msg->pos_pts[j].x + fraction * msg->pos_pts[j + 1].x;
                    pos_xyz.y() = (1.0 - fraction) * msg->pos_pts[j].y + fraction * msg->pos_pts[j + 1].y;
                    pos_xyz.z() = (1.0 - fraction) * msg->pos_pts[j].z + fraction * msg->pos_pts[j + 1].z;
                }

                // 构造控制点：位置插值结果 + 直接抓取的 yaw 数据
                ControlPoint cp;
                cp.x    = pos_xyz.x();
                cp.y    = pos_xyz.y();
                cp.yaw  = msg->yaw_pts[i];   // 不进行插值，直接使用所有 yaw 信息
                cp.time = current_time_sec + t;
                control_points_.push_back(cp);
            }

            // 5. 打印调试信息
            log_file_ << "[" << current_time << "] Constructed " 
                    << control_points_.size() << " control points from B-spline." << std::endl;
            for (size_t i = 0; i < control_points_.size(); i++) {
                log_file_ << "  Pt[" << i << "] (x=" << control_points_[i].x 
                        << ", y=" << control_points_[i].y 
                        << ", yaw=" << control_points_[i].yaw
                        << ", t=" << control_points_[i].time - current_time_sec << ")" << std::endl;
            }

            // 6. 将第一个控制点作为当前目标（如果存在）
            if (!control_points_.empty()) {
                target_x_   = control_points_[0].x;
                target_y_   = control_points_[0].y;
                target_yaw_ = control_points_[0].yaw;
            }
        }

        void control_loop() {
            double current_time_sec = this->now().seconds();
            std::string current_time = get_current_time_str();
        
            // 若没有控制点，则记录日志并返回
            if (control_points_.empty()) {
                log_file_ << "-------------------------" << std::endl;
                log_file_ << "[" << current_time << "] Selected control point: NA" << std::endl;
                log_file_ << "-------------------------" << std::endl;
                return;
            }
        
            // ===== 外环控制：选择目标控制点 =====
            ControlPoint target_cp;
            bool target_found = false;
            for (const auto &cp : control_points_) {
                if (cp.time > current_time_sec) {
                    target_cp = cp;
                    target_found = true;
                    break;
                }
            }
            if (!target_found) {
                target_cp = control_points_.back();
            }
            target_x_   = target_cp.x;
            target_y_   = target_cp.y;
            target_yaw_ = target_cp.yaw;
        
            // ===== 计算位置和航向误差 =====
            double dx = target_x_ - current_position_x_;
            double dy = target_y_ - current_position_y_;
            double distance = std::sqrt(dx * dx + dy * dy);
            double angle_to_target = std::atan2(dy, dx);
            double alpha = angle_to_target - current_yaw_;
            while (alpha > M_PI)  alpha -= 2.0 * M_PI;
            while (alpha < -M_PI) alpha += 2.0 * M_PI;
            double e_forward = distance * std::cos(alpha);
        
            // ===== 使用PID控制器计算输出 =====
            double linear_cmd = pid_control_.compute(e_forward, 0.0);
            double angular_cmd = pid_orientation_control_.compute(alpha, 0.0);
        
            // 限幅处理
            const double max_linear_speed = 2.0;
            const double max_angular_speed = 0.5;
            if (linear_cmd >  max_linear_speed)  linear_cmd =  max_linear_speed;
            if (linear_cmd < -max_linear_speed)  linear_cmd = -max_linear_speed;
            if (angular_cmd >  max_angular_speed)  angular_cmd =  max_angular_speed;
            if (angular_cmd < -max_angular_speed)  angular_cmd = -max_angular_speed;
        
            // ===== 准备日志信息 =====
            std::string position_str = odom_received_
                ? ("x: " + std::to_string(current_position_x_) + ", y: " + std::to_string(current_position_y_))
                : "NA";
            std::string yaw_str = odom_received_
                ? ("Current Yaw: " + std::to_string(current_yaw_))
                : "NA";
            std::string target_position_str = bspline_received_
                ? ("Target Position - x: " + std::to_string(target_x_) + ", y: " + std::to_string(target_y_))
                : "NA";
            std::string target_yaw_str = bspline_received_
                ? ("Target Yaw: " + std::to_string(target_yaw_))
                : "NA";
            std::string actual_linear_velocity_str = odom_received_ ? std::to_string(current_linear_velocity_) : "NA";
            std::string actual_angular_velocity_str = odom_received_ ? std::to_string(current_angular_velocity_) : "NA";
        
            // ===== 日志记录（格式保持不变） =====
            log_file_ << "-------------------------" << std::endl;
            log_file_ << "[" << get_current_time_str() << "] Selected control point: "
                      << "(x: " << target_x_ << ", y: " << target_y_ << ", yaw: " << target_yaw_ << ")"
                      << std::endl;
            log_file_ << "-- PID Output: Linear Velocity: " << linear_cmd
                      << ", Angular Velocity: " << angular_cmd << std::endl;
            log_file_ << "Linear Velocity Target: " << target_linear_velocity_
                      << ", Angular Velocity Target: " << target_angular_velocity_ << std::endl;
            log_file_ << target_position_str << ", " << target_yaw_str << std::endl;
            log_file_ << "Current Position - " << position_str << ", " << yaw_str << std::endl;
            log_file_ << "--Distance to Target (abs): " << distance << std::endl;
            log_file_ << "--Position ERROR (signed): " << e_forward << std::endl;
            log_file_ << "--Orientation ERROR: " << alpha << std::endl;
            log_file_ << "Current Linear Velocity: " << actual_linear_velocity_str << std::endl;
            log_file_ << "Current Angular Velocity: " << actual_angular_velocity_str << std::endl;
            // 简化后的算法未使用内环控制，故速度误差均为0
            log_file_ << "--Linear Velocity ERROR: " << 0.0 << std::endl;
            log_file_ << "--Angular Velocity ERROR: " << 0.0 << std::endl;
            log_file_ << "-------------------------" << std::endl;
        
            // ===== 发布控制指令 =====
            geometry_msgs::msg::Twist cmd_msg;
            cmd_msg.linear.x  = linear_cmd;
            cmd_msg.angular.z = angular_cmd;
            cmd_publisher_->publish(cmd_msg);
        }

};
    

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajPidNode>());
    rclcpp::shutdown();
    return 0;
}