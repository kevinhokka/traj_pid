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
#include <Eigen/Dense>  // 用于Eigen矩阵操作


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

        std::string get_parameters_str() const {
            std::ostringstream oss;
            oss << "P: " << p_gain << ", I: " << i_gain << ", D: " << d_gain;
            return oss.str();
        }
    
    private:
        double p_gain, i_gain , d_gain;
        double prev_error, integral;
};
    
class TrajPidNode : public rclcpp::Node {
    public:
        TrajPidNode()
        : Node("traj_pid_node"),
        pid_control_(1, 0, 1),  // 初始化 PID 控制器（位置）
        pid_orientation_control_(1, 0, 1),  // 初始化朝向 PID 控制器

        pid_velocity_control_(1, 0, 1),  // 初始化线速度 PID 控制器
        pid_angular_velocity_control_(1, 0, 0),  // 初始化角速度 PID 控制器
        // pid_angular_velocity_control_(0.5, 0, 3),  // 初始化角速度 PID 控制器


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

            log_file_ << "-------------------------" << std::endl;
            log_file_ << "PID Controller Parameters:" << std::endl;
            log_file_ << "Position PID: " << pid_control_.get_parameters_str() << std::endl;
            log_file_ << "Orientation PID: " << pid_orientation_control_.get_parameters_str() << std::endl;
            log_file_ << "Linear Velocity PID: " << pid_velocity_control_.get_parameters_str() << std::endl;
            log_file_ << "Angular Velocity PID: " << pid_angular_velocity_control_.get_parameters_str() << std::endl;
            log_file_ << "-------------------------" << std::endl;
        
            RCLCPP_INFO(this->get_logger(), "Logging to file: %s", log_filename.c_str());
    
            // traj_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            //     "/traj", 10, std::bind(&TrajPidNode::traj_callback, this, std::placeholders::_1));
    
            odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "/fastlio2/lio_odom", 10, std::bind(&TrajPidNode::odom_callback, this, std::placeholders::_1));

            imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
                "/livox/imu", 10, std::bind(&TrajPidNode::imu_callback, this, std::placeholders::_1));
    
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
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
        rclcpp::Subscription<planner::msg::Bspline>::SharedPtr bspline_subscription_;  // Bspline 订阅者
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;  // 速度控制指令发布者
        rclcpp::TimerBase::SharedPtr control_timer_;  // 定时器
    
        // 用于B-spline轨迹的成员变量
        std::vector<planner::NonUniformBspline> traj_;
        std::chrono::time_point<std::chrono::system_clock> start_time_;
        double traj_duration_;
        bool receive_traj_;
    
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
        // void traj_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        //     target_linear_velocity_ = msg->linear.x;
        //     target_angular_velocity_ = msg->angular.z;
        //     log_file_ << "Received trajectory - Linear velocity: " << target_linear_velocity_
        //                 << ", Angular velocity: " << target_angular_velocity_ << std::endl;
        // }


        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
            // 从 IMU 消息获取角速度数据的 z 分量
            current_angular_velocity_ = msg->angular_velocity.z;
            
            // 将 IMU 角速度写入日志文件
            if (log_file_.is_open()) {
                const auto &ang = msg->angular_velocity;  // Use the full vector here.
                log_file_ << msg->header.stamp.sec << "." << std::setw(9) << std::setfill('0')
                          << msg->header.stamp.nanosec 
                          << " Received IMU Angular Velocity - ["
                          << ang.x << ", " << ang.y << ", " << ang.z << "]" << std::endl;
            }
        }
    
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
            if (msg == nullptr) {
                odom_received_ = false;
                return;
            }
            odom_received_ = true;
        
            // 原始传感器测量的位置（里程计数据）
            double sensor_x = msg->pose.pose.position.x;
            double sensor_y = msg->pose.pose.position.y;
            
            // 获取传感器测量的线速度（来自 odom）
            double sensor_linear_vel = msg->twist.twist.linear.x;
            
            // 从四元数中提取航向
            tf2::Quaternion quat;
            tf2::fromMsg(msg->pose.pose.orientation, quat);
            double roll, pitch;
            tf2::Matrix3x3(quat).getRPY(roll, pitch, current_yaw_);
            
            // 定义传感器相对于车辆运动学中心的偏移量（单位：米）
            // 例如：传感器在车辆运动学中心的右后侧：后移0.5米，右侧偏移0.2米
            double sensor_offset_x = -0.12;  // 负值表示后侧
            double sensor_offset_y = -0.07;  // 负值表示右侧（车辆坐标系中Y轴正向左）
        
            // 使用转换公式计算车辆运动学中心的实际位置
            current_position_x_ = sensor_x - (std::cos(current_yaw_) * sensor_offset_x - std::sin(current_yaw_) * sensor_offset_y);
            current_position_y_ = sensor_y - (std::sin(current_yaw_) * sensor_offset_x + std::cos(current_yaw_) * sensor_offset_y);
        
            // 对 odom 读取的线速度进行补偿
            // 公式： v_center = v_sensor + omega * d_y
            // 注意：此处使用从 IMU 得到的 current_angular_velocity_ 作为 ω
            current_linear_velocity_ = sensor_linear_vel + current_angular_velocity_ * sensor_offset_y;
            
            // 记录日志时输出修正后的位置信息与线速度
            log_file_ << "Received Odometry - Position: x=" << current_position_x_
                      << ", y=" << current_position_y_
                      << ", Linear Velocity: " << current_linear_velocity_
                      << std::endl;
            
            log_file_ << "Current yaw: " << current_yaw_ << " (roll: " << roll << ", pitch: " << pitch << ")" << std::endl;
        }
    
        void bspline_callback(const planner::msg::Bspline::SharedPtr msg) {
            std::string current_time = get_current_time_str();
        
            if (!msg) {
                log_file_ << "[" << current_time << "] B-spline message is null." << std::endl;
                receive_traj_ = false;
                return;
            }
            if (msg->pos_pts.empty() || msg->yaw_pts.empty() || msg->knots.empty()) {
                log_file_ << "[" << current_time << "] B-spline message missing pos_pts, yaw_pts 或 knots." << std::endl;
                receive_traj_ = false;
                return;
            }
        
            // 构造位置点矩阵
            Eigen::MatrixXd pos_pts(msg->pos_pts.size(), 3);
            for (size_t i = 0; i < msg->pos_pts.size(); i++) {
                pos_pts(i, 0) = msg->pos_pts[i].x;
                pos_pts(i, 1) = msg->pos_pts[i].y;
                pos_pts(i, 2) = msg->pos_pts[i].z;
            }
            // 构造 knots 向量
            Eigen::VectorXd knots(msg->knots.size());
            for (size_t i = 0; i < msg->knots.size(); i++) {
                knots(i) = msg->knots[i];
            }
        
            // 构造位置轨迹，时间分辨率设置为 0.1
            planner::NonUniformBspline pos_traj(pos_pts, msg->order, 0.1);
            pos_traj.setKnot(knots);
        
            // 构造 yaw 轨迹
            Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
            for (size_t i = 0; i < msg->yaw_pts.size(); i++) {
                yaw_pts(i, 0) = msg->yaw_pts[i];
            }
            planner::NonUniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);
        
            // 使用当前系统时间作为轨迹起始时间
            start_time_ = std::chrono::system_clock::now();
            traj_.clear();
            traj_.push_back(pos_traj);                    // traj_[0]：位置
            traj_.push_back(pos_traj.getDerivative());      // traj_[1]：速度
            traj_.push_back(traj_[1].getDerivative());        // traj_[2]：加速度
            traj_.push_back(yaw_traj);                      // traj_[3]：航向
            traj_.push_back(yaw_traj.getDerivative());        // traj_[4]：航向导数
        
            traj_duration_ = traj_[0].getTimeSum();
            receive_traj_ = true;
        
            log_file_ << "[" << current_time << "] Received Bspline message - pos_pts size: " 
                      << msg->pos_pts.size() << ", yaw_pts size: " << msg->yaw_pts.size() 
                      << ", order=" << msg->order << std::endl;
        
        
            // ----------------- 遍历轨迹并打印每个控制点的信息 -----------------
            log_file_ << "----- Control Points -----" << std::endl;
            double dt = 0.1;  // 采样时间间隔，可根据需要调整
            int index = 0;
            for (double t = 0; t <= traj_duration_; t += dt, index++) {
                Eigen::Vector3d p = traj_[0].evaluateDeBoor(t);
                Eigen::Vector3d v = traj_[1].evaluateDeBoor(t);
                double y = traj_[3].evaluateDeBoor(t)(0);
                double y_dot = traj_[4].evaluateDeBoor(t)(0);
                log_file_ << "Point[" << index << "]: time = " << t 
                          << ", pos = (" << p(0) << ", " << p(1) << ", " << p(2) << ")"
                          << ", vel = (" << v(0) << ", " << v(1) << ", " << v(2) << ")"
                          << ", yaw = " << y << ", yaw_dot = " << y_dot << std::endl;
            }
            log_file_ << "--------------------------" << std::endl;
        }
    
        // 控制循环：利用 B-spline 直接评估当前轨迹状态

        double previous_linear_cmd_ = 0.0;
        double previous_angular_cmd_ = 0.0;//1帧前线速度角速度命令

        void control_loop() {
            double current_time_sec = this->now().seconds();
            std::string current_time = get_current_time_str();
        
            if (!receive_traj_) {
                log_file_ << "-------------------------" << std::endl;
                log_file_ << "[" << current_time << "] No B-spline trajectory received." << std::endl;
                log_file_ << "-------------------------" << std::endl;
                return;
            }
        
            auto now_tp = std::chrono::system_clock::now();
            double t_diff = std::chrono::duration_cast<std::chrono::duration<double>>(now_tp - start_time_ ).count();
            
            //  // ----------------------------
            // // 距离前视：基于当前位置与轨迹点间距离选择目标点
            // // ----------------------------
            // double look_ahead_distance = 0.05;  // 前视距离（单位：米），可根据实际情况调节
            // double t_target = t_diff;          // 从当前时刻开始搜索
            // Eigen::Vector3d pos_target = traj_[0].evaluateDeBoor(t_target);
            // double dist = std::sqrt(std::pow(pos_target(0) - current_position_x_, 2) +
            //                         std::pow(pos_target(1) - current_position_y_, 2));
            // double dt_sample = 0.1;            // 采样时间间隔（秒），可调节以提高搜索精度

            // // 向后搜索，直到目标点与当前位置的距离达到预设前视距离
            // while (dist < look_ahead_distance && t_target < traj_duration_) {
            //     t_target += dt_sample;
            //     pos_target = traj_[0].evaluateDeBoor(t_target);
            //     dist = std::sqrt(std::pow(pos_target(0) - current_position_x_, 2) +
            //                     std::pow(pos_target(1) - current_position_y_, 2));
            // }
            // // 如果遍历到轨迹末尾仍未达到前视距离，则选择轨迹终点
            // if (t_target > traj_duration_) {
            //     t_target = traj_duration_;
            //     pos_target = traj_[0].evaluateDeBoor(t_target);
            // }

            // ----------------------------
            // 时间前视：基于当前位置与轨迹点间距离选择目标点
            // ----------------------------

            double look_ahead_time_offset = 0.5;  // 前视偏移量，单位秒
            double t_target = t_diff + look_ahead_time_offset;

            if (t_target > traj_duration_) {
                t_target = traj_duration_;
            }

            Eigen::Vector3d pos = traj_[0].evaluateDeBoor(t_target);
            Eigen::Vector3d vel = traj_[1].evaluateDeBoor(t_diff);
            double yaw = traj_[3].evaluateDeBoor(t_target)(0);
            double yaw_dot = traj_[4].evaluateDeBoor(t_target)(0);
        
            // 保证 t_diff 非负
            // if(t_diff < 0)
            //     t_diff = 0;
            
            // 如果 t_diff 超过轨迹持续时间，则机器人停止（直至下一段 Bspline 消息到来）
            if (t_diff > traj_duration_) {
                log_file_ << "-------------------------" << std::endl;
                log_file_ << "[" << get_current_time_str() << "] Trajectory finished (t_diff: " 
                            << t_diff << " > traj_duration: " << traj_duration_ << "), stopping robot." << std::endl;
                log_file_ << "-------------------------" << std::endl;
        
                geometry_msgs::msg::Twist cmd_msg;
                cmd_msg.linear.x  = 0.0;
                // cmd_msg.angular.z = 0.0;
                cmd_publisher_->publish(cmd_msg);
                return;// 在 control_loop() 开头部分后，对 t_diff 超过 traj_duration_ 的情况进行特殊处理
                if (t_diff > traj_duration_) {
                    // 获取B样条轨迹最后一个点的yaw
                    double final_yaw = traj_[3].evaluateDeBoor(traj_duration_)(0);
                    // 计算yaw误差，并归一化到[-pi, pi]
                    double yaw_error = final_yaw - current_yaw_;
                    while (yaw_error > M_PI)  yaw_error -= 2.0 * M_PI;
                    while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;
                    
                    // 定义一个yaw误差阈值（例如0.05弧度）
                    const double yaw_threshold = 0.05;
                    
                    if (std::fabs(yaw_error) > yaw_threshold) {
                        // 如果误差较大，则使用yaw的PID控制进行校正
                        double yaw_align_cmd = pid_orientation_control_.compute(yaw_error, 0);
                        geometry_msgs::msg::Twist cmd_msg;
                        cmd_msg.linear.x  = 0.0;  // 保持线速度为0
                        cmd_msg.angular.z = yaw_align_cmd;
                        cmd_publisher_->publish(cmd_msg);
                        log_file_ << "[" << get_current_time_str() << "] Aligning yaw: final_yaw = " 
                                  << final_yaw << ", current_yaw = " << current_yaw_ 
                                  << ", yaw_error = " << yaw_error 
                                  << ", yaw_align_cmd = " << yaw_align_cmd << std::endl;
                    } else {
                        // 当yaw对齐后，停止车辆并等待下一段Bspline消息
                        geometry_msgs::msg::Twist cmd_msg;
                        cmd_msg.linear.x  = 0.0;
                        cmd_msg.angular.z = 0.0;
                        cmd_publisher_->publish(cmd_msg);
                        log_file_ << "[" << get_current_time_str() << "] Yaw aligned (error " 
                                  << yaw_error << " < threshold), stopping robot." << std::endl;
                        // 可选：重置轨迹接收标志，等待下一段Bspline消息
                        receive_traj_ = false;
                    }
                    return;
                }
                
            }
            
        
            // 设置目标值（轨迹输出）
            target_x_ = pos(0);
            target_y_ = pos(1);
            // target_yaw_ = yaw;
            target_linear_velocity_ = vel(0);
            // target_angular_velocity_ = vel(1);
            target_angular_velocity_ = yaw_dot;
        
            // 计算位置误差
            double dx = target_x_ - current_position_x_;
            double dy = target_y_ - current_position_y_;
            double distance = std::sqrt(dx * dx + dy * dy);

        
            // 计算目标方向（用于反馈控制）
            double angle_to_target = std::atan2(dy, dx);

            target_yaw_ = angle_to_target;

            double alpha = angle_to_target - current_yaw_;
            while (alpha > M_PI)  alpha -= 2.0 * M_PI;
            while (alpha < -M_PI) alpha += 2.0 * M_PI;

            double abs_alpha = std::fabs(alpha);


            // 前向误差（仅用于日志输出）
            double e_forward = distance * std::cos(alpha);

            double pos_pid = pid_control_.compute(distance, 0);
            double norm_target_yaw = std::atan2(std::sin(target_yaw_), std::cos(target_yaw_));
            double norm_current_yaw = std::atan2(std::sin(current_yaw_), std::cos(current_yaw_));
            double orient_pid = pid_orientation_control_.compute(alpha, 0);
        
        
            // 添加可调权重参数（这里设置的初始值均为1.0，可根据需要调整或通过ROS参数加载）
            double weight_position = 1;    // 位置权重
            double weight_orientation = 1; // 朝向权重


            double error_linear_vel = current_linear_velocity_ - previous_linear_cmd_;
            double error_angular_vel = current_angular_velocity_ - previous_angular_cmd_;

            double weight_lin_vel = 0;  
            double weight_ang_vel = 0; 

            double linear_vel_pid = pid_velocity_control_.compute(error_linear_vel,0);
            double angular_vel_pid = pid_angular_velocity_control_.compute(error_angular_vel,0);


            double x_fraction_vel = vel(0);
            double y_fraction_vel = vel(1);

            double linear_cmd = 0 * sqrt(x_fraction_vel * x_fraction_vel + y_fraction_vel * y_fraction_vel) + weight_position * pos_pid;
            double angular_cmd = + weight_orientation * orient_pid;

            // 参数k可自行调节（比如1.0~5.0），k越大，转弯时速度衰减越猛烈
            double k = 1;  
            // 这个因子在alpha=0时为1, alpha越大越接近0
            double speed_scale = std::exp(-k * abs_alpha * abs_alpha); 
            // 再把它限制在[0.1, 1.0]之间，防止完全衰减到0
            if (speed_scale < 0.1) speed_scale = 0.1;

            linear_cmd *= speed_scale;
        
            // 限幅处理
            const double max_linear_speed = 1.0;
            const double max_angular_speed = 1;
            if (linear_cmd >  max_linear_speed)  linear_cmd =  max_linear_speed;
            if (linear_cmd < -max_linear_speed)  linear_cmd = -max_linear_speed;
            if (angular_cmd >  max_angular_speed)  angular_cmd =  max_angular_speed;
            if (angular_cmd < -max_angular_speed)  angular_cmd = -max_angular_speed;


            previous_linear_cmd_ = linear_cmd;
            previous_angular_cmd_ = angular_cmd;
            





        
            // 以下输出日志格式保持不变
            std::string position_str = (current_linear_velocity_ != 0 || current_angular_velocity_ != 0)
                ? ("x: " + std::to_string(current_position_x_) + ", y: " + std::to_string(current_position_y_))
                : "NA";
            std::string yaw_str = (current_linear_velocity_ != 0 || current_angular_velocity_ != 0)
                ? ("Current Yaw: " + std::to_string(current_yaw_))
                : "NA";
            std::string target_position_str = receive_traj_
                ? ("Target Position - x: " + std::to_string(target_x_) + ", y: " + std::to_string(target_y_))
                : "NA";
            std::string target_yaw_str = receive_traj_
                ? ("Target Yaw: " + std::to_string(target_yaw_))
                : "NA";
            std::string actual_linear_velocity_str = (current_linear_velocity_ != 0) ? std::to_string(current_linear_velocity_) : "NA";
            std::string actual_angular_velocity_str = (current_angular_velocity_ != 0) ? std::to_string(current_angular_velocity_) : "NA";
        
            log_file_ << "-------------------------" << std::endl;
            log_file_ << "[" << get_current_time_str() << "] Selected control point: "
                        << "(x: " << target_x_ << ", y: " << target_y_ 
                        << ", yaw: " << target_yaw_ << ")" << std::endl;
            log_file_ << "Time diff: " <<  t_diff  << std::endl;       
            log_file_ << "-- PID Output: Linear Velocity: " << linear_cmd
                        << ", Angular Velocity: " << angular_cmd << std::endl;
            log_file_ << "Linear Velocity Target: " << linear_cmd
                        << ", Angular Velocity Target: " << angular_cmd << std::endl;
            log_file_ << target_position_str << ", " << target_yaw_str << std::endl;
            log_file_ << "Current Position - " << position_str << ", " << yaw_str << std::endl;
            log_file_ << "--Distance to Target (abs): " << distance << std::endl;
            log_file_ << "--Position ERROR (signed): " << e_forward << std::endl;
            log_file_ << "--Orientation ERROR: " << alpha << std::endl;
            log_file_ << "Current Linear Velocity: " << actual_linear_velocity_str << std::endl;
            log_file_ << "Current Angular Velocity: " << actual_angular_velocity_str << std::endl;
            log_file_ << "--Linear Velocity ERROR: " << error_linear_vel << std::endl;
            log_file_ << "--Angular Velocity ERROR: " << error_angular_vel << std::endl;
            log_file_ << "-------------------------" << std::endl;
        
            // 发布控制指令
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