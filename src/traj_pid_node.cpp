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
        pid_control_(4.7932, 0.0010, 0.0010),  // 初始化 PID 控制器（位置）
        pid_orientation_control_(2.4884, 0.0010, 0.0010),  // 初始化朝向 PID 控制器

        //   pid_velocity_control_(2, 0.5, 0.1),  // 初始化线速度 PID 控制器
        //   pid_angular_velocity_control_(3, 0.8, 0.2),  // 初始化角速度 PID 控制器

        pid_velocity_control_(0.0010, 9.9736, 0.0010),  // 初始化线速度 PID 控制器
        pid_angular_velocity_control_(0.0010, 8.8820, 0.1125),  // 初始化角速度 PID 控制器


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
            if (msg->pos_pts.empty() || msg->knots.empty()) {
                log_file_ << "[" << current_time << "] B-spline message missing pos_pts or knots." << std::endl;
                bspline_received_ = false;
                return;
            }
            if (msg->yaw_pts.empty()) {
                log_file_ << "[" << current_time << "] B-spline message missing yaw_pts." << std::endl;
                bspline_received_ = false;
                return;
            }

            bspline_received_ = true;
            log_file_ << "[" << current_time << "] Received Bspline message - pos_pts size: " 
                    << msg->pos_pts.size() << ", yaw_pts size: " << msg->yaw_pts.size() 
                    << ", order=" << msg->order << std::endl;

            // 2. 把消息中的位置控制点 pos_pts 转成 Eigen 矩阵
            int n_pos = static_cast<int>(msg->pos_pts.size());
            Eigen::MatrixXd pos_pts(n_pos, 3);
            for (int i = 0; i < n_pos; i++) {
                pos_pts(i,0) = msg->pos_pts[i].x;
                pos_pts(i,1) = msg->pos_pts[i].y;
                pos_pts(i,2) = msg->pos_pts[i].z;
            }

            // 3. 构造 NonUniformBspline 对象 (位置)
            //    第三个参数 dt 先给个占位值，比如0.1
            planner::NonUniformBspline pos_bspline(pos_pts, msg->order, 0.1);

            // 4. 拷贝 knots 到 Eigen::VectorXd 再 setKnot
            int n_knots = static_cast<int>(msg->knots.size());
            Eigen::VectorXd eigen_knots(n_knots);
            for (int i = 0; i < n_knots; i++) {
                eigen_knots(i) = msg->knots[i];
            }
            pos_bspline.setKnot(eigen_knots);

            // 5. 获取 yaw_dt
            double yaw_dt = msg->yaw_dt;
            int n_yaw = static_cast<int>(msg->yaw_pts.size());

            // 6. 清空原 control_points_ 容器
            control_points_.clear();

            // 7. 遍历 knots，计算每个时刻 t=knots[i] 时的位置
            //    并用 (t / yaw_dt) 找到航向角 yaw
            // 注意三次B样条常在首尾各重复 3 次 knot，你可自行决定跳过
            // 这里简单示例全部遍历
            for (int i = 0; i < n_knots; i++) {
                double t = eigen_knots(i);

                // Evaluate 轨迹在时刻 t 的 (x,y,z)
                // 如果 t 不在 [knot(0), knot(end)] 范围，也可能要额外处理
                Eigen::Vector3d pos_xyz;
                try {
                    pos_xyz = pos_bspline.evaluateDeBoorT(t);
                } catch(...) {
                    // 避免越界/异常
                    continue;
                }

                // 根据 t 算 yaw 的索引
                int j = static_cast<int>( std::floor(t / yaw_dt) );
                if      (j < 0)       j = 0;
                else if (j >= n_yaw)  j = n_yaw - 1;

                double yaw_val = msg->yaw_pts[j];

                // 构造 ControlPoint
                ControlPoint cp;
                cp.x    = pos_xyz.x();
                cp.y    = pos_xyz.y();
                cp.yaw  = yaw_val;
                cp.time = cp.time = current_time_sec + t;  

                control_points_.push_back(cp);
            }

            // 8. 打印调试信息
            log_file_ << "[" << current_time << "] Constructed " 
                    << control_points_.size() << " control points from B-spline." << std::endl;
            for (size_t i = 0; i < control_points_.size(); i++) {
                log_file_ << "  Pt[" << i << "] (x=" << control_points_[i].x 
                        << ", y=" << control_points_[i].y 
                        << ", yaw=" << control_points_[i].yaw
                        << ", t=" << control_points_[i].time - current_time_sec << ")" << std::endl;
            }

            // 9. 若需要，把第一个控制点当作当前 target
            if (!control_points_.empty()) {
                target_x_   = control_points_[0].x;
                target_y_   = control_points_[0].y;
                target_yaw_ = control_points_[0].yaw;
            }
        }
    
        void control_loop() 
        {
            double current_time_sec = this->now().seconds();
            std::string current_time = get_current_time_str();

            // 若无控制点，直接返回
            if (control_points_.empty()) {
                log_file_ << "-------------------------" << std::endl;
                log_file_ << "-------------------------" << std::endl;
                log_file_ << "[" << get_current_time_str() << "] Selected control point: NA" << std::endl;
                log_file_ << "&&&&&& TIMESTAMP: &&&&&&&" << get_current_time_str() << std::endl;
                log_file_ << "-------------------------" << std::endl;
                log_file_ << "-------------------------" << std::endl;
                return;
            }

            // ===== 外环控制：根据B样条获得目标点，计算位置和姿态误差 =====
            // 从control_points_中找到当前时刻要跟踪的目标控制点
            ControlPoint target_cp;
            bool target_found = false;
            for (size_t i = 0; i < control_points_.size(); ++i) {
                if (control_points_[i].time > current_time_sec) {
                    target_cp = control_points_[i];
                    target_found = true;
                    break;
                }
            }
            if (!target_found) {
                target_cp = control_points_.back();
            }
            // 更新全局目标变量（后续日志依赖它们）
            target_x_   = target_cp.x;
            target_y_   = target_cp.y;
            target_yaw_ = target_cp.yaw;

            // 计算位置误差（dx, dy、距离）和朝向误差
            double dx = target_x_ - current_position_x_;
            double dy = target_y_ - current_position_y_;
            double distance = std::sqrt(dx * dx + dy * dy);
            double angle_to_target = std::atan2(dy, dx);
            double alpha = angle_to_target - current_yaw_;
            if (alpha > M_PI)  alpha -= 2.0 * M_PI;
            else if (alpha < -M_PI) alpha += 2.0 * M_PI;
            double e_forward = distance * std::cos(alpha);

            // 外环PID：利用几何/姿态误差计算期望速度（v_d, w_d）
            double v_outer = pid_control_.compute(e_forward, 0.0);
            double w_outer = pid_orientation_control_.compute(alpha, 0.0);

            // ===== 内环控制：基于目标与实际速度差异补偿 =====
            // 速度误差（这里仍保留原来由/traj话题设置的目标速度）
            double velocity_error = target_linear_velocity_ - current_linear_velocity_;
            double angular_velocity_error = target_angular_velocity_ - current_angular_velocity_;

            // 内环PID：对速度误差进行跟踪
            double v_inner = pid_velocity_control_.compute(velocity_error, 0.0);
            double w_inner = pid_angular_velocity_control_.compute(angular_velocity_error, 0.0);

            // 将外环和内环的输出按一定权重组合（此处权重可调）
            const double position_weight = 0;
            const double velocity_weight = 0.6546;
            const double orientation_weight = 1;
            const double angular_velocity_weight = 0.2424;
            double pid_linear_output  = v_outer * position_weight + v_inner * velocity_weight;
            double pid_angular_output = w_outer * orientation_weight + w_inner * angular_velocity_weight;

            // 在角速度上加一简单滑模补偿（示例：积分+符号修正，防止抖振可做边界层处理）
            static double w_sliding_integral = 0.0;
            w_sliding_integral += angular_velocity_error * 0.1; // 10Hz采样周期
            double sliding_gain = 0.1;
            double sliding_term = sliding_gain * (w_outer - current_angular_velocity_ + 0.01 * w_sliding_integral);
            pid_angular_output += sliding_term;

            // 限幅处理
            const double max_linear_speed = 2.0;
            if (pid_linear_output >  max_linear_speed)  pid_linear_output =  max_linear_speed;
            if (pid_linear_output < -max_linear_speed)  pid_linear_output = -max_linear_speed;
            const double max_angular_speed = 0.5;
            if (pid_angular_output >  max_angular_speed)  pid_angular_output =  max_angular_speed;
            if (pid_angular_output < -max_angular_speed)  pid_angular_output = -max_angular_speed;

            // ===== 日志记录部分【格式保持不变】=====
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

            // 以下日志格式与原代码完全一致，不做任何调整
            log_file_ << "-------------------------" << std::endl;
            log_file_ << "-------------------------" << std::endl;
            log_file_ << "[" << get_current_time_str() << "] Selected control point: "
                    << "(x: " << target_x_ << ", y: " << target_y_ << ", yaw: " << target_yaw_ << ")"
                    << std::endl;
            log_file_ << "&&&&&& TIMESTAMP: &&&&&&&" << get_current_time_str() << std::endl;
            log_file_ << "-- PID Output: Linear Velocity: " << pid_linear_output
                    << ", Angular Velocity: " << pid_angular_output << std::endl;
            log_file_ << "Linear Velocity Target: " << target_linear_velocity_
                    << ", Angular Velocity Target: " << target_angular_velocity_ << std::endl;
            log_file_ << target_position_str << ", " << target_yaw_str << std::endl;
            log_file_ << "Current Position - " << position_str << ", " << yaw_str << std::endl;
            log_file_ << "--Distance to Target (abs): " << distance << std::endl;
            log_file_ << "--Position ERROR (signed): " << e_forward << std::endl;
            log_file_ << "--Orientation ERROR: " << alpha << std::endl;
            log_file_ << "Current Linear Velocity: " << actual_linear_velocity_str << std::endl;
            log_file_ << "Current Angular Velocity: " << actual_angular_velocity_str << std::endl;
            log_file_ << "--Linear Velocity ERROR: " << velocity_error << std::endl;
            log_file_ << "--Angular Velocity ERROR: " << angular_velocity_error << std::endl;
            log_file_ << "-------------------------" << std::endl;
            log_file_ << "-------------------------" << std::endl;

            // ===== 最后发布控制指令 =====
            geometry_msgs::msg::Twist cmd_msg;
            cmd_msg.linear.x  = pid_linear_output;
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