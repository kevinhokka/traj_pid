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
      // 原先在此写死了 PID 参数，这里先初始化为 0，然后从配置文件读完后再赋值
      pid_position_control_(0.0, 0.0, 0.0),
      pid_orientation_control_(0.0, 0.0, 0.0),
      pid_velocity_control_(0.0, 0.0, 0.0),
      pid_angular_velocity_control_(0.0, 0.0, 0.0),

      target_linear_velocity_(0.0), 
      target_angular_velocity_(0.0),
      current_linear_velocity_(0.0), 
      current_angular_velocity_(0.0),
      current_position_x_(0.0), 
      current_position_y_(0.0), 
      current_yaw_(0.0),
      target_x_(0.0), 
      target_y_(0.0), 
      target_yaw_(0.0)
    {
        // 1) 先读取配置文件中的所有参数
        readParametersFromFile();

        // 2) 根据读取到的参数，初始化各 PID 控制器
        pid_position_control_       = PIDController(param_position_p_, param_position_i_, param_position_d_);
        pid_orientation_control_    = PIDController(param_orientation_p_, param_orientation_i_, param_orientation_d_);
        pid_velocity_control_       = PIDController(param_velocity_p_, param_velocity_i_, param_velocity_d_);
        pid_angular_velocity_control_ = PIDController(param_angular_velocity_p_, param_angular_velocity_i_, param_angular_velocity_d_);

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
        log_file_ << "Position PID: " << pid_position_control_.get_parameters_str() << std::endl;
        log_file_ << "Orientation PID: " << pid_orientation_control_.get_parameters_str() << std::endl;
        log_file_ << "Linear Velocity PID: " << pid_velocity_control_.get_parameters_str() << std::endl;
        log_file_ << "Angular Velocity PID: " << pid_angular_velocity_control_.get_parameters_str() << std::endl;
        log_file_ << "-------------------------" << std::endl;

        RCLCPP_INFO(this->get_logger(), "Logging to file: %s", log_filename.c_str());

        cboard_odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom_CBoard", 10,
            std::bind(&TrajPidNode::cboard_odom_callback, this, std::placeholders::_1));
        
        // 外部imu订阅
        external_imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data_raw", 10,
            std::bind(&TrajPidNode::external_imu_callback, this, std::placeholders::_1));
        

        // odometry 订阅
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/fastlio2/lio_odom", 10, 
            std::bind(&TrajPidNode::odom_callback, this, std::placeholders::_1));

        // IMU 订阅
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/livox/imu", 10, 
            std::bind(&TrajPidNode::imu_callback, this, std::placeholders::_1));

        // B-spline 订阅
        bspline_subscription_ = this->create_subscription<planner::msg::Bspline>(
            "/bspline", 10, 
            std::bind(&TrajPidNode::bspline_callback, this, std::placeholders::_1));

        // 控制指令发布
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "PID 轨迹跟踪节点已启动");

        // 输出PID控制器参数（打印到控制台）
        pid_position_control_.print_parameters();
        pid_orientation_control_.print_parameters();
        pid_velocity_control_.print_parameters();
        pid_angular_velocity_control_.print_parameters();

        // 定时器，每 100ms 调一次控制循环
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TrajPidNode::control_loop, this));
    }

    ~TrajPidNode() {
        if (log_file_.is_open()) {
            log_file_.close();  // 确保文件在节点退出时被关闭
        }
    }

private:
    // ==================== 新增：从文本文件读取参数 ====================
    void readParametersFromFile() {
        // 默认情况下，从当前包的 src 文件夹下读取
        // 注意修改成你自己在仓库中的绝对/相对路径
        // std::string config_file = 
        //     std::string(std::filesystem::current_path()) + "/traj_pid_config.txt";

        // 若想固定读取路径，比如就在 traj_pid/src 下，可自行替换：
        std::string config_file = "/home/jetson/ros2_ws/src/traj_pid/src/traj_pid_config.txt";

        std::ifstream fin(config_file);
        if (!fin.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "无法打开配置文件: %s", config_file.c_str());
            throw std::runtime_error("无法打开配置文件");
        }

        // 用一个 map 存储键值对
        std::map<std::string, double> param_map;
        std::string line;
        while (std::getline(fin, line)) {
            // 忽略空行和注释行
            if (line.empty() || line[0] == '#') continue;

            // 解析格式: key=value
            std::stringstream ss(line);
            std::string key;
            double value;
            if (std::getline(ss, key, '=') && (ss >> value)) {
                param_map[key] = value;
            }
        }
        fin.close();

        // 从 map 中取值，如果没取到就用默认值
        auto get_val = [&](const std::string &k, double default_val) {
            return (param_map.find(k) != param_map.end()) ? param_map[k] : default_val;
        };

        // 位置 PID
        param_position_p_ = get_val("pid_position_p", 1.0);
        param_position_i_ = get_val("pid_position_i", 0.0);
        param_position_d_ = get_val("pid_position_d", 1.0);

        // 朝向 PID
        param_orientation_p_ = get_val("pid_orientation_p", 2.46);
        param_orientation_i_ = get_val("pid_orientation_i", 0.0);
        param_orientation_d_ = get_val("pid_orientation_d", 4.0);

        // 线速度 PID
        param_velocity_p_ = get_val("pid_velocity_p", 0.75);
        param_velocity_i_ = get_val("pid_velocity_i", 0.0);
        param_velocity_d_ = get_val("pid_velocity_d", 1.0);

        // 角速度 PID
        param_angular_velocity_p_ = get_val("pid_angular_velocity_p", 1.0);
        param_angular_velocity_i_ = get_val("pid_angular_velocity_i", 0.0);
        param_angular_velocity_d_ = get_val("pid_angular_velocity_d", 0.0);

        // 其他控制参数
        param_k_                   = get_val("k", 2.0);
        param_look_ahead_time_offset_ = get_val("look_ahead_time_offset", 0.5);
        param_alpha_threshold_     = get_val("alpha_threshold", M_PI / 2.0);
        param_max_linear_speed_    = get_val("max_linear_speed", 0.6);
        param_max_angular_speed_   = get_val("max_angular_speed", 1.0);

        // 机器人 / 轨迹相关参数
        param_sensor_offset_x_     = get_val("sensor_offset_x", -0.12);
        param_sensor_offset_y_     = get_val("sensor_offset_y", -0.07);
        param_distance_threshold_  = get_val("distance_threshold", 0.3);
        param_dt_                  = get_val("dt", 0.1);
    }
    // ===================================================================

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

    // ========== 新增：读取到的配置参数存储 ==========
    double param_position_p_;
    double param_position_i_;
    double param_position_d_;

    double param_orientation_p_;
    double param_orientation_i_;
    double param_orientation_d_;

    double param_velocity_p_;
    double param_velocity_i_;
    double param_velocity_d_;

    double param_angular_velocity_p_;
    double param_angular_velocity_i_;
    double param_angular_velocity_d_;

    double param_k_;
    double param_look_ahead_time_offset_;
    double param_alpha_threshold_;
    double param_max_linear_speed_;
    double param_max_angular_speed_;

    double param_sensor_offset_x_;
    double param_sensor_offset_y_;
    double param_distance_threshold_;
    double param_dt_;
    // =============================================

    // ===== CBoard 里程计相关 =====
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr cboard_odom_subscription_;
    double roll_CBoard_{0.0};
    double pitch_CBoard_{0.0};
    double current_yaw_CBoard_{0.0};

    // ===== 外部 IMU 相关 =====
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr external_imu_subscription_;
    bool   external_imu_initialized_{false};
    double init_roll_IMU_{0.0};
    double init_pitch_IMU_{0.0};
    double init_yaw_IMU_{0.0};
    double roll_IMU_{0.0};
    double pitch_IMU_{0.0};
    double yaw_IMU_{0.0};


    // 声明成员变量
    PIDController pid_position_control_;         // PID 控制器（位置）
    PIDController pid_orientation_control_;      // PID 控制器（朝向）
    PIDController pid_velocity_control_;         // PID 控制器（线速度）
    PIDController pid_angular_velocity_control_; // PID 控制器（角速度）

    std::ofstream log_file_;  // 日志文件
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr traj_subscription_;  // （未使用，可删除或保留）
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;  
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<planner::msg::Bspline>::SharedPtr bspline_subscription_;  
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;  
    rclcpp::TimerBase::SharedPtr control_timer_;  

    // 用于B-spline轨迹的成员变量
    std::vector<planner::NonUniformBspline> traj_;
    std::chrono::time_point<std::chrono::system_clock> start_time_;
    double traj_duration_;
    bool receive_traj_;

    // 状态变量
    bool bspline_received_ = false;
    bool odom_received_ = false;
    bool imu_received_ = false;  // 新增：IMU消息接收状态
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

    // MID360 IMU 回调函数
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        if (!msg) {
            imu_received_ = false;
            return;
        }
        imu_received_ = true;
        // Get the z component of angular velocity from the IMU message
        current_angular_velocity_ = msg->angular_velocity.z;

        // Write the IMU angular velocity to the log file
        if (log_file_.is_open()) {
            const auto &ang = msg->angular_velocity;  // Use the full angular velocity vector
            // log_file_ << msg->header.stamp.sec << "." << std::setw(9) << std::setfill('0')
            //           << msg->header.stamp.nanosec 
            //           << " Received IMU Angular Velocity - ["
            //           << ang.x << ", " << ang.y << ", " << ang.z << "]" << std::endl;
        }
    }

    // 记录第一次姿态以做零点
    bool   cboard_odom_initialized_{false};
    double init_roll_CBoard_{0.0};
    double init_pitch_CBoard_{0.0};
    double init_yaw_CBoard_{0.0};

    // CBoard 里程计回调
    void cboard_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!msg) return;

        tf2::Quaternion q;
        tf2::fromMsg(msg->pose.pose.orientation, q);
        tf2::Matrix3x3 m(q);

        double raw_roll, raw_pitch, raw_yaw;
        m.getRPY(raw_roll, raw_pitch, raw_yaw);

        // 第一次收到消息时，记录为零点
        if (!cboard_odom_initialized_) {
            init_roll_CBoard_  = raw_roll;
            init_pitch_CBoard_ = raw_pitch;
            init_yaw_CBoard_   = raw_yaw;
            cboard_odom_initialized_ = true;
        }

        // 计算相对姿态
        roll_CBoard_  = raw_roll  - init_roll_CBoard_;
        pitch_CBoard_ = raw_pitch - init_pitch_CBoard_;
        current_yaw_CBoard_ = raw_yaw - init_yaw_CBoard_;

        // 将 yaw 归一化到 [-pi, pi]
        while (current_yaw_CBoard_ >  M_PI) current_yaw_CBoard_ -= 2.0 * M_PI;
        while (current_yaw_CBoard_ < -M_PI) current_yaw_CBoard_ += 2.0 * M_PI;

        if (log_file_.is_open()) {
            log_file_ << "[CBoard Odom] roll_rel="  << roll_CBoard_
                    << ", pitch_rel="            << pitch_CBoard_
                    << ", yaw_rel="              << current_yaw_CBoard_ << std::endl;
        }
    }

    // 外部 IMU 回调
    void external_imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        if (!msg) return;

        tf2::Quaternion q;
        tf2::fromMsg(msg->orientation, q);
        tf2::Matrix3x3 m(q);

        double raw_roll, raw_pitch, raw_yaw;
        m.getRPY(raw_roll, raw_pitch, raw_yaw);

        // 首次收到时设为零点
        if (!external_imu_initialized_) {
            init_roll_IMU_  = raw_roll;
            init_pitch_IMU_ = raw_pitch;
            init_yaw_IMU_   = raw_yaw;
            external_imu_initialized_ = true;
        }

        // 计算相对姿态
        roll_IMU_  = raw_roll  - init_roll_IMU_;
        pitch_IMU_ = raw_pitch - init_pitch_IMU_;
        yaw_IMU_   = raw_yaw   - init_yaw_IMU_;

        // 归一化 yaw 至 [-π, π]
        while (yaw_IMU_ >  M_PI) yaw_IMU_ -= 2.0 * M_PI;
        while (yaw_IMU_ < -M_PI) yaw_IMU_ += 2.0 * M_PI;

        if (log_file_.is_open()) {
            log_file_ << "[Ext IMU] roll_rel="  << roll_IMU_
                    << ", pitch_rel="        << pitch_IMU_
                    << ", yaw_rel="          << yaw_IMU_ << std::endl;
        }
    }


    // ODOM 回调函数
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (msg == nullptr) {
            odom_received_ = false;
            return;
        }
        odom_received_ = true;

        // Raw sensor measurement position (odometry data)
        double sensor_x = msg->pose.pose.position.x;
        double sensor_y = msg->pose.pose.position.y;

        // Get sensor measured linear velocity (from odom)
        double sensor_linear_vel = msg->twist.twist.linear.x;

        // Extract yaw from the quaternion
        tf2::Quaternion quat;
        tf2::fromMsg(msg->pose.pose.orientation, quat);
        double roll, pitch;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, current_yaw_);

        // 使用配置文件中的偏移
        double sensor_offset_x = param_sensor_offset_x_;
        double sensor_offset_y = param_sensor_offset_y_;

        // Calculate the actual position of the vehicle kinematic center
        current_position_x_ = sensor_x - (std::cos(current_yaw_) * sensor_offset_x 
                                        - std::sin(current_yaw_) * sensor_offset_y);
        current_position_y_ = sensor_y - (std::sin(current_yaw_) * sensor_offset_x 
                                        + std::cos(current_yaw_) * sensor_offset_y);

        // Compensate the linear velocity read from odom
        current_linear_velocity_ = sensor_linear_vel + current_angular_velocity_ * sensor_offset_y;

        // Log the corrected position and linear velocity
        log_file_ << "Received Odometry - Position: x=" << current_position_x_
                  << ", y=" << current_position_y_
                  << ", Linear Velocity: " << current_linear_velocity_
                  << std::endl;

        log_file_ << "Current yaw: " << current_yaw_ << " (roll: " << roll << ", pitch: " << pitch << ")" << std::endl;
    }

    // B-spline 回调函数
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

        // 构造位置轨迹，时间分辨率设置为 param_dt_
        planner::NonUniformBspline pos_traj(pos_pts, msg->order, param_dt_);
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
        traj_.push_back(pos_traj);                       // traj_[0]：位置
        traj_.push_back(pos_traj.getDerivative());       // traj_[1]：速度
        traj_.push_back(traj_[1].getDerivative());       // traj_[2]：加速度
        traj_.push_back(yaw_traj);                       // traj_[3]：航向
        traj_.push_back(yaw_traj.getDerivative());       // traj_[4]：航向导数

        traj_duration_ = traj_[0].getTimeSum();
        receive_traj_ = true;

        log_file_ << "[" << current_time << "] Received Bspline message - pos_pts size: " 
                  << msg->pos_pts.size() << ", yaw_pts size: " << msg->yaw_pts.size() 
                  << ", order=" << msg->order << std::endl;

        // ----------------- 遍历轨迹并打印每个控制点的信息 -----------------
        log_file_ << "----- Control Points -----" << std::endl;
        double dt = param_dt_;  // 采样时间间隔（从配置文件读取）
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

    // 控制循环
    double previous_linear_cmd_ = 0.0;
    double previous_angular_cmd_ = 0.0; // 1帧前线速度角速度命令

    void control_loop() {
        std::string current_time = get_current_time_str();

        // if (!odom_received_ || !imu_received_) {
        //    log_file_  <<  "未接收到 odometry 或 IMU 消息，车辆停止。" << std::endl;
        //    geometry_msgs::msg::Twist cmd_msg;
        //    cmd_msg.linear.x = 0.0;
        //    cmd_msg.angular.z = 0.0;
        //    cmd_publisher_->publish(cmd_msg);
        //    return;
        // }

        if (!receive_traj_) {
            log_file_ << "-------------------------" << std::endl;
            log_file_ << "[" << current_time << "] No B-spline trajectory received." << std::endl;
            log_file_ << "-------------------------" << std::endl;
            return;
        }

        auto now_tp = std::chrono::system_clock::now();
        double t_diff = std::chrono::duration_cast<std::chrono::duration<double>>(now_tp - start_time_ ).count();

        // 如果 t_diff 超过轨迹持续时间，则机器人停止并对齐yaw
        if (t_diff > traj_duration_) {
            // 这里为了演示直接 yaw 对齐后停止
            double final_yaw = 0.0; // 也可: traj_[3].evaluateDeBoor(traj_duration_)(0)
            // double yaw_error = final_yaw - current_yaw_;
            double yaw_error = final_yaw - current_yaw_CBoard_;
            // double yaw_error = final_yaw - yaw_IMU_;

            while (yaw_error > M_PI)  yaw_error -= 2.0 * M_PI;
            while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;

            const double yaw_threshold = 0.05;
            if (std::fabs(yaw_error) > yaw_threshold) {
                double yaw_align_cmd = pid_orientation_control_.compute(yaw_error, 0);
                geometry_msgs::msg::Twist cmd_msg;
                cmd_msg.linear.x  = 0.0;  
                cmd_msg.angular.z = yaw_align_cmd;
                cmd_publisher_->publish(cmd_msg);
                log_file_ << "[" << get_current_time_str() << "] Aligning yaw: final_yaw = " 
                          << final_yaw << ", yaw_IMU_ = " << yaw_IMU_ 
                          << ", yaw_error = " << yaw_error 
                          << ", yaw_align_cmd = " << yaw_align_cmd << std::endl;
            } else {
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

        // 时间前视
        double t_target = t_diff + param_look_ahead_time_offset_;
        if (t_target > traj_duration_) {
            t_target = traj_duration_;
        }

        Eigen::Vector3d pos = traj_[0].evaluateDeBoor(t_target);
        Eigen::Vector3d vel = traj_[1].evaluateDeBoor(t_diff);
        double yaw = traj_[3].evaluateDeBoor(t_target)(0);
        double yaw_dot = traj_[4].evaluateDeBoor(t_target)(0);

        target_x_ = pos(0);
        target_y_ = pos(1);
        target_linear_velocity_ = vel(0);
        target_angular_velocity_ = yaw_dot;

        // 计算位置误差
        double dx = target_x_ - current_position_x_;
        double dy = target_y_ - current_position_y_;
        double distance = std::sqrt(dx * dx + dy * dy);
        
        double angle_to_target = std::atan2(dy, dx);
        target_yaw_ = angle_to_target;

        // double alpha = angle_to_target - current_yaw_;
        double alpha = angle_to_target - current_yaw_CBoard_;
        // double alpha = angle_to_target - yaw_IMU_;

        while (alpha > M_PI)  alpha -= 2.0 * M_PI;
        while (alpha < -M_PI) alpha += 2.0 * M_PI;
        double abs_alpha = std::fabs(alpha);

        // 计算轨迹起点到目标点的总距离（示例中未使用到，可自行处理）
        double ori_x = 0.0;
        double ori_y = 0.0;
        double ori_dx = target_x_ - ori_x;
        double ori_dy = target_y_ - ori_y;
        double traj_distance = std::sqrt(ori_dx * ori_dx + ori_dy * ori_dy);

        // 轨迹较小的时候，避免 yaw 跳变
        if (traj_distance < param_distance_threshold_) {
            target_yaw_ = yaw; 
            alpha = 0.0;
        }

        double e_forward = distance * std::cos(alpha);
        double pos_pid = pid_position_control_.compute(distance, 0);
        double orient_pid = pid_orientation_control_.compute(alpha, 0);

        double error_linear_vel = current_linear_velocity_ - previous_linear_cmd_;
        double error_angular_vel = current_angular_velocity_ - previous_angular_cmd_;

        double linear_vel_pid = pid_velocity_control_.compute(error_linear_vel, 0);
        double angular_vel_pid = pid_angular_velocity_control_.compute(error_angular_vel, 0);

        // 仅用于示例，不混合速度 PID，还是按原逻辑 pos_pid / orient_pid 来做
        // 线速度
        double x_fraction_vel = vel(0);
        double y_fraction_vel = vel(1);
        double linear_cmd = 0.0 * std::sqrt(x_fraction_vel * x_fraction_vel + y_fraction_vel * y_fraction_vel) 
                            + pos_pid;
        // 角速度
        double angular_cmd = orient_pid;

        // // 当航向误差过大时，线速度置 0，仅用角速度对齐
        // if (std::fabs(alpha) > param_alpha_threshold_) {
        //     linear_cmd = 0.0;
        // }

        // 根据转角衰减线速度
        double speed_scale = std::exp(-param_k_ * abs_alpha * abs_alpha);
        linear_cmd *= speed_scale;

        // static bool aligning = false;                    // 只在本函数记一次状态

        // // 触发对齐：|alpha| 大于外层阈值时进入
        // if (!aligning && std::fabs(alpha) > param_alpha_threshold_) {
        //     aligning = true;
        // }

        // // 对齐执行：对齐中始终用低速 0.1
        // if (aligning) {
        //     linear_cmd = 0.1;                            // 低速缓行
        //     if (std::fabs(alpha) < 0.05) {               // 误差≤0.05 rad 认为对齐完成
        //         aligning = false;                        // 退出对齐
        //     }
        // }

        // 限幅
        if (linear_cmd > param_max_linear_speed_)  linear_cmd = param_max_linear_speed_;
        if (linear_cmd < -param_max_linear_speed_) linear_cmd = -param_max_linear_speed_;
        if (angular_cmd > param_max_angular_speed_)  angular_cmd = param_max_angular_speed_;
        if (angular_cmd < -param_max_angular_speed_) angular_cmd = -param_max_angular_speed_;

        previous_linear_cmd_ = linear_cmd;
        previous_angular_cmd_ = angular_cmd;

        // 以下输出日志格式保持一致
        std::string position_str = (current_linear_velocity_ != 0 || current_angular_velocity_ != 0)
            ? ("x: " + std::to_string(current_position_x_) + ", y: " + std::to_string(current_position_y_))
            : "NA";
        // std::string yaw_str = (current_linear_velocity_ != 0 || current_angular_velocity_ != 0)
        //     ? ("Current Yaw: " + std::to_string(current_yaw_))
        //     : "NA";

        std::string yaw_str = (current_linear_velocity_ != 0 || current_angular_velocity_ != 0)
            ? ("Current Yaw: " + std::to_string(current_yaw_CBoard_))
            : "NA";
        // std::string yaw_str = (current_linear_velocity_ != 0 || current_angular_velocity_ != 0)
        //     ? ("Current Yaw: " + std::to_string(yaw_IMU_))
        //     : "NA";
        std::string target_position_str = receive_traj_
            ? ("Target Position - x: " + std::to_string(target_x_) + ", y: " + std::to_string(target_y_))
            : "NA";
        std::string target_yaw_str = receive_traj_
            ? ("Target Yaw: " + std::to_string(target_yaw_))
            : "NA";
        std::string actual_linear_velocity_str = (current_linear_velocity_ != 0) 
            ? std::to_string(current_linear_velocity_) : "NA";
        std::string actual_angular_velocity_str = (current_angular_velocity_ != 0) 
            ? std::to_string(current_angular_velocity_) : "NA";

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
