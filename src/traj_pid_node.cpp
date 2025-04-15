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

#include <deque>  // 必须包含此头文件，否则 std::deque 会报错

// 保留你的原有注释和日志结构，请勿改动
std::string get_current_time_str() {
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    std::tm tm = *std::localtime(&now_time_t);
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    std::stringstream ss;
    ss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S") << "." << std::setw(3) << std::setfill('0') << milliseconds.count();
    return ss.str();
}

// ------------------ PID控制器 ------------------
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

    void set_gains(double p, double i, double d) {
        p_gain = p;
        i_gain = i;
        d_gain = d;
        // 如果每次修改需要清理积分，可在此处清理:
        // integral = 0;
        // prev_error = 0;
    }

    double getP() const { return p_gain; }
    double getI() const { return i_gain; }
    double getD() const { return d_gain; }

    void setIntegral(double val) { integral = val; }
    double getIntegral() const { return integral; }

private:
    double p_gain, i_gain, d_gain;
    double prev_error, integral;
};

// 用于读取单个PID
struct PIDParams {
    double p;
    double i;
    double d;
    PIDParams() : p(0.0), i(0.0), d(0.0) {}
};

// 用于读取各种阈值
struct CommonThresholds {
    double orient_err_sign_threshold;
    int    orient_sign_change_limit;
    double orient_avg_err_increase_threshold;
    int    orient_sign_change_small;

    double bridging_done_thresh;
    double look_ahead_time_offset;
    double k_for_speed_scale;
    double alpha_threshold;
    double distance_threshold;

    CommonThresholds()
        : orient_err_sign_threshold(0.05),
          orient_sign_change_limit(5),
          orient_avg_err_increase_threshold(0.1),
          orient_sign_change_small(2),
          bridging_done_thresh(0.2),
          look_ahead_time_offset(0.5),
          k_for_speed_scale(1.0),
          alpha_threshold(M_PI/2),
          distance_threshold(0.25)
    {}
};

// ------------------ 轨迹跟踪节点 ------------------
class TrajPidNode : public rclcpp::Node {
public:
    TrajPidNode()
        : Node("traj_pid_node"),

          // 先写死默认的 PID 值
          pid_position_control_(1, 0, 1),  
          pid_orientation_control_(1.5, 0, 0),
          pid_velocity_control_(0.75, 0, 1),  
          pid_angular_velocity_control_(1, 0, 0),

          // 默认的阈值
          orient_err_sign_threshold_(0.15),
          orient_sign_change_limit_(2),
          orient_avg_err_increase_threshold_(0.15),
          orient_sign_change_small_(0),
          bridging_done_thresh_(0.2),
          look_ahead_time_offset_(0.5),
          k_for_speed_scale_(1.0),
          alpha_threshold_(M_PI/2),
          distance_threshold_(0.25),

          target_linear_velocity_(0.0), target_angular_velocity_(0.0),
          current_linear_velocity_(0.0), current_angular_velocity_(0.0),
          current_position_x_(0.0), current_position_y_(0.0), current_yaw_(0.0),
          target_x_(0.0), target_y_(0.0), target_yaw_(0.0)
    {   
        // 1) 读取配置文件
        PIDParams pos_pid, ori_pid, vel_pid, angvel_pid;
        CommonThresholds thr;
        bool success = loadConfig("/home/jetson/ros2_ws/src/traj_pid/traj_pid/src/traj_pid_config.txt",
            pos_pid, ori_pid, vel_pid, angvel_pid, thr);
        if (!success) {
            RCLCPP_WARN(this->get_logger(), "无法从 traj_pid_config.txt 读取完整参数，将使用构造里的默认值");
        } else {
            // 覆盖PID
            pid_position_control_.set_gains(pos_pid.p, pos_pid.i, pos_pid.d);
            pid_orientation_control_.set_gains(ori_pid.p, ori_pid.i, ori_pid.d);
            pid_velocity_control_.set_gains(vel_pid.p, vel_pid.i, vel_pid.d);
            pid_angular_velocity_control_.set_gains(angvel_pid.p, angvel_pid.i, angvel_pid.d);

            // 覆盖各种阈值
            orient_err_sign_threshold_         = thr.orient_err_sign_threshold;
            orient_sign_change_limit_          = thr.orient_sign_change_limit;
            orient_avg_err_increase_threshold_ = thr.orient_avg_err_increase_threshold;
            orient_sign_change_small_          = thr.orient_sign_change_small;

            bridging_done_thresh_              = thr.bridging_done_thresh;
            look_ahead_time_offset_            = thr.look_ahead_time_offset;
            k_for_speed_scale_                 = thr.k_for_speed_scale;
            alpha_threshold_                   = thr.alpha_threshold;
            distance_threshold_                = thr.distance_threshold;
        }

        // 保留你原有的日志和注释
        std::string log_dir = "/home/jetson/ros2_ws/src/traj_pid/traj_pid_log";
        if (!std::filesystem::exists(log_dir)) {
            std::filesystem::create_directory(log_dir);
        }

        std::string log_filename = log_dir + "/" + get_current_time_str() + ".txt";
        log_file_.open(log_filename, std::ios::out);

        if (!log_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "无法打开日志文件: %s", log_filename.c_str());
            throw std::runtime_error("无法打开日志文件");
        }

        // 打印PID到日志
        log_file_ << "-------------------------" << std::endl;
        log_file_ << "PID Controller Parameters:" << std::endl;
        log_file_ << "Position PID: " << pid_position_control_.get_parameters_str() << std::endl;
        log_file_ << "Orientation PID: " << pid_orientation_control_.get_parameters_str() << std::endl;
        log_file_ << "Linear Velocity PID: " << pid_velocity_control_.get_parameters_str() << std::endl;
        log_file_ << "Angular Velocity PID: " << pid_angular_velocity_control_.get_parameters_str() << std::endl;
        log_file_ << "-------------------------" << std::endl;

        // 也记录一下阈值
        log_file_ << "Thresholds & offsets: \n";
        log_file_ << " orient_err_sign_threshold=" << orient_err_sign_threshold_ << "\n";
        log_file_ << " orient_sign_change_limit=" << orient_sign_change_limit_ << "\n";
        log_file_ << " orient_avg_err_increase_threshold=" << orient_avg_err_increase_threshold_ << "\n";
        log_file_ << " orient_sign_change_small=" << orient_sign_change_small_ << "\n";
        log_file_ << " bridging_done_thresh=" << bridging_done_thresh_ << "\n";
        log_file_ << " look_ahead_time_offset=" << look_ahead_time_offset_ << "\n";
        log_file_ << " k_for_speed_scale=" << k_for_speed_scale_ << "\n";
        log_file_ << " alpha_threshold=" << alpha_threshold_ << "\n";
        log_file_ << " distance_threshold=" << distance_threshold_ << "\n";
        log_file_ << "-------------------------" << std::endl;

        RCLCPP_INFO(this->get_logger(), "Logging to file: %s", log_filename.c_str());

        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/fastlio2/lio_odom", 10, std::bind(&TrajPidNode::odom_callback, this, std::placeholders::_1));

        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/livox/imu", 10, std::bind(&TrajPidNode::imu_callback, this, std::placeholders::_1));

        bspline_subscription_ = this->create_subscription<planner::msg::Bspline>(
            "/bspline", 10, std::bind(&TrajPidNode::bspline_callback, this, std::placeholders::_1));

        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "PID 轨迹跟踪节点已启动");

        pid_position_control_.print_parameters();
        pid_orientation_control_.print_parameters();
        pid_velocity_control_.print_parameters();
        pid_angular_velocity_control_.print_parameters();

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 10Hz
            std::bind(&TrajPidNode::control_loop, this));
    }

    ~TrajPidNode() {
        if (log_file_.is_open()) {
            log_file_.close();
        }
    }

private:
    // ==================== 读取配置文件函数 ====================
    bool loadConfig(const std::string &filename,
                    PIDParams &pos_pid,
                    PIDParams &ori_pid,
                    PIDParams &vel_pid,
                    PIDParams &angvel_pid,
                    CommonThresholds &thr)
    {
        std::ifstream ifs(filename.c_str());
        if(!ifs.is_open()) {
            std::cerr << "Failed to open config file: " << filename << std::endl;
            return false;
        }

        std::string line;
        enum Section {
            NONE,
            POSITION,
            ORIENTATION,
            VELOCITY,
            ANGULAR_VELOCITY,
            THRESHOLDS
        } currentSection = NONE;

        while(std::getline(ifs, line)) {
            // 去空格
            auto trim = [](std::string &s) {
                while(!s.empty() && (s.front()==' ' || s.front()=='\t')) s.erase(s.begin());
                while(!s.empty() && (s.back()==' ' || s.back()=='\t')) s.pop_back();
            };
            trim(line);
            if(line.empty()) continue;

            // 匹配段落
            if(line.find("[pid_position_control]") != std::string::npos) {
                currentSection = POSITION;
                continue;
            }
            else if(line.find("[pid_orientation_control]") != std::string::npos) {
                currentSection = ORIENTATION;
                continue;
            }
            else if(line.find("[pid_velocity_control]") != std::string::npos) {
                currentSection = VELOCITY;
                continue;
            }
            else if(line.find("[pid_angular_velocity_control]") != std::string::npos) {
                currentSection = ANGULAR_VELOCITY;
                continue;
            }
            else if(line.find("[thresholds]") != std::string::npos) {
                currentSection = THRESHOLDS;
                continue;
            }

            // key=value
            if(line.find('=') != std::string::npos) {
                auto pos = line.find('=');
                std::string key = line.substr(0, pos);
                std::string val = line.substr(pos+1);
                trim(key);
                trim(val);

                double dval = std::stod(val);

                switch(currentSection) {
                case POSITION:
                    if(key=="P") pos_pid.p = dval;
                    if(key=="I") pos_pid.i = dval;
                    if(key=="D") pos_pid.d = dval;
                    break;
                case ORIENTATION:
                    if(key=="P") ori_pid.p = dval;
                    if(key=="I") ori_pid.i = dval;
                    if(key=="D") ori_pid.d = dval;
                    break;
                case VELOCITY:
                    if(key=="P") vel_pid.p = dval;
                    if(key=="I") vel_pid.i = dval;
                    if(key=="D") vel_pid.d = dval;
                    break;
                case ANGULAR_VELOCITY:
                    if(key=="P") angvel_pid.p = dval;
                    if(key=="I") angvel_pid.i = dval;
                    if(key=="D") angvel_pid.d = dval;
                    break;
                case THRESHOLDS:
                    if(key=="orient_err_sign_threshold") thr.orient_err_sign_threshold = dval;
                    if(key=="orient_sign_change_limit")   thr.orient_sign_change_limit  = static_cast<int>(dval);
                    if(key=="orient_avg_err_increase_threshold") thr.orient_avg_err_increase_threshold = dval;
                    if(key=="orient_sign_change_small")   thr.orient_sign_change_small  = static_cast<int>(dval);

                    if(key=="bridging_done_thresh")       thr.bridging_done_thresh = dval;
                    if(key=="look_ahead_time_offset")     thr.look_ahead_time_offset = dval;
                    if(key=="k_for_speed_scale")          thr.k_for_speed_scale = dval;
                    if(key=="alpha_threshold")            thr.alpha_threshold = dval;
                    if(key=="distance_threshold")         thr.distance_threshold = dval;
                    break;
                default:
                    break;
                }
            }
        }

        ifs.close();
        return true;
    }
    // =========================================================

private:
    // 保留你的结构不变
    struct ControlPoint {
        double x;
        double y;
        double yaw;
        double time;
    };

    friend std::ostream& operator<<(std::ostream& os, const ControlPoint& cp) {
        os << "(x: " << cp.x << ", y: " << cp.y << ", yaw: " << cp.yaw << ", time: " << cp.time << ")";
        return os;
    };

    // ------------------ PID实例 ------------------
    PIDController pid_position_control_;
    PIDController pid_orientation_control_;
    PIDController pid_velocity_control_;
    PIDController pid_angular_velocity_control_;

    // ------------------ 日志输出 ------------------
    std::ofstream log_file_;

    // ------------------ ROS 相关 ------------------
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr traj_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<planner::msg::Bspline>::SharedPtr bspline_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // ------------------ 轨迹信息 ------------------
    std::vector<planner::NonUniformBspline> traj_;
    std::chrono::time_point<std::chrono::system_clock> start_time_;
    double traj_duration_ = 0.0;
    bool receive_traj_ = false;

    bool bspline_received_ = false;
    bool odom_received_ = false;
    bool imu_received_ = false;
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

    double previous_linear_cmd_ = 0.0;
    double previous_angular_cmd_ = 0.0;
    double real_start_time_sec = this->now().seconds();

    // === 新增: 用于判断不连续轨迹 & bridging ===
    bool has_previous_traj_ = false;
    double last_traj_end_x_ = 0.0;
    double last_traj_end_y_ = 0.0;
    double discontinuous_threshold_ = 1.0; // 大于1米认为不连续

    bool bridging_ = false;
    double bridging_x_ = 0.0;
    double bridging_y_ = 0.0;
    std::vector<planner::NonUniformBspline> next_traj_;
    double next_traj_duration_ = 0.0;

    // ============== 历史误差队列 ==============
    static const size_t ORIENT_ERR_QUEUE_SIZE_ = 100; 
    std::deque<double> orient_err_queue_;

    // ============== 以下是原本写死的阈值，改成成员变量，从文件读 ==============
    double orient_err_sign_threshold_;         
    int    orient_sign_change_limit_;          
    double orient_avg_err_increase_threshold_;
    int    orient_sign_change_small_;

    double bridging_done_thresh_;             
    double look_ahead_time_offset_;           
    double k_for_speed_scale_;                
    double alpha_threshold_;                  
    double distance_threshold_;               

private:
    // ------------------ 回调：IMU ------------------
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        if (!msg) {
            imu_received_ = false;
            return;
        }
        imu_received_ = true;
        current_angular_velocity_ = msg->angular_velocity.z;

        // if (log_file_.is_open()) {
        //     const auto &ang = msg->angular_velocity;
        //     log_file_ << msg->header.stamp.sec << "." << std::setw(9) << std::setfill('0')
        //               << msg->header.stamp.nanosec 
        //               << " Received IMU Angular Velocity - ["
        //               << ang.x << ", " << ang.y << ", " << ang.z << "]" << std::endl;
        // }
    }

    // ------------------ 回调：里程计 ------------------
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (!msg) {
            odom_received_ = false;
            return;
        }
        odom_received_ = true;

        double sensor_x = msg->pose.pose.position.x;
        double sensor_y = msg->pose.pose.position.y;
        double sensor_linear_vel = msg->twist.twist.linear.x;

        tf2::Quaternion quat;
        tf2::fromMsg(msg->pose.pose.orientation, quat);
        double roll, pitch;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, current_yaw_);

        double sensor_offset_x = -0.12; 
        double sensor_offset_y = -0.07; 

        current_position_x_ = sensor_x - (std::cos(current_yaw_) * sensor_offset_x 
                                   - std::sin(current_yaw_) * sensor_offset_y);
        current_position_y_ = sensor_y - (std::sin(current_yaw_) * sensor_offset_x 
                                   + std::cos(current_yaw_) * sensor_offset_y);

        current_linear_velocity_ = sensor_linear_vel + current_angular_velocity_ * sensor_offset_y;

        log_file_ << "Received Odometry - Position: x=" << current_position_x_
                  << ", y=" << current_position_y_
                  << ", Linear Velocity: " << current_linear_velocity_
                  << std::endl;

        log_file_ << "Current yaw: " << current_yaw_ 
                  << " (roll: " << roll << ", pitch: " << pitch << ")" 
                  << std::endl;
    }

    // ------------------ 回调：Bspline ------------------
    void bspline_callback(const planner::msg::Bspline::SharedPtr msg) {
        std::string current_time = get_current_time_str();

        if (!msg) {
            log_file_ << "[" << current_time << "] B-spline message is null." << std::endl;
            return;
        }
        if (msg->pos_pts.empty() || msg->yaw_pts.empty() || msg->knots.empty()) {
            log_file_ << "[" << current_time << "] B-spline message missing pos_pts, yaw_pts 或 knots." << std::endl;
            return;
        }

        // 构造位置点矩阵
        Eigen::MatrixXd pos_pts(msg->pos_pts.size(), 3);
        for (size_t i = 0; i < msg->pos_pts.size(); i++) {
            pos_pts(i, 0) = msg->pos_pts[i].x;
            pos_pts(i, 1) = msg->pos_pts[i].y;
            pos_pts(i, 2) = msg->pos_pts[i].z;
        }
        Eigen::VectorXd knots(msg->knots.size());
        for (size_t i = 0; i < msg->knots.size(); i++) {
            knots(i) = msg->knots[i];
        }

        planner::NonUniformBspline pos_traj(pos_pts, msg->order, 0.1);
        pos_traj.setKnot(knots);

        Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
        for (size_t i = 0; i < msg->yaw_pts.size(); i++) {
            yaw_pts(i, 0) = msg->yaw_pts[i];
        }
        planner::NonUniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

        double new_traj_duration = pos_traj.getTimeSum();

        // === 检查是否连续 ===
        bool is_discontinuous = false;
        if (has_previous_traj_) {
            double new_start_x = msg->pos_pts.front().x;
            double new_start_y = msg->pos_pts.front().y;
            double dist = std::hypot(new_start_x - last_traj_end_x_,
                                     new_start_y - last_traj_end_y_);
            if (dist > discontinuous_threshold_) {
                is_discontinuous = true;
            }
        }

        // 记录末端信息
        has_previous_traj_ = true;
        size_t last_i = msg->pos_pts.size() - 1;
        last_traj_end_x_ = msg->pos_pts[last_i].x;
        last_traj_end_y_ = msg->pos_pts[last_i].y;

        // --- 如果不连续，则 bridging ---
        if (is_discontinuous) {
            bridging_ = true;
            bridging_x_ = msg->pos_pts.front().x; 
            bridging_y_ = msg->pos_pts.front().y;

            // 缓存新轨迹
            next_traj_.clear();
            next_traj_.push_back(pos_traj);
            next_traj_.push_back(pos_traj.getDerivative());
            next_traj_.push_back(next_traj_[1].getDerivative());
            next_traj_.push_back(yaw_traj);
            next_traj_.push_back(yaw_traj.getDerivative());

            next_traj_duration_ = new_traj_duration;

            // 不立即 receive_traj_=true，不清空队列，也不停止
            log_file_ << "[" << current_time << "] B-spline不连续, bridging to ("
                      << bridging_x_ << ", " << bridging_y_ << "), dist="
                      << std::hypot(bridging_x_ - current_position_x_, 
                                    bridging_y_ - current_position_y_)
                      << std::endl;

        } else {
            // 正常连续 => 直接赋值
            bridging_ = false; // 万一之前在bridging，这里取消
            // 正常加载
            start_time_ = std::chrono::system_clock::now();
            traj_.clear();
            traj_.push_back(pos_traj);
            traj_.push_back(pos_traj.getDerivative());
            traj_.push_back(traj_[1].getDerivative());
            traj_.push_back(yaw_traj);
            traj_.push_back(yaw_traj.getDerivative());

            traj_duration_ = new_traj_duration;
            receive_traj_ = true;

            log_file_ << "[" << current_time << "] Received Bspline message - pos_pts size: " 
                      << msg->pos_pts.size() << ", yaw_pts size: " << msg->yaw_pts.size() 
                      << ", order=" << msg->order << std::endl;

            log_file_ << "----- Control Points -----" << std::endl;
            double dt = 0.1;  
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
    }

    // ------------------ 控制循环 ------------------
    void control_loop() {
        // 如果 bridging_ = true，则先跑 bridging 逻辑
        if (bridging_) {
            double dx = bridging_x_ - current_position_x_;
            double dy = bridging_y_ - current_position_y_;
            double dist = std::sqrt(dx*dx + dy*dy);

            // 用位置PID
            double bridging_angle = std::atan2(dy, dx);
            double alpha = bridging_angle - current_yaw_;
            while (alpha > M_PI) alpha -= 2.0*M_PI;
            while (alpha < -M_PI) alpha += 2.0*M_PI;

            double linear_cmd = pid_position_control_.compute(dist, 0.0);
            double orient_cmd = pid_orientation_control_.compute(alpha, 0.0);

            // 大于90度先转向
            if (std::fabs(alpha) > alpha_threshold_) {
                linear_cmd = 0.0;
            }

            // 简单限幅
            const double max_linear_speed = 0.6;
            const double max_angular_speed = 1.0;
            if (linear_cmd > max_linear_speed) linear_cmd = max_linear_speed;
            if (linear_cmd < -max_linear_speed) linear_cmd = -max_linear_speed;
            if (orient_cmd > max_angular_speed) orient_cmd = max_angular_speed;
            if (orient_cmd < -max_angular_speed) orient_cmd = -max_angular_speed;

            geometry_msgs::msg::Twist cmd_msg;
            cmd_msg.linear.x  = linear_cmd;
            cmd_msg.angular.z = orient_cmd;
            cmd_publisher_->publish(cmd_msg);

            // 记录日志
            log_file_ << "-------------------------" << std::endl;
            log_file_ << "[" << get_current_time_str() << "] bridging_ to("
                      << bridging_x_ << "," << bridging_y_ << "), dist=" << dist
                      << ", bridging_angle=" << bridging_angle
                      << std::endl;
            log_file_ << "-- PID Output: Linear Velocity: " << linear_cmd
                      << ", Angular Velocity: " << orient_cmd << std::endl;
            log_file_ << "Linear Velocity Target: " << linear_cmd
                      << ", Angular Velocity Target: " << orient_cmd << std::endl;
            log_file_ << "Target Position - x: " << bridging_x_
                      << ", y: " << bridging_y_ << ", yaw: " << bridging_angle << std::endl;
            log_file_ << "Current Position - x: " << current_position_x_
                      << ", y: " << current_position_y_
                      << ", Current Yaw: " << current_yaw_ << std::endl;
            log_file_ << "--Distance to Target (abs): " << dist << std::endl;
            double e_forward = dist * std::cos(alpha); 
            log_file_ << "--Position ERROR (signed): " << e_forward << std::endl;
            log_file_ << "--Orientation ERROR: " << alpha << std::endl;
            log_file_ << "Current Linear Velocity: " << current_linear_velocity_ << std::endl;
            log_file_ << "Current Angular Velocity: " << current_angular_velocity_ << std::endl;
            log_file_ << "-------------------------" << std::endl;

            // 如果距离小于某阈值 => bridging完成
            if (dist < bridging_done_thresh_) {
                bridging_ = false;
                // 切换到 next_traj_
                traj_ = next_traj_;
                traj_duration_ = next_traj_duration_;

                // 重新计时
                start_time_ = std::chrono::system_clock::now();
                receive_traj_ = true;

                log_file_ << "[" << get_current_time_str() << "] bridging done, now use new B-spline, duration="
                          << traj_duration_ << std::endl;
            }
            return; 
        }

        // 如果没有轨迹，就打印日志然后退出
        if (!receive_traj_) {
            std::string current_time = get_current_time_str();
            log_file_ << "-------------------------" << std::endl;
            log_file_ << "[" << current_time << "] No B-spline trajectory received." << std::endl;
            log_file_ << "-------------------------" << std::endl;
            return;
        }

        // 计算轨迹时间
        auto now_tp = std::chrono::system_clock::now();
        double t_diff = std::chrono::duration_cast<std::chrono::duration<double>>(
                            now_tp - start_time_).count();

        // 如果超过持续时间，则做尾部对齐
        if (t_diff > traj_duration_) {
            double final_yaw = traj_[3].evaluateDeBoor(traj_duration_)(0);
            double yaw_error = final_yaw - current_yaw_;
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
                          << final_yaw << ", current_yaw = " << current_yaw_ 
                          << ", yaw_error = " << yaw_error 
                          << ", yaw_align_cmd = " << yaw_align_cmd << std::endl;
            } else {
                geometry_msgs::msg::Twist cmd_msg;
                cmd_msg.linear.x  = 0.0;
                cmd_msg.angular.z = 0.0;
                cmd_publisher_->publish(cmd_msg);
                log_file_ << "[" << get_current_time_str() << "] Yaw aligned (error " 
                          << yaw_error << " < threshold), stopping robot." << std::endl;
                receive_traj_ = false;
            }
            return;
        }

        // 前视 look_ahead_time_offset_ 秒
        double t_target = t_diff + look_ahead_time_offset_;
        if (t_target > traj_duration_) {
            t_target = traj_duration_;
        }

        // 从轨迹评估目标
        Eigen::Vector3d pos = traj_[0].evaluateDeBoor(t_target);
        Eigen::Vector3d vel = traj_[1].evaluateDeBoor(t_diff);
        double yaw = traj_[3].evaluateDeBoor(t_target)(0);
        double yaw_dot = traj_[4].evaluateDeBoor(t_target)(0);

        target_x_ = pos(0);
        target_y_ = pos(1);
        target_linear_velocity_  = vel(0);
        target_angular_velocity_ = yaw_dot;

        // 位置误差
        double dx = target_x_ - current_position_x_;
        double dy = target_y_ - current_position_y_;
        double distance = std::sqrt(dx*dx + dy*dy);

        // 朝向误差 alpha
        double angle_to_target = std::atan2(dy, dx);
        target_yaw_ = angle_to_target;
        double alpha = angle_to_target - current_yaw_;
        while (alpha > M_PI)  alpha -= 2.0 * M_PI;
        while (alpha < -M_PI) alpha += 2.0 * M_PI;

        double abs_alpha = std::fabs(alpha);

        // 如果距离小于 distance_threshold_，直接用轨迹自带 yaw
        double ori_x = 0.0;  
        double ori_y = 0.0;  
        double ori_dx = target_x_ - ori_x;
        double ori_dy = target_y_ - ori_y;
        double traj_distance = std::sqrt(ori_dx*ori_dx + ori_dy*ori_dy);
        if (traj_distance < distance_threshold_) {
            target_yaw_ = yaw;
            alpha = 0.0;
        }

        // e_forward 仅日志
        double e_forward = distance * std::cos(alpha);

        // 计算 PID
        double pos_pid = pid_position_control_.compute(distance, 0.0);
        double orient_pid = pid_orientation_control_.compute(alpha, 0.0);

        double error_linear_vel = current_linear_velocity_ - previous_linear_cmd_;
        double error_angular_vel = current_angular_velocity_ - previous_angular_cmd_;

        double linear_vel_pid = pid_velocity_control_.compute(error_linear_vel, 0);
        double angular_vel_pid = pid_angular_velocity_control_.compute(error_angular_vel, 0);

        // 我们暂时直接用 pos_pid 当线速度
        double linear_cmd = pos_pid;
        double angular_cmd = orient_pid;

        // 若航向误差太大(> alpha_threshold_) => 原地转向
        if (std::fabs(alpha) > alpha_threshold_) {
            linear_cmd = 0.0;
        }

        // 对线速度做一个基于航向误差的衰减
        double speed_scale = std::exp(-k_for_speed_scale_ * abs_alpha * abs_alpha);
        linear_cmd *= speed_scale;

        // 限幅
        const double max_linear_speed = 0.6;
        const double max_angular_speed = 1.0;
        if (linear_cmd >  max_linear_speed)  linear_cmd =  max_linear_speed;
        if (linear_cmd < -max_linear_speed)  linear_cmd = -max_linear_speed;
        if (angular_cmd >  max_angular_speed)  angular_cmd =  max_angular_speed;
        if (angular_cmd < -max_angular_speed)  angular_cmd = -max_angular_speed;

        previous_linear_cmd_ = linear_cmd;
        previous_angular_cmd_ = angular_cmd;

        // ---- 日志输出 (保持原样) ----
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

        // 发布控制指令
        geometry_msgs::msg::Twist cmd_msg;
        cmd_msg.linear.x  = linear_cmd;
        cmd_msg.angular.z = angular_cmd;
        cmd_publisher_->publish(cmd_msg);

        // =================== 更复杂的航向PID自适应逻辑 ===================
        orient_err_queue_.push_back(alpha);
        if (orient_err_queue_.size() > ORIENT_ERR_QUEUE_SIZE_) {
            orient_err_queue_.pop_front(); 
        }

        auto count_sign_changes = [this]() {
            int sign_changes = 0;
            int last_sign = 0;
            for (auto &val : this->orient_err_queue_) {
                if (std::fabs(val) > orient_err_sign_threshold_) {
                    int s = (val >= 0.0) ? +1 : -1;
                    if (last_sign != 0 && s != last_sign) {
                        sign_changes++;
                    }
                    last_sign = s;
                }
            }
            return sign_changes;
        };

        int sc_count = count_sign_changes();

        double sum_err = 0.0;
        for (auto &val : orient_err_queue_) {
            sum_err += std::fabs(val);
        }
        double avg_err = 0.0;
        if (!orient_err_queue_.empty()) {
            avg_err = sum_err / static_cast<double>(orient_err_queue_.size());
        }

        double currP = pid_orientation_control_.getP();
        double currI = pid_orientation_control_.getI();
        double currD = pid_orientation_control_.getD();

        // 1) 震荡 => 减P增D
        if (sc_count >= orient_sign_change_limit_) {
            double newP = currP - 0.05;
            if (newP < 0.0) newP = 0.0;
            double newD = currD + 0.05;
            if (newD > 10.0) newD = 10.0;

            pid_orientation_control_.set_gains(newP, currI, newD);

            log_file_ << "[" << get_current_time_str() << "] Orientation sign changes = "
                      << sc_count << ", reduce P from " << currP << " to " << newP
                      << ", increase D from " << currD << " to " << newD
                      << ", avg_err = " << avg_err << std::endl;
        }
        // 2) 平均误差大 => 增P
        else if (avg_err > orient_avg_err_increase_threshold_ && sc_count <= orient_sign_change_small_) {
            double newP = currP + 0.03;
            if (newP > 10.0) newP = 10.0;

            pid_orientation_control_.set_gains(newP, currI, currD);

            log_file_ << "[" << get_current_time_str() << "] High avg orient error = "
                      << avg_err << ", sign changes = " << sc_count
                      << ", increase P from " << currP << " to " << newP
                      << " (D stays " << currD << "), I=" << currI << std::endl;
        }
        // 3) 长时间同向 => 加 I
        else if (avg_err > 0.2 && sc_count == 0) {
            double newI = currI + 0.001; 
            if (newI > 0.5) newI = 0.5;

            pid_orientation_control_.set_gains(currP, newI, currD);

            log_file_ << "[" << get_current_time_str() << "] Sustained same-direction error, increase I from "
                      << currI << " to " << newI
                      << ", P=" << currP << ", D=" << currD << std::endl;
        }
        // 4) 频繁反转且积分很大 => 减 I 并清理
        else if (sc_count > 3 && std::fabs(pid_orientation_control_.getIntegral()) > 30.0) {
            double newI = currI - 0.002;
            if (newI < 0.0) newI = 0.0;
            pid_orientation_control_.setIntegral(0.0);
            pid_orientation_control_.set_gains(currP, newI, currD);

            log_file_ << "[" << get_current_time_str() << "] Sign changes + large integral => reduce I from "
                      << currI << " to " << newI << " and reset integral to 0."
                      << ", P=" << currP << ", D=" << currD << std::endl;
        }
        // 其他情况就先不动

        // 打印当前 orientation PID
        {
            double pOri = pid_orientation_control_.getP();
            double iOri = pid_orientation_control_.getI();
            double dOri = pid_orientation_control_.getD();
            log_file_ << "[" << get_current_time_str() << "] Current orientation PID: "
                      << "P = " << pOri << ", I = " << iOri << ", D = " << dOri << std::endl;
        }
        log_file_ << "-------------------------" << std::endl;
    }
};

// ------------------ main ------------------
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajPidNode>());
    rclcpp::shutdown();
    return 0;
}
