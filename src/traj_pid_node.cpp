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

class PIDController {
public:
    PIDController(double p, double i, double d)
    : p_gain(p), i_gain(i), d_gain(d), prev_error(0), integral(0) {}

    double compute(double setpoint, double actual) {
        double error = setpoint - actual;
        integral += error;
        double derivative = error - prev_error;
        prev_error = error;

        // PID公式
        double output = p_gain * error + i_gain * integral + d_gain * derivative;

        return output;
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
        // 订阅轨迹话题
        traj_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/traj", 10, std::bind(&TrajPidNode::traj_callback, this, std::placeholders::_1));

        // 订阅里程计话题
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10, std::bind(&TrajPidNode::odom_callback, this, std::placeholders::_1));

        // 订阅IMU话题
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/livox/imu", 10, std::bind(&TrajPidNode::imu_callback, this, std::placeholders::_1));

        // 订阅Bspline轨迹话题
        bspline_subscription_ = this->create_subscription<planner::msg::Bspline>(
            "/bspline", 10, std::bind(&TrajPidNode::bspline_callback, this, std::placeholders::_1));

        // 控制命令发布者
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "PID 轨迹跟踪节点已启动");
    }

private:
    void traj_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        target_linear_velocity_ = msg->linear.x;
        target_angular_velocity_ = msg->angular.z;

        RCLCPP_INFO(this->get_logger(), "Received trajectory - Linear velocity: %f, Angular velocity: %f",
                    target_linear_velocity_, target_angular_velocity_);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_linear_velocity_ = msg->twist.twist.linear.x;
        current_angular_velocity_ = msg->twist.twist.angular.z;

        // 打印当前位置和姿态信息
        current_position_x_ = msg->pose.pose.position.x;
        current_position_y_ = msg->pose.pose.position.y;

        RCLCPP_INFO(this->get_logger(), "Received Odometry - Position: x=%f, y=%f, Linear Velocity: %f, Angular Velocity: %f",
                    current_position_x_, current_position_y_, current_linear_velocity_, current_angular_velocity_);

        // 获取当前朝向（四元数转换为欧拉角）
        tf2::Quaternion quat;
        tf2::fromMsg(msg->pose.pose.orientation, quat);

        // 获取 yaw 值（转换为欧拉角）
        double roll, pitch;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, current_yaw_);

        RCLCPP_INFO(this->get_logger(), "Current yaw: %f (roll: %f, pitch: %f)", current_yaw_, roll, pitch);
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // 获取IMU的角速度和线性加速度
        double angular_velocity = msg->angular_velocity.z;  // 角速度
        double linear_acceleration = msg->linear_acceleration.x;  // 线性加速度

        RCLCPP_INFO(this->get_logger(), "IMU - Angular Velocity: %f, Linear Acceleration: %f", 
                    angular_velocity, linear_acceleration);
    }

    void bspline_callback(const planner::msg::Bspline::SharedPtr msg) {
        // 提取参考轨迹数据
        if (!msg->pos_pts.empty()) {
            target_x_ = msg->pos_pts[0].x;
            target_y_ = msg->pos_pts[0].y;

            // 提取目标朝向
            if (!msg->yaw_pts.empty()) {
                target_yaw_ = msg->yaw_pts[0];  // 假设目标朝向是第一个控制点的yaw值
            }

            RCLCPP_INFO(this->get_logger(), "Received Bspline - Target position: x=%f, y=%f, target yaw=%f",
                        target_x_, target_y_, target_yaw_);
        }
    }

    void control_loop() {
        // 计算与参考轨迹的误差
        double position_error = std::sqrt(std::pow(target_x_ - current_position_x_, 2) + std::pow(target_y_ - current_position_y_, 2));

        // 使用PID控制器调整线速度
        double pid_linear_output = pid_control_.compute(position_error, 0.0);  // 位置误差

        // 计算朝向误差
        double orientation_error = target_yaw_ - current_yaw_;

        // 确保朝向误差在-π到π之间
        if (orientation_error > M_PI) {
            orientation_error -= 2 * M_PI;
        } else if (orientation_error < -M_PI) {
            orientation_error += 2 * M_PI;
        }

        // 使用PID控制器调整角速度
        double pid_angular_output = pid_orientation_control_.compute(orientation_error, 0.0);  // 朝向误差

        // 打印PID控制输出
        RCLCPP_INFO(this->get_logger(), "Control Loop - PID Output: Linear Velocity: %f, Angular Velocity: %f",
                    pid_linear_output, pid_angular_output);

        // 生成控制命令
        geometry_msgs::msg::Twist cmd_msg;
        cmd_msg.linear.x = pid_linear_output;
        cmd_msg.angular.z = pid_angular_output;

        // 发布控制命令
        cmd_publisher_->publish(cmd_msg);
    }

    // PID控制器实例
    PIDController pid_control_;
    PIDController pid_orientation_control_; // 朝向的PID控制器

    // 订阅者
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr traj_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<planner::msg::Bspline>::SharedPtr bspline_subscription_;

    // 发布者
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;

    // 当前目标速度
    double target_linear_velocity_ = 0.0;
    double target_angular_velocity_ = 0.0;

    // 当前速度
    double current_linear_velocity_ = 0.0;
    double current_angular_velocity_ = 0.0;

    // 参考轨迹的目标坐标
    double target_x_ = 0.0;
    double target_y_ = 0.0;

    // 当前的实际坐标（假设从里程计中获取）
    double current_position_x_ = 0.0;
    double current_position_y_ = 0.0;

    // 当前朝向（从里程计中获取）
    double current_yaw_ = 0.0;

    // 目标朝向
    double target_yaw_ = 0.0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajPidNode>());
    rclcpp::shutdown();
    return 0;
}
