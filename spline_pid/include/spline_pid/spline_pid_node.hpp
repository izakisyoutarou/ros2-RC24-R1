#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "path_msg/msg/path.hpp"
#include "socketcan_interface_msg/msg/socketcan_if.hpp"
#include "trapezoidal_velocity_planner.hpp"

#include "spline_pid/visibility_control.h"

using VelPlanner = velocity_planner::trapezoidal_velocity_planner::TrapezoidalVelocityPlanner;
using VelPlannerLimit = velocity_planner::trapezoidal_velocity_planner::Limit_t;

namespace spline_pid{

class SplinePid : public rclcpp::Node {
public:
    SPLINE_PID_PUBLIC
    explicit SplinePid(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    SPLINE_PID_PUBLIC
    explicit SplinePid(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    rclcpp::Subscription<path_msg::msg::Path>::SharedPtr _subscription_path;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr _subscription_self_pose;

    void _subscriber_callback_path(const path_msg::msg::Path::SharedPtr msg);
    void _subscriber_callback_self_pose(const geometry_msgs::msg::Vector3::SharedPtr msg);

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_velocity;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_is_tracking;
    rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr publisher_linear;
    rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr publisher_angular;

    rclcpp::TimerBase::SharedPtr _pub_timer;
    void _publisher_callback();

    rclcpp::QoS _qos = rclcpp::QoS(40).keep_all();

    // path_msg::msg::Path path;  //経路
    std::shared_ptr<path_msg::msg::Path> path;

    //計画機
    VelPlanner velPlanner_linear;
    VelPlannerLimit limit_linear;
    VelPlanner velPlanner_angular;
    VelPlannerLimit limit_angular;

    //ゲイン
    double curvature_attenuation_rate = 0.0;
    double linear_planner_vel_limit_gain = 0.0;
    double linear_planner_gain = 0.0;
    double linear_pos_gain = 0.0;
    double linear_pos_integral_gain = 0.0;

    double angular_planner_gain = 0.0;
    double angular_pos_gain = 0.0;
    double angular_pos_integral_gain = 0.0;

    //許容値
    double linear_pos_tolerance = 0.0;
    double angular_pos_tolerance = 0.0;

    //フィールド
    int current_count = 0;
    int max_trajectories = 0;
    bool is_target_arrived = false;
    bool is_arrived = false;
    geometry_msgs::msg::Vector3 self_pose; //自己位置
    geometry_msgs::msg::Vector3 last_target_position;   //最後に参照した目標位置
    double x_diff = 1.0;
    double y_diff = 1.0;    //0での除算を防ぐため1を入れておく
    //積分
    double sampling_time = 0.0;
    double error_x_integral = 0.0;
    double error_y_integral = 0.0;
    double error_a_integral = 0.0;
};

}  // namespace spline_pid
