#pragma once

#include <rclcpp/rclcpp.hpp>
#include "injection_interface/visibility_control.h"

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include "injection_interface_msg/msg/injection_command.hpp"
#include "path_msg/msg/turning.hpp"
#include "socketcan_interface_msg/msg/socketcan_if.hpp"
#include "controller_interface_msg/msg/base_control.hpp"

#include "utilities/two_vector.hpp"
#include "utilities/utils.hpp"

#include <string>
#include <cmath>

namespace injection_interface{

class InjectionInterface : public rclcpp::Node {
    using TwoVector = utils::TwoVectord;
    
public:
    INJECTION_INTERFACE_PUBLIC
    explicit InjectionInterface(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    INJECTION_INTERFACE_PUBLIC
    explicit InjectionInterface(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _subscriber_injection_calculate;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _subscriber_is_move_tracking;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr _subscriber_self_pose;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr _subscriber_move_target_pose;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _subscriber_target_node;
    rclcpp::Subscription<controller_interface_msg::msg::BaseControl>::SharedPtr _subscription_base_control;

    void _callback_injection_calculate(const std_msgs::msg::Empty::SharedPtr msg);
    void _callback_is_move_tracking(const std_msgs::msg::Bool::SharedPtr msg);
    void _callback_self_pose(const geometry_msgs::msg::Vector3::SharedPtr msg);
    void _callback_move_target_pose(const geometry_msgs::msg::Vector3::SharedPtr msg);
    void _callback_target_node(const std_msgs::msg::String::SharedPtr msg);
    void _subscriber_callback_base_control(const controller_interface_msg::msg::BaseControl::SharedPtr msg);
    
    rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _publisher_canusb;
    rclcpp::Publisher<injection_interface_msg::msg::InjectionCommand>::SharedPtr _publisher_injection;
    rclcpp::Publisher<path_msg::msg::Turning>::SharedPtr _publisher_spin_position;

    void set_calculate_vel();

    void command_calculation_vel();
    void command_injection_turn();
    void command_injection_pitch(double linear_pitch);

    rclcpp::QoS _qos = rclcpp::QoS(10);

    const int16_t can_inject_pitch_id;

    TwoVector target_pos;

    //定数
    const std::vector<double> tf_injection2robot;
    std::vector<double> strage_backside_0;
    std::vector<double> strage_backside_1;
    std::vector<double> strage_front;
    const std::vector<double> linear_pitch;
    const std::vector<double> linear_tf;
    std::vector<double> injection_gain;

    //フィールド
    geometry_msgs::msg::Vector3 self_pose;
    geometry_msgs::msg::Vector3 move_target_pose;
    bool is_move_tracking = false;
    bool is_correction_required = false;
    bool last_target = false;
    double pitch = 0.0;
    int injection_num  = 0;
    std::string node = "";
    TwoVector diff;
    double target_height = 0.0;

    struct Node{
        std::string name;
        double x;
        double y;
        double z;
    };

    std::vector<Node> node_list;

    std::string injection_point = "";

    std::vector<double>  injectionpoint_tolerance;
    std::string old_injectionpoint = "";

    bool is_move_autonomous = false;
};

}