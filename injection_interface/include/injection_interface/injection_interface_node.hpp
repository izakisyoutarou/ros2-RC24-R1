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
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _sub_is_backside;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _sub_is_move_tracking;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr _sub_self_pose;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr _sub_move_target_pose;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub_move_node;

    void _callback_is_backside(const std_msgs::msg::Bool::SharedPtr msg);
    void _callback_is_move_tracking(const std_msgs::msg::Bool::SharedPtr msg);
    void _callback_self_pose(const geometry_msgs::msg::Vector3::SharedPtr msg);
    void _callback_move_target_pose(const geometry_msgs::msg::Vector3::SharedPtr msg);
    void _callback_move_node(const std_msgs::msg::String::SharedPtr msg);
    
    rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb;
    rclcpp::Publisher<injection_interface_msg::msg::InjectionCommand>::SharedPtr _pub_injection;
    rclcpp::Publisher<path_msg::msg::Turning>::SharedPtr _pub_spin_position;

    void set_calculate_vel(bool is_backside);

    void command_injection_turn();
    void command_injection_pitch(double linear_pitch);

    rclcpp::QoS _qos = rclcpp::QoS(10);

    const int16_t can_backspin_vel_id;
    const int16_t can_inject_pitch_id;

    TwoVector target_pos;

    //定数
    const std::vector<double> tf_injection2robot;
    std::vector<double> strage_backside;
    std::vector<double> strage_front;
    std::vector<double> linear_pitch;
    std::vector<double> linear_tf;
    std::vector<double> injection_gain;

    //フィールド
    geometry_msgs::msg::Vector3 self_pose;
    geometry_msgs::msg::Vector3 move_target_pose;
    bool is_move_tracking = false;
    bool is_correction_required = false;
    bool last_target;
    const std::string court_color_;
    double pitch = 0.0;
    int injection_num  = 0;
};

}