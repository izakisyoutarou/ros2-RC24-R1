#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>
#include "socketcan_interface_msg/msg/socketcan_if.hpp"
#include "controller_interface_msg/msg/base_control.hpp"
#include "controller_interface_msg/msg/convergence.hpp"
#include "controller_interface_msg/msg/injection.hpp"
#include "socket_udp.hpp"

#include "sequencer/visibility_control.h"

namespace sequencer{

class Sequencer : public rclcpp::Node {
public:
    SEQUENCER_PUBLIC
    explicit Sequencer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    SEQUENCER_PUBLIC
    explicit Sequencer(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    rclcpp::Subscription<controller_interface_msg::msg::BaseControl>::SharedPtr _subscription_base_control;
    rclcpp::Subscription<controller_interface_msg::msg::Convergence>::SharedPtr _subscription_convergence;
    rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _subscription_movable;
    rclcpp::Subscription<controller_interface_msg::msg::Injection>::SharedPtr _subscription_injected;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _subscription_pole;

    void _subscriber_callback_base_control(const controller_interface_msg::msg::BaseControl::SharedPtr msg);
    void _subscriber_callback_convergence(const controller_interface_msg::msg::Convergence::SharedPtr msg);
    void _subscriber_callback_movable(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);
    void _subscriber_callback_injected(const controller_interface_msg::msg::Injection::SharedPtr msg);
    void _subscriber_callback_pole(const std_msgs::msg::String::SharedPtr msg);
    void _recv_callback();
    void _recv_robot_state(const unsigned char data[2]);
    void cancel_inject(const bool mech0, const bool mech1);

    rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr publisher_can;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_pole_m0;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_pole_m1;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_move_node;
    rclcpp::Publisher<controller_interface_msg::msg::Injection>::SharedPtr publisher_rings;

    rclcpp::TimerBase::SharedPtr _socket_timer;

    RecvUDP socket_robot_state;

    rclcpp::QoS _qos = rclcpp::QoS(10);

    const int16_t can_movable_id;
    const int16_t can_digital_button_id;
    const int16_t can_inject_id;
    const int16_t can_cancel_inject_id;

    const int max_poles = 5;
    std::array<int,2> current_poles = {max_poles,max_poles};

    controller_interface_msg::msg::Convergence judge_convergence;

    std::string current_pickup_state;
    std::string current_inject_state;
    std::string initial_state;


};

}  // namespace sequencer
