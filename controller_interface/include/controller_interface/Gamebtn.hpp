#pragma once
#include <rclcpp/rclcpp.hpp>
#include <float.h>
#include <string>
//使うmsg
#include "socketcan_interface_msg/msg/socketcan_if.hpp"
#include "controller_interface_msg/msg/base_control.hpp"
#include "controller_interface_msg/msg/convergence.hpp"
#include "controller_interface_msg/msg/colorball.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"

using namespace std;

class Gamebtn{
    public:
        Gamebtn();
        void injection(bool is_injection_convergence,bool is_injection_mech_stop_m,int16_t can_inject_id,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb);
        bool injection_spining_start(std::string move_node,bool is_backside,rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_injection,int16_t can_inject_spinning_id,bool is_move_autonomous,bool is_injection_mech_stop_m,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb);
        void injection_spining_stop(int16_t can_inject_spinning_id,bool is_injection_mech_stop_m,bool is_move_autonomous,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb);
        void steer_reset(int16_t can_steer_reset_id,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb);
        void calibrate(int16_t can_calibrate_id,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb);
        void main_reset(int16_t can_reset_id,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb);
        void io_reset(int16_t can_reset_id,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb);
        void paddy_install_right(bool is_ballhand_convergence,int16_t can_paddy_install_id,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb);
        void paddy_install_left(rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_injection,int16_t can_inject_spinning_id,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb);
        void paddy_collect_right(rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_injection,int16_t can_inject_spinning_id,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb);
        void paddy_collect_left(bool is_ballhand_convergence,int16_t can_paddy_collect_id,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb);
        void initial_sequense(std::string initial_pickup_state,rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub_initial_sequense);
    private:
        // rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb;
        // rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_inject_info;
        // rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_injection;
};