#pragma once
#include <rclcpp/rclcpp.hpp>
#include <float.h>
#include <string>
//使う
#include "socketcan_interface_msg/msg/socketcan_if.hpp"
#include "controller_interface_msg/msg/base_control.hpp"
#include "controller_interface_msg/msg/convergence.hpp"
#include "controller_interface_msg/msg/colorball.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
//他のpkg
#include "utilities/can_utils.hpp"
#include "utilities/utils.hpp"
#include "socket_udp.hpp"
#include "trapezoidal_velocity_planner.hpp"

#include <sys/time.h>
#include <sys/types.h>
#include <chrono>
#include <iostream>
#include <future>

using namespace std;

// struct Variable{
//      std_msgs::msg::String::SharedPtr msg;
//      int16_t can_restart_id;
//      int16_t can_emergency_id;
//      int16_t can_main_button_id;
//      int16_t can_inject_id;
//      int16_t can_inject_spinning_id;
//      int16_t can_steer_reset_id;
//      int16_t can_calibrate_id;
//      int16_t can_reset_id;
//      int16_t can_paddy_install_id;
//      int16_t can_paddy_collect_id;
//      std::string initial_pickup_state;
//     std::string initial_state = "";
//     bool robotcontrol_flag;
//     bool is_emergency;
//     bool is_injection_mech_stop_m;
//     bool is_move_autonomous;
//     bool defalt_move_autonomous_flag;
//     bool is_injection_autonomous;
//     bool defalt_injection_autonomous_flag;
//     bool is_slow_speed;
//     bool defalt_slow_speed_flag;
//     bool is_spline_convergence;
//     bool defalt_spline_convergence;
//     bool is_injection_calculator_convergence;
//     bool defalt_injection_calculator_convergence;
//     bool is_seedlinghand_convergence;
//     bool is_injection_convergence;
//     bool defalt_injection_convergence;
//     bool defalt_seedlinghand_convergence;
//     bool is_ballhand_convergence;
//     bool defalt_ballhand_convergence;
//     bool is_backside;
//     bool is_reset;
//     rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb;
//     rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_injection;
//     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub_initial_sequense;
//     rclcpp::Publisher<controller_interface_msg::msg::BaseControl>::SharedPtr _pub_base_control;
//     rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_base_restart;
//     rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_base_emergency;
//     rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_move_auto;
//     rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_base_injection;
//     controller_interface_msg::msg::BaseControl msg_base_control;
//     std_msgs::msg::Bool msg_unity_control;
//     };

//     struct Sub_variable{
//         rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb;
//          std_msgs::msg::String::SharedPtr msg;
//          int16_t can_seedling_collect_id;
//          int16_t can_seedling_install_id;
//         bool is_seedlinghand_convergence;
//     };

class Gamepadbtn{
    public:
        Gamepadbtn();
        std_msgs::msg::String main_screen_btn(const int16_t can_inject_spinning_id,const std_msgs::msg::String::SharedPtr msg, std_msgs::msg::String msg_move_node);
        // void main_physics_btn(Variable variable);
        controller_interface_msg::msg::Colorball sub_screen_btn(const std_msgs::msg::String::SharedPtr msg, controller_interface_msg::msg::Colorball msg_colorball_info);
        // void sub_physics_btn(Sub_variable sub_variable);

        
    private:
};
