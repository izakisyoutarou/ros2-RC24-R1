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

class Gamepadbtn{
    
    public:
        Gamepadbtn();
        std_msgs::msg::String main_screen_btn(const int16_t can_inject_spinning_id,const std_msgs::msg::String::SharedPtr msg, std_msgs::msg::String msg_move_node);
        controller_interface_msg::msg::Colorball sub_screen_btn(const std_msgs::msg::String::SharedPtr msg, controller_interface_msg::msg::Colorball msg_colorball_info);

        bool main_physics_btn_emergency(bool robotcontrol_flag,bool is_emergency);
        bool main_physics_btn_restart();
        bool main_physics_btn_r1(bool is_injection_convergence,bool is_injection_mech_stop_m,const int16_t can_inject_id);
        void main_physics_btn_r2();
        void main_physics_btn_r3();
        void main_physics_btn_l1();
        void main_physics_btn_l2();
        void main_physics_btn_l3();
        void main_physics_btn_right();
        void main_physics_btn_left();
        void main_physics_btn_up();
        void main_physics_btn_down();
        void main_physics_btn_a();
        void main_physics_btn_b();
        void main_physics_btn_x();
        void main_physics_btn_y();   

    private:
};
