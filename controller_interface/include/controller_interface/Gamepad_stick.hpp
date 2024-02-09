// #pragma once
// #include <rclcpp/rclcpp.hpp>
// #include <float.h>
// #include <string>
// //使うmsg
// #include "socketcan_interface_msg/msg/socketcan_if.hpp"
// #include "geometry_msgs/msg/twist.hpp"
// #include "std_msgs/msg/bool.hpp"
// #include "std_msgs/msg/string.hpp"
// #include "std_msgs/msg/empty.hpp"
// //他のpkg
// #include "utilities/can_utils.hpp"
// #include "utilities/utils.hpp"
// #include "socket_udp.hpp"
// #include "trapezoidal_velocity_planner.hpp"

// #include <sys/time.h>
// #include <sys/types.h>
// #include <chrono>
// #include <iostream>
// #include <future>

// using VelPlanner = velocity_planner::trapezoidal_velocity_planner::TrapezoidalVelocityPlanner;
// using VelPlannerLimit = velocity_planner::trapezoidal_velocity_planner::Limit_t;

// struct msg{
//     geometry_msgs::msg::Twist *msg_gazebo;
//     socketcan_interface_msg::msg::SocketcanIF msg_linear;
//     socketcan_interface_msg::msg::SocketcanIF msg_angular;
// };
        
// class Gamepadstick{
//     public:
//         Gamepadstick();

//         msg _recv_joy_main(const unsigned char data[16],const int16_t can_linear_id,const int16_t can_angular_id,bool is_move_autonomous,bool is_slow_speed,
//                             const float high_manual_linear_max_vel,const float slow_manual_linear_max_vel,const float manual_angular_max_vel);
//         void recv();
//         void _recv_joy_main_slow();
//         void _recv_joy_main_hight();

//         //timer
//         rclcpp::TimerBase::SharedPtr _socket_timer;

//     private:
        
//         rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb;
//         rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _pub_gazebo;

//         //計画機
//         VelPlanner high_velPlanner_linear_x;
//         VelPlanner high_velPlanner_linear_y;
//         VelPlannerLimit high_limit_linear;

//         VelPlanner slow_velPlanner_linear_x;
//         VelPlanner slow_velPlanner_linear_y;
//         VelPlannerLimit slow_limit_linear;

//         VelPlanner velPlanner_angular_z;
//         VelPlannerLimit limit_angular;

//         RecvUDP recvudp;

// };