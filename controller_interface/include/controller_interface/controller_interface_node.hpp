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
#include "std_msgs/msg/u_int8.hpp"
#include "controller_interface/Gamebtn.hpp"
//他のpkg
#include "utilities/can_utils.hpp"
#include "utilities/utils.hpp"
#include "socket_udp.hpp"
#include "trapezoidal_velocity_planner.hpp"

#include "visibility_control.h"

namespace controller_interface
{
    using VelPlanner = velocity_planner::trapezoidal_velocity_planner::TrapezoidalVelocityPlanner;
    using VelPlannerLimit = velocity_planner::trapezoidal_velocity_planner::Limit_t;

    class SmartphoneGamepad : public rclcpp::Node
    {
        public:
            CONTROLLER_INTERFACE_PUBLIC
            explicit SmartphoneGamepad(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
            
            CONTROLLER_INTERFACE_PUBLIC
            explicit SmartphoneGamepad(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

        private:
            //R1_mainのcontrollerから
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub_main_pad;
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub_screen_pad;
            rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _sub_inject_calibration;
            rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _sub_motor_calibration;
            rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _sub_connection_state; 

            //R1_subのcontrollerから
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub_pad;

            //mainボードから
            rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _sub_inject_convergence;
            rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _sub_paddy_convergence;
            rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _sub_seedling_convergence;
            rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _sub_emergency_state;

            //spline_pidから
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _sub_is_move_tracking;

            //injection_param_calculatorから
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _sub_calculator_convergence;

            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _sub_move_autonomous;

            //CanUsbへ
            rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb;

            //各nodeと共有
            rclcpp::Publisher<controller_interface_msg::msg::BaseControl>::SharedPtr _pub_base_control;
            rclcpp::Publisher<controller_interface_msg::msg::Convergence>::SharedPtr _pub_convergence;
            rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr _pub_injection_calculate;

            //gazebo_simulator用のpub
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _pub_gazebo;

            //sprine_pid
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub_target_node;

            rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr _pub_is_start;

            //timer
            rclcpp::TimerBase::SharedPtr _pub_heartbeat;
            rclcpp::TimerBase::SharedPtr _pub_timer_convergence;
            rclcpp::TimerBase::SharedPtr _socket_timer;
            rclcpp::TimerBase::SharedPtr _pub_state_communication_timer;
            rclcpp::TimerBase::SharedPtr check_controller_connection;
            rclcpp::TimerBase::SharedPtr check_mainboard_connection;

            //QoS
            rclcpp::QoS _qos = rclcpp::QoS(10);
            
            //controller_mainからのcallback
            void callback_main_pad(const std_msgs::msg::String::SharedPtr msg);
            void callback_screen_mainpad(const std_msgs::msg::String::SharedPtr msg);
            void callback_inject_calibration(const std_msgs::msg::Empty::SharedPtr msg);
            void callback_motor_calibration(const std_msgs::msg::Empty::SharedPtr msg);
            void callback_connection_state(const std_msgs::msg::Empty::SharedPtr msg);

            //controller_subからのcallback
            void callback_subpad(const std_msgs::msg::String::SharedPtr msg);
            void callback_screen_subpad(const std_msgs::msg::String::SharedPtr msg);
            
            //mainからのcallback
            void callback_emergency_state(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);
            void callback_inject_convergence(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);
            void callback_seedling_convergence(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);
            void callback_paddy_convergence(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);

            //splineからのcallback
            void callback_is_move_tracking(const std_msgs::msg::Bool::SharedPtr msg);

            //injection_param_calculatorからのcallback
            void callback_calculator_convergence(const std_msgs::msg::Bool::SharedPtr msg);

            void callback_move_autonomous(const std_msgs::msg::Bool::SharedPtr msg);

            void _recv_callback();

            void _recv_joy_main(const unsigned char data[16]);

            //メッセージ型の宣言
            controller_interface_msg::msg::BaseControl msg_base_control;
            controller_interface_msg::msg::Convergence msg_convergence;           

            //robotcontrol_flagはtrueのときpublishできる
            bool robotcontrol_flag = false;

            //初期化指定用
            const float high_manual_linear_max_vel;
            const float slow_manual_linear_max_vel;
            const float manual_angular_max_vel;
            
            const bool defalt_restart_flag;
            const bool defalt_move_autonomous_flag;
            const bool defalt_injection_autonomous_flag;
            const bool defalt_emergency_flag;
            const bool defalt_slow_speed_flag;

            const bool defalt_spline_convergence;
            const bool defalt_injection_calculator_convergence;
            const bool defalt_injection_convergence;
            const bool defalt_seedlinghand_convergence;
            const bool defalt_ballhand_convergence;  

            const int16_t can_emergency_id;
            const int16_t can_heartbeat_id;
            const int16_t can_restart_id;
            const int16_t can_calibrate_id;
            const int16_t can_reset_id;
            const int16_t can_emergency_state_id;   
            const int16_t can_linear_id;
            const int16_t can_angular_id;
            const int16_t can_steer_reset_id;
            const int16_t can_inject_id;
            const int16_t can_inject_convergence_id;
            const int16_t can_inject_calibration_id;
            const int16_t can_motor_calibration_id;
            const int16_t can_seedling_collect_id;
            const int16_t can_seedling_install_id;
            const int16_t can_seedling_convergence_id;
            const int16_t can_paddy_collect_id;
            const int16_t can_paddy_install_id;
            const int16_t can_paddy_convergence_id;
            const int16_t can_arm_expansion_id;
            const int16_t can_led_id;

            //計画機
            VelPlanner high_velPlanner_linear_x;
            VelPlanner high_velPlanner_linear_y;
            const VelPlannerLimit high_limit_linear;

            VelPlanner slow_velPlanner_linear_x;
            VelPlanner slow_velPlanner_linear_y;
            const VelPlannerLimit slow_limit_linear;

            VelPlanner velPlanner_angular_z;
            const VelPlannerLimit limit_angular;

            RecvUDP joy_main;

            Gamebtn gamebtn;
            bool arm_expansion_flag = false;

            std::chrono::system_clock::time_point get_controller_time;
            std::chrono::system_clock::time_point get_mainboard_time;

            const bool connection_check;

            int led_num = 0;

    };
}