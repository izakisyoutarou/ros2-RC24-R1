#pragma once
#include <rclcpp/rclcpp.hpp>
#include <float.h>
#include <string>
//使うmsg
#include "socketcan_interface_msg/msg/socketcan_if.hpp"
#include "controller_interface_msg/msg/base_control.hpp"
#include "controller_interface_msg/msg/convergence.hpp"
#include "controller_interface_msg/msg/colorball.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/u_int8.hpp"
//他のpkg
#include "utilities/can_utils.hpp"
#include "utilities/utils.hpp"
#include "trapezoidal_velocity_planner.hpp"
#include "controller_interface/Gamebtn.hpp"

#include "my_visibility.h"

using namespace utils;

namespace controller_interface
{
    using VelPlanner = velocity_planner::trapezoidal_velocity_planner::TrapezoidalVelocityPlanner;
    using VelPlannerLimit = velocity_planner::trapezoidal_velocity_planner::Limit_t;

    class DualSense : public rclcpp::Node
    {
        public:
            CONTROLLER_INTERFACE_PUBLIC
            explicit DualSense(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
            
            CONTROLLER_INTERFACE_PUBLIC
            explicit DualSense(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

        private:
            rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _sub_dualsense;
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub_screen_pad;
            rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _sub_emergency_state;
            rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _sub_inject_convergence;
            rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _sub_paddy_convergence;
            rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _sub_seedling_convergence;
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _sub_is_move_tracking;
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _sub_calculator_convergence;
            rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _sub_inject_calibration;
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _sub_move_autonomous;
            rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _sub_motor_calibration;
            
            rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb;
            rclcpp::Publisher<controller_interface_msg::msg::BaseControl>::SharedPtr _pub_base_control;
            rclcpp::Publisher<controller_interface_msg::msg::Convergence>::SharedPtr _pub_convergence;
            rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr _pub_injection_calculate;
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub_target_node;
            rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr _pub_is_start;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _pub_gazebo;

            rclcpp::TimerBase::SharedPtr _pub_heartbeat;
            rclcpp::TimerBase::SharedPtr _pub_timer_convergence;
            rclcpp::TimerBase::SharedPtr check_controller_connection;
            rclcpp::TimerBase::SharedPtr check_mainboard_connection;
            rclcpp::TimerBase::SharedPtr Joystick_timer;

            rclcpp::QoS _qos = rclcpp::QoS(10);

            void callback_dualsense(const sensor_msgs::msg::Joy::SharedPtr msg);
            void callback_screen_mainpad(const std_msgs::msg::String::SharedPtr msg);
            void callback_motor_calibration(const std_msgs::msg::Empty::SharedPtr msg);
            void callback_emergency_state(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);
            void callback_inject_convergence(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);
            void callback_seedling_convergence(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);
            void callback_paddy_convergence(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);
            void callback_is_move_tracking(const std_msgs::msg::Bool::SharedPtr msg);
            void callback_calculator_convergence(const std_msgs::msg::Bool::SharedPtr msg);
            void callback_inject_calibration(const std_msgs::msg::Empty::SharedPtr msg);
            void callback_move_autonomous(const std_msgs::msg::Bool::SharedPtr msg);
            void callback_Joystick();

            VelPlanner high_velPlanner_linear_x;
            VelPlanner high_velPlanner_linear_y;
            const VelPlannerLimit high_limit_linear;

            VelPlanner slow_velPlanner_linear_x;
            VelPlanner slow_velPlanner_linear_y;
            const VelPlannerLimit slow_limit_linear;

            VelPlanner velPlanner_angular_z;
            const VelPlannerLimit limit_angular;

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
            const int16_t can_arm_down_id;
            const int16_t can_led_id;

            const bool connection_check;
            bool arm_expansion_flag = false;

            int led_num = 0;

            Gamebtn gamebtn;

            controller_interface_msg::msg::BaseControl msg_base_control;
            controller_interface_msg::msg::Convergence msg_convergence;  

            std::chrono::system_clock::time_point get_controller_time;
            std::chrono::system_clock::time_point get_mainboard_time;

            //dualsenseのボタン配列メモ
            //buttons:  0:cross  1:circle  2:triangle  3:square  4:L1  5:R1  6:L2  7:R2  8:create  9:option  10:ps  11:joyL  12:joyR
            //axes:  0:joyL_x  1:joyL_y  2:L2  3:joyR_x  4:joyR_y  5:R2  6:Left(1),Right(-1)  7:Up(1),Down(-1)
            
            UpEdge upedge_buttons[13];
            UpEdge upedge_LRUD[4];
            bool buttons[13] = {};
            bool LRUD[4] = {};
            double axes[7] = {};

            enum ps5_buttons{
                cross,
                circle,
                triangle,
                square,
                l1,
                r1,
                l2,
                r2,
                create,
                option,
                ps,
                l3,
                r3               
            };

            enum ps5_axes{
                lx,
                ly,
                L2,
                rx,
                ry,
                R2,
                LR,
                UD             
            };
    
            enum ps5_LRUD{
                left,
                right,
                up,
                down           
            };
            
    };
}