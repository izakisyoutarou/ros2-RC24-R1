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

    class Unity : public rclcpp::Node
    {
        public:
            CONTROLLER_INTERFACE_PUBLIC
            explicit Unity(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
            
            CONTROLLER_INTERFACE_PUBLIC
            explicit Unity(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        private:

            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_base_restart;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_base_emergency;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_move_auto;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_base_injection;
            rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr _pub_base_state_communication;
            
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_con_spline;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_con_colcurator;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_con_injection;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_con_seedlinghand;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_con_ballhand;

            rclcpp::Subscription<controller_interface_msg::msg::BaseControl>::SharedPtr _sub_unity;
            
            rclcpp::Subscription<controller_interface_msg::msg::Convergence>::SharedPtr _sub_convergence_unity;

            //walltimer
            rclcpp::TimerBase::SharedPtr _pub_state_communication_timer;

            //QoS
            rclcpp::QoS _qos = rclcpp::QoS(10);

            void unity_callback(const controller_interface_msg::msg::BaseControl::SharedPtr msg);
            void convergence_unity_callback(const controller_interface_msg::msg::Convergence::SharedPtr msg);

            std_msgs::msg::Bool msg_unity_control;

            //base_control用
            bool is_reset = false;
            bool is_emergency = false;
            bool is_move_autonomous = false;
            bool is_injection_autonomous = false;
            bool is_slow_speed = false;
            bool is_injection_mech_stop_m = true;
            std::string initial_state = "";

            //convergence用
            bool is_spline_convergence;
            bool is_injection_calculator_convergence;
            bool is_injection_convergence;
            bool is_seedlinghand_convergence;
            bool is_ballhand_convergence;

            //unityにsubscrib
            bool is_reset_unity = false;
            bool is_emergency_unity = false;
            bool is_move_autonomous_unity = false;
            bool is_injection_autonomous_unity = false;
            bool is_slow_speed_unity = false;
            bool is_injection_mech_stop_m_unity = false;
            std::string initial_state_unity = "";
            
            bool spline_convergence = false;
            bool injection_calculator = false;
            bool injection = false;
            bool seedlinghand = false;
            bool ballhand = false;
            bool injection_flag = false;

            bool is_backside = false;

            bool robotcontrol_flag = false;


            const bool defalt_spline_convergence;
            const bool defalt_injection_calculator_convergence;
            const bool defalt_injection_convergence;
            const bool defalt_seedlinghand_convergence;
            const bool defalt_ballhand_convergence; 
            const bool defalt_restart_flag;
            const bool defalt_move_autonomous_flag;
            const bool defalt_injection_autonomous_flag;
            const bool defalt_emergency_flag;
            const bool defalt_slow_speed_flag;
    };

}