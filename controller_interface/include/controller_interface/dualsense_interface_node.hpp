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
//他のpkg
#include "utilities/can_utils.hpp"
#include "utilities/utils.hpp"
#include "socket_udp.hpp"
#include "trapezoidal_velocity_planner.hpp"

#include "send_udp.hpp"
#include "super_command.hpp"

#include "visibility_control.h"


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
            rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _sub_dualsense_main;
            //ER_subのcontrollerから
            rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _sub_dualsense_sub;

            //mainボードから
            rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _sub_main_injection_possible;
            rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _sub_main_ballhand_possible;
            rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _sub_main_Seedlinghand_possible;

            //spline_pidから
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _sub_spline;


            //injection_param_calculatorから
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _sub_injection_calculator;
            //sequencerから
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub_injection_strange;

            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub_collection_ball;

            //CanUsbへ
            rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb;

            //各nodeと共有
            rclcpp::Publisher<controller_interface_msg::msg::BaseControl>::SharedPtr _pub_base_control;
            rclcpp::Publisher<controller_interface_msg::msg::Convergence>::SharedPtr _pub_convergence;
            rclcpp::Publisher<controller_interface_msg::msg::Colorball>::SharedPtr _pub_color_ball;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_injection;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_coat_state;

            //ボールと苗の回収&設置
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_seedling_collection;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_seedling_installation;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_ball_collection;

            //gazebo_simulator用のpub
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _pub_gazebo;

            //main_controlerからのsubscriber
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub_initial_state;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_base_restart;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_base_emergency;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_move_auto;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_base_injection;
            
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_con_spline;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_con_colcurator;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_con_injection;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_con_seedlinghand;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_con_ballhand;

            //sprine_pid
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_move_node;

            //timer
            rclcpp::TimerBase::SharedPtr _pub_heartbeat;
            rclcpp::TimerBase::SharedPtr _pub_timer_convergence;
            rclcpp::TimerBase::SharedPtr _socket_timer;
            rclcpp::TimerBase::SharedPtr _start_timer;

            //QoS
            rclcpp::QoS _qos = rclcpp::QoS(10);
            
            //controller_mainからのcallback
            void callback_dualsense_main(const sensor_msgs::msg::Joy::SharedPtr msg);
            void callback_dualsense_sub(const sensor_msgs::msg::Joy::SharedPtr msg);


            //controller_subからのcallback
            void callback_sub_pad(const std_msgs::msg::String::SharedPtr msg);
            void callback_sub_gamepad(const std_msgs::msg::String::SharedPtr msg);

            //mainからのcallback
            void callback_main_injection_possible(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);
            void callback_main_Seedlinghand_possible(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);
            void callback_main_ballhand_possible(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);
            void callback_injection_complete(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);

            //splineからのcallback
            void callback_spline(const std_msgs::msg::Bool::SharedPtr msg);

            //injection_param_calculatorからのcallback
            void callback_injection_calculator(const std_msgs::msg::Bool::SharedPtr msg);
            //void callback_calculator_convergenced_arm(const std_msgs::Bool::SharedPtr msg);
            void callback_initial_state(const std_msgs::msg::String::SharedPtr msg);

            //sequencerからのcallback
            void callback_injection_strage(const std_msgs::msg::String::SharedPtr msg);
            void callback_collecting_ball(const std_msgs::msg::String::SharedPtr msg);
            void _recv_callback();


            void _recv_joy_main(const unsigned char data[16]);

            //メッセージ型の宣言
            controller_interface_msg::msg::BaseControl msg_base_control;
            controller_interface_msg::msg::Colorball msg_colorball_info;
            std_msgs::msg::Bool msg_unity_control;
            std_msgs::msg::Bool msg_unity_sub_control;
            std_msgs::msg::String msg_unity_initial_state;


            //base_control用
            bool is_reset = false;
            bool is_emergency = false;
            bool is_move_autonomous = false;
            bool is_injection_autonomous = false;
            bool is_slow_speed = false;
            bool is_injection_mech_stop_m = false;
            std::string initial_state = "";

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


            //canusb
            bool a;
            bool b;
            bool y;
            bool x;
            bool r1;
            bool r2;
            bool r3;
            bool l1;
            bool l2;
            bool l3;
            bool s;
            bool g;
            bool up;
            bool left;
            bool down;
            bool right;

            //robotcontrol_flagはtrueのときpublishできる
            bool robotcontrol_flag = false;



            //convergence用
            bool is_spline_convergence;
            bool is_injection_calculator_convergence;
            bool is_injection_convergence;
            bool is_seedlinghand_convergence;
            bool is_ballhand_convergence;

            //初期化指定用
            const float high_manual_linear_max_vel;
            const float slow_manual_linear_max_vel;
            const float manual_angular_max_vel;
            
            const bool defalt_restart_flag;
            const bool defalt_move_autonomous_flag;
            const bool defalt_injection_autonomous_flag;
            const bool defalt_emergency_flag;
            const bool defalt_slow_speed_flag;
            //subcontrollerのカラー情報
            const bool defalt_color_information_flag;

            const bool defalt_spline_convergence;
            const bool defalt_injection_calculator_convergence;
            const bool defalt_injection_convergence;
            const bool defalt_seedlinghand_convergence;
            const bool defalt_ballhand_convergence;            
            const int16_t can_emergency_id;
            const int16_t can_heartbeat_id;
            const int16_t can_restart_id;
            const int16_t can_calibrate_id;
            const int16_t can_linear_id;
            const int16_t can_angular_id;
            const int16_t can_main_button_id;
            const int16_t can_sub_button_id;
            const int16_t can_inject_id;
            const int16_t can_inject_spinning_id;
            const int16_t can_seedling_collect_id;
            const int16_t can_seedling_install_id;
            const int16_t can_paddy_collect_id;
            const int16_t can_paddy_install_id;
            const int16_t can_steer_reset_id;
            const int16_t can_reset_id;


            const std::string r1_pc;
            const std::string r2_pc;

            const std::string initial_pickup_state;
            const std::string initial_inject_state;

            //udp初期化用
            const int udp_port_state;
            bool start_r1_main;
            bool start_flag;

            //Joyの値を格納(main)
            float analog_l_x_main = 0.0f;
            float analog_l_y_main = 0.0f;
            float analog_r_x_main = 0.0f;
            float analog_r_y_main = 0.0f;

            //Joyの値を格納(sub)
            float analog_l_x_sub = 0.0f;
            float analog_l_y_sub = 0.0f;
            float analog_r_x_sub = 0.0f;
            float analog_r_y_sub = 0.0f;

            std::string move_node_string;
            std::string before_pole_string_m0;
            std::string before_pole_string_m1;
            bool move_node_lock;
            bool injection0_lock;
            bool injection1_lock;

            //計画機
            VelPlanner high_velPlanner_linear_x;
            VelPlanner high_velPlanner_linear_y;
            const VelPlannerLimit high_limit_linear;

            VelPlanner slow_velPlanner_linear_x;
            VelPlanner slow_velPlanner_linear_y;
            const VelPlannerLimit slow_limit_linear;

            VelPlanner velPlanner_angular_z;
            const VelPlannerLimit limit_angular;

            class UpEdge{
                public:
	                bool operator()(bool value){
                        if(!oldValue && value){
                            oldValue=value;
                            return true;
                        }else{
                            oldValue=value;
                            return false;
                        }
	                };
                private:
	                bool oldValue = true;
            };
            
            UpEdge upedge_autonomous_main;
            UpEdge upedge_emergency_main;
            UpEdge upedge_restart_main;
            UpEdge upedge_ps_main;
            UpEdge upedge_r1_main;
            UpEdge upedge_r2_main;
            UpEdge upedge_r3_main;
            UpEdge upedge_l1_main;
            UpEdge upedge_l2_main;
            UpEdge upedge_l3_main;
            UpEdge upedge_up_main;
            UpEdge upedge_down_main;
            UpEdge upedge_right_main;
            UpEdge upedge_left_main;
            UpEdge upedge_axes6_main;
            UpEdge upedge_axes7_main;
            UpEdge upedge_autonomous_sub;
            UpEdge upedge_emergency_sub;
            UpEdge upedge_restart_sub;
            UpEdge upedge_ps_sub;
            UpEdge upedge_r1_sub;
            UpEdge upedge_r2_sub;
            UpEdge upedge_r3_sub;
            UpEdge upedge_l1_sub;
            UpEdge upedge_l2_sub;
            UpEdge upedge_l3_sub;
            UpEdge upedge_buttons0_sub;
            UpEdge upedge_buttons1_sub;
            UpEdge upedge_buttons2_sub;
            UpEdge upedge_buttons3_sub;
            UpEdge upedge_axes6_sub;
            UpEdge upedge_axes7_sub;

            super_command command;
    };
}