#pragma once
#include <rclcpp/rclcpp.hpp>
#include <float.h>
#include <string>
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

        struct CANID{
            int16_t calibrate;
            int16_t reset;
            int16_t inject;
            int16_t seedling_collect;
            int16_t seedling_install;
            int16_t paddy_collect;
            int16_t paddy_install;
            int16_t steer_reset;
            int16_t arm_expansion;
            int16_t arm_down;
            int16_t inject_calibration;
            int16_t led;
        };
        CANID canid;
        //mainコントローラー物理ボタン
        void calibrate(rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb);
        void board_reset(rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb);
        void steer_reset(rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb);
        void injection_calculate(rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr _pub_injection_calculate);
        void injection(bool is_injection_convergence, bool injection_calculator_convergence, rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb);
        void seedling_collect_right(bool is_seedlinghand_convergence,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb);
        void seedling_collect_left(bool is_seedlinghand_convergence,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb);
        void seedling_install_right(bool is_seedlinghand_convergence,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb);
        void seedling_install_right0(bool is_seedlinghand_convergence,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb);
        void seedling_install_right1(bool is_seedlinghand_convergence,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb);
        void seedling_install_left(bool is_seedlinghand_convergence,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb);
        void seedling_install_left0(bool is_seedlinghand_convergence,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb);
        void seedling_install_left1(bool is_seedlinghand_convergence,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb);
        void paddy_control(bool is_ballhand_convergence,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb);
        void paddy_collect(rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb);
        void paddy_install(rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb);
        void initial_sequense(std::string initial_pickup_state,rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub_initial_sequense);
        void arm_expansion(rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb);
        void arm_down(rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb);
        void inject_calibration(rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb);
        void led(uint8_t led_num, rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb);

    private:
        int seed_right_flag = 0;
        int seed_left_flag = 0;
        bool paddy_flag = true; 
        int pre_num = 100;
};