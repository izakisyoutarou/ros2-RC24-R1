#pragma once
#include "sequencer/visibility_control.h"
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <chrono>
#include <vector>
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include <geometry_msgs/msg/vector3.hpp>
#include "controller_interface_msg/msg/convergence.hpp"
#include "controller_interface_msg/msg/colorball.hpp"
#include "controller_interface_msg/msg/base_control.hpp"
#include "socketcan_interface_msg/msg/socketcan_if.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace sequencer{

class Sequencer : public rclcpp::Node {
public:

    SEQUENCER_PUBLIC
    explicit Sequencer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    SEQUENCER_PUBLIC
    explicit Sequencer(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:

    enum class SEQUENCE_MODE{
        stop,
        planting,
        harvesting,
        comeback,
        evacuate
    } sequence_mode = SEQUENCE_MODE::stop;

    rclcpp::Subscription<controller_interface_msg::msg::Convergence>::SharedPtr _subscription_convergence;
    rclcpp::Subscription<controller_interface_msg::msg::BaseControl>::SharedPtr _subscription_base_control;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _subscription_target_node;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _subscription_way_point;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr _subscription_is_start;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr _subscription_self_pose;

    void callback_convergence(const controller_interface_msg::msg::Convergence::SharedPtr msg);
    void callback_base_control(const controller_interface_msg::msg::BaseControl::SharedPtr msg);
    void callback_target_node(const std_msgs::msg::String::SharedPtr msg);
    void callback_is_start(const std_msgs::msg::UInt8::SharedPtr msg);
    void callback_way_point(const std_msgs::msg::String::SharedPtr msg);
    void _subscriber_callback_self_pose(const geometry_msgs::msg::Vector3::SharedPtr msg);

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher_move_node;
    rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _publisher_canusb;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _publisher_move_autonomous;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher_move_interrupt_node;

    void command_sequence(SEQUENCE_MODE sequence);
    void command_move_node(const std::string node);
    void command_move_interrupt_node(const std::string node);
    
    void command_canusb_empty(const int16_t id);
    void command_canusb_uint8(const int16_t id, const uint8_t data);

    void command_seedling_collect_right();
    void command_seedling_collect_left();
    
    void command_seedling_install_right_0();
    void command_seedling_install_right_1();
    void command_seedling_install_left_0();
    void command_seedling_install_left_1();

    void command_paddy_collect();
    void command_paddy_install();
    void command_arm_expansion();
    void command_arm_up();
    void command_move_autonomous(bool flag);

    void timer(int ms);
    bool timer();

    rclcpp::QoS _qos = rclcpp::QoS(10);

    const int16_t canid_inject;
    const int16_t canid_seedling_collect;
    const int16_t canid_seedling_install;
    const int16_t canid_paddy_collect;
    const int16_t canid_paddy_install;
    const int16_t canid_arm_expansion;
    const int16_t canid_arm_up;

    std::string target_node = "";
    std::string pre_target_node = "";
    int progress = 0;

    bool tof[3] = {};
    std::string way_point = "";

    const std::string court_color;

    std::chrono::system_clock::time_point time;

    int check_time = 0;

    std::string way = "";

    geometry_msgs::msg::Vector3 self_pose;

    bool arm_up_flag = false;

    int sequence_count = 0;

    bool move_autonomous = false;
};

}  // namespace sequencer