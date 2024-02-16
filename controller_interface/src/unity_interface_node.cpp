#include "controller_interface/unity_interface_node.hpp"
#include <sys/time.h>
#include <sys/types.h>
#include <chrono>
#include <iostream>
#include <future>

namespace controller_interface
{
    using std::string;

    Unity::Unity(const rclcpp::NodeOptions &options) : Unity("",options) {}
    Unity::Unity(const std::string &name_space, const rclcpp::NodeOptions &options): rclcpp::Node("controller_interface_node", name_space, options),
    //リスタート
    defalt_restart_flag(get_parameter("defalt_restart_flag").as_bool()),
    //緊急停止
    defalt_emergency_flag(get_parameter("defalt_emergency_flag").as_bool()),
    //自動化
    defalt_move_autonomous_flag(get_parameter("defalt_move_autonomous_flag").as_bool()),
    //自動射出
    defalt_injection_autonomous_flag(get_parameter("defalt_injection_autonomous_flag").as_bool()),
    //低速モード
    defalt_slow_speed_flag(get_parameter("defalt_slow_speed_flag").as_bool()),
    //収束の確認
    defalt_spline_convergence(get_parameter("defalt_spline_convergence").as_bool()),

    defalt_injection_calculator_convergence(get_parameter("defalt_injection_calculator_convergence").as_bool()),
    //射出のyaw軸が目標の値まで移動して停止した状態を保存するための変数
    defalt_injection_convergence(get_parameter("defalt_injection_convergence").as_bool()),
    //苗ハンドの収束状況を保存するための変数
    defalt_seedlinghand_convergence(get_parameter("defalt_seedlinghand_convergence").as_bool()),
    //ボール回収ハンドの収束状況を保存するための変数
    defalt_ballhand_convergence(get_parameter("defalt_ballhand_convergence").as_bool())

    {
        const auto base_state_communication_ms = this->get_parameter("base_state_communication_ms").as_int();
        const auto convergence_ms = this->get_parameter("convergence_ms").as_int();

        //controller_mainからsub
        _sub_unity = this->create_subscription<controller_interface_msg::msg::BaseControl>(
            "base_control",
            _qos,
            std::bind(&Unity::unity_callback, this, std::placeholders::_1)
        );
        _sub_convergence_unity = this->create_subscription<controller_interface_msg::msg::Convergence>(
            "convergence",
            _qos,
            std::bind(&Unity::convergence_unity_callback, this, std::placeholders::_1)
        );

        _pub_base_restart = this->create_publisher<std_msgs::msg::Bool>("restart_unity", _qos);
        _pub_base_emergency = this->create_publisher<std_msgs::msg::Bool>("emergency_unity", _qos);
        _pub_move_auto = this->create_publisher<std_msgs::msg::Bool>("move_autonomous_unity", _qos);
        _pub_base_injection = this->create_publisher<std_msgs::msg::Bool>("injection_autonomous_unity", _qos);

        _pub_con_spline = this->create_publisher<std_msgs::msg::Bool>("spline_convergence_unity", _qos);
        _pub_con_colcurator = this->create_publisher<std_msgs::msg::Bool>("injection_calcurator_unity", _qos);
        _pub_con_injection = this->create_publisher<std_msgs::msg::Bool>("injection_convergence_unity", _qos);
        _pub_con_seedlinghand = this->create_publisher<std_msgs::msg::Bool>("seedlinghand_convergence_unity", _qos);
        _pub_con_ballhand = this->create_publisher<std_msgs::msg::Bool>("ballhand_convergence_unity", _qos);
        _pub_base_state_communication = this->create_publisher<std_msgs::msg::Empty>("state_communication_unity", _qos);

        this->is_reset = defalt_restart_flag;
        this->is_emergency = defalt_emergency_flag;
        this->is_move_autonomous = defalt_move_autonomous_flag;
        this->is_injection_autonomous = defalt_injection_autonomous_flag;
        this->spline_convergence = defalt_spline_convergence;
        this->injection_calculator = defalt_injection_calculator_convergence;
        this->injection = defalt_injection_convergence;
        this->seedlinghand = defalt_seedlinghand_convergence;
        this->ballhand = defalt_ballhand_convergence;


        auto msg_unity_control = std::make_shared<std_msgs::msg::Bool>();
        msg_unity_control->data = is_reset;
        _pub_base_restart->publish(*msg_unity_control);

        msg_unity_control->data = is_emergency;
        _pub_base_emergency->publish(*msg_unity_control);

        msg_unity_control->data = is_move_autonomous;
        _pub_move_auto->publish(*msg_unity_control);

        msg_unity_control->data = is_injection_autonomous;
        _pub_base_injection->publish(*msg_unity_control);

        msg_unity_control->data = spline_convergence;
        _pub_con_spline->publish(*msg_unity_control);

        msg_unity_control->data = injection_calculator;
        _pub_con_colcurator->publish(*msg_unity_control);

        msg_unity_control->data = seedlinghand;
        _pub_con_seedlinghand->publish(*msg_unity_control);

        msg_unity_control->data = ballhand;
        _pub_con_ballhand->publish(*msg_unity_control);

        //スマホコントローラとの通信状況を確認
        _pub_state_communication_timer = create_wall_timer(
            std::chrono::milliseconds(base_state_communication_ms),
            [this] {
                auto msg_base_state_communication = std::make_shared<std_msgs::msg::Empty>();
                _pub_base_state_communication->publish(*msg_base_state_communication);
            }
        );
    }

    void Unity::unity_callback(const controller_interface_msg::msg::BaseControl::SharedPtr msg){

        msg_unity_control.data = msg->is_restart;
        _pub_base_restart->publish(msg_unity_control);

        msg_unity_control.data = msg->is_emergency;
        _pub_base_emergency->publish(msg_unity_control);

        msg_unity_control.data = msg->is_move_autonomous;
        _pub_move_auto->publish(msg_unity_control);

        msg_unity_control.data = msg->is_injection_autonomous;
        _pub_base_injection->publish(msg_unity_control);
    }
    void Unity::convergence_unity_callback(const controller_interface_msg::msg::Convergence::SharedPtr msg){
        
        auto msg_unity_control = std::make_shared<std_msgs::msg::Bool>();
        msg_unity_control->data = msg->spline_convergence;
        _pub_con_spline->publish(*msg_unity_control);
        msg_unity_control->data = msg->injection_calculator;
        _pub_con_colcurator->publish(*msg_unity_control);
        msg_unity_control->data = msg->injection;
        _pub_con_injection->publish(*msg_unity_control);
        msg_unity_control->data = msg->seedlinghand;
        _pub_con_seedlinghand->publish(*msg_unity_control);
        msg_unity_control->data = msg->ballhand;
        _pub_con_ballhand->publish(*msg_unity_control);
    }
}