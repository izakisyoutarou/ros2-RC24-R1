#include <rclcpp/rclcpp.hpp>
#include "injection_param_calculator/my_visibility.h"

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

#include "controller_interface_msg/msg/convergence.hpp"
#include "injection_interface_msg/msg/injection_command.hpp"
#include "socketcan_interface_msg/msg/socketcan_if.hpp"


namespace injection_param_calculator
{
    class InjectionParamCalculator : public rclcpp::Node
    {
    public:
        INJECTION_PARAM_CALCULATOR_PUBLIC
        explicit InjectionParamCalculator(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

        INJECTION_PARAM_CALCULATOR_PUBLIC
        explicit InjectionParamCalculator(const std::string &name_space, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    private:
        rclcpp::Subscription<injection_interface_msg::msg::InjectionCommand>::SharedPtr _subscriber_injection_command;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr _subscriber_param;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr _subscriber_gain;

        rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _publisher_can;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _publisher_is_convergenced;

        rclcpp::QoS _qos = rclcpp::QoS(10);

        void callback_injection_command(const injection_interface_msg::msg::InjectionCommand::SharedPtr msg);
        void callback_param(const std_msgs::msg::Float64::SharedPtr msg);
        void callback_gain(const std_msgs::msg::Float64::SharedPtr msg);

        bool calculateVelocity();
        double f(double v0);
        double diff(double v0);

        injection_interface_msg::msg::InjectionCommand injection_command;
        const int16_t can_inject_vel_id;
        double velocity;
        double vel_gain = 1.0;

        const double mass;                                    // リングの重量[kg]
        const double gravitational_accelerastion;             // 重力加速度[m/s^2]
        double air_resistance;                          // 空気抵抗係数[kg/s]
        const double foundation_hight;                        // 射出機構の地面からの高さ[m]
        const double velocity_lim_max;                        // 最大初速度[m/s]
        const double injection_angle;                         // 射出角度[deg]
        const int max_loop;                                   // ニュートン法のループ制限回数
        const double eps = 0.000001;                          // 微分で使用
        bool is_convergence;                                  // 収束 : true  発散 : false
    };
}