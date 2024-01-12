#include "injection_param_calculator/injection_param_calculator.hpp"
#include "utilities/utils.hpp"
#include "utilities/can_utils.hpp"

namespace injection_param_calculator
{
    InjectionParamCalculator::InjectionParamCalculator(const rclcpp::NodeOptions &options) : InjectionParamCalculator("", options) {}
    InjectionParamCalculator::InjectionParamCalculator(const std::string &name_space, const rclcpp::NodeOptions &options)
        : rclcpp::Node("injection_param_calculator_node", name_space, options),
          mass(get_parameter("mass").as_double()),                                                  // リングの重量[kg]
          gravitational_accelerastion(get_parameter("gravitational_accelerastion").as_double()),    // 重力加速度[m/s^2]
          air_resistance(get_parameter("air_resistance").as_double()),                              // 空気抵抗係数[kg/s]
          foundation_hight(get_parameter("foundation_hight").as_double()),                          // 射出機構の地面からの高さ[m]
          velocity_lim_max(get_parameter("velocity_lim_max").as_double()),                          // 最大初速度[m/s]
          injection_angle(get_parameter("injection_angle").as_double()),                            // 射出角度[deg]
          max_loop(get_parameter("max_loop").as_int()),                                             // ニュートン法のループ制限回数
          can_inject_vel_id(get_parameter("canid.inject_vel").as_int())
        {
            _sub_injection_command = this->create_subscription<injection_interface_msg::msg::InjectionCommand>(
                "injection_command", _qos,
                std::bind(&InjectionParamCalculator::callback_injection,this,std::placeholders::_1)
            );

            // _sub_is_convergence = this->create_subscription<controller_interface_msg::msg::Convergence>(
            //     "convergence",
            //     _qos,
            //     std::bind(&InjectionParamCalculator::callback_is_convergence,this,std::placeholders::_1)
            // );

            _pub_can = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);
            _pub_isConvergenced = this->create_publisher<std_msgs::msg::Bool>("calculator_convergenced_", _qos);
            RCLCPP_INFO(this->get_logger(), "create injection_param");
        }

    void InjectionParamCalculator::callback_injection(const injection_interface_msg::msg::InjectionCommand::SharedPtr msg)
    {
        auto msg_injection = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_injection->canid = can_inject_vel_id;        
        msg_injection->candlc = 8;


        auto msg_isConvergenced = std::make_shared<std_msgs::msg::Bool>();
        bool isConvergenced = false;
        injection_command.distance = msg->distance;
        injection_command.height = msg->height;
        // injection_command.velocity_gain = msg->velocity_gain; //実験して使うか決める

        isConvergenced = calculateVelocity();
        msg_isConvergenced->data = isConvergenced;
        
        uint8_t _candata[4];
        float_to_bytes(_candata, static_cast<float>(velocity));
        for (int i = 0; i < msg_injection->candlc; i++)msg_injection->candata[i] = _candata[i];

        //送信
        _pub_isConvergenced->publish(*msg_isConvergenced);

        if (isConvergenced)
        {
            RCLCPP_INFO(get_logger(), "計算が収束しました");
            _pub_can->publish(*msg_injection);
        }
    }

    // void InjectionParamCalculator::callback_is_convergence(const controller_interface_msg::msg::Convergence::SharedPtr msg)
    // {
    //     is_convergence = msg->spline_convergence;
    // }

    bool InjectionParamCalculator::calculateVelocity()
    {
        bool isConvergenced = false;
        bool isAiming = false;
        int num_loop = 0;
        double old_velocity = 1;

        while (!isAiming)
        {
            double new_velocity = old_velocity - f(old_velocity) / diff(old_velocity);
            if (fabs(new_velocity - old_velocity) < eps && 0 < new_velocity && new_velocity < velocity_lim_max)
            {
                velocity = new_velocity;
                isAiming = true;
                isConvergenced = true;
                break;
            }
            old_velocity = new_velocity;
            num_loop++;
            if (num_loop > max_loop)
            {
                isAiming = false;
                isConvergenced = false;
                RCLCPP_INFO(get_logger(), "発散しました!");
                break;
            }
        }
        return isConvergenced;
    }

    double InjectionParamCalculator::f(double v0)
    {
        double m = mass;
        double g = gravitational_accelerastion;
        double k = air_resistance;
        double y0 = foundation_hight;
        double angle = utils::dtor(injection_angle);
        double x = injection_command.distance;
        double y = injection_command.height;
        return -m / k * log(cos(atan(v0 * sin(angle) * sqrt(k / (m * g)))) * cosh(1 / (v0 * sin(angle)) * sqrt(m * g / k) * (exp(k * x / m) - 1) - atan(v0 * sin(angle) * sqrt(k / (m * g))))) + y0 - y;
    }

    double InjectionParamCalculator::diff(double v0)
    {
        return (f(v0 + eps) - f(v0 - eps)) / (2.0 * eps);
    }
}