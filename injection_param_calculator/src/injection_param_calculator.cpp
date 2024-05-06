#include "injection_param_calculator/injection_param_calculator.hpp"
#include "utilities/utils.hpp"
#include "utilities/can_utils.hpp"
#include <cmath>

namespace injection_param_calculator{
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

            _subscriber_injection_command = this->create_subscription<injection_interface_msg::msg::InjectionCommand>(
                "injection_command", _qos,
                std::bind(&InjectionParamCalculator::callback_injection_command,this,std::placeholders::_1)
            );

            _subscriber_param = this->create_subscription<std_msgs::msg::Float64>(
                "param", _qos,
                std::bind(&InjectionParamCalculator::callback_param,this,std::placeholders::_1)
            );

            _subscriber_gain = this->create_subscription<std_msgs::msg::Float64>(
                "gain", _qos,
                std::bind(&InjectionParamCalculator::callback_gain,this,std::placeholders::_1)
            );

            _publisher_can = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);
            _publisher_is_convergenced = this->create_publisher<std_msgs::msg::Bool>("calculator_convergenced_", _qos);

        }

    void InjectionParamCalculator::callback_injection_command(const injection_interface_msg::msg::InjectionCommand::SharedPtr msg)
    {
        auto msg_injection = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_injection->canid = can_inject_vel_id;        
        msg_injection->candlc = 8;

        auto msg_is_convergenced = std::make_shared<std_msgs::msg::Bool>();
        bool is_convergenced = false;

        injection_command.distance = msg->distance;
        injection_command.height = msg->height;
        injection_command.pitch = msg->pitch;
        injection_command.gain = msg->gain;

        is_convergenced = calculateVelocity();
        msg_is_convergenced->data = is_convergenced;
        
        uint8_t _candata[4];
        float_to_bytes(_candata, static_cast<float>(velocity*injection_command.gain));
        // float_to_bytes(_candata, static_cast<float>(velocity*vel_gain));
        for (int i = 0; i < msg_injection->candlc; i++)msg_injection->candata[i] = _candata[i];

        //送信
        _publisher_is_convergenced->publish(*msg_is_convergenced);

        if (is_convergenced){
            RCLCPP_INFO(get_logger(), "計算が収束しました:%f",velocity*injection_command.gain );
            // RCLCPP_INFO(get_logger(), "計算が収束しました:%f",velocity*vel_gain );
            _publisher_can->publish(*msg_injection);
        }
    }

    bool InjectionParamCalculator::calculateVelocity(){
        bool is_convergenced = false;
        bool isAiming = false;
        int num_loop = 0;
        double old_velocity = 1;

        while (!isAiming){
            double new_velocity = old_velocity - f(old_velocity) / diff(old_velocity);
            if (fabs(new_velocity - old_velocity) < eps && 0 < new_velocity && new_velocity < velocity_lim_max){
                velocity = new_velocity;
                isAiming = true;
                is_convergenced = true;
                break;
            }
            old_velocity = new_velocity;
            num_loop++;
            if (num_loop > max_loop){
                isAiming = false;
                is_convergenced = false;
                RCLCPP_INFO(get_logger(), "発散しました!");
                break;
            }
        }
        return is_convergenced;
    }

    double InjectionParamCalculator::f(double v0){
        double m = mass;
        double g = gravitational_accelerastion;
        double k = air_resistance;
        double omega = std::sqrt(m*g/k);
        double angle = utils::dtor(injection_command.pitch);
        double T = m/(k*omega)*std::atan(v0*std::sin(angle)/omega);
        double X = m/k*std::log(k*v0*std::cos(angle)/m*T+1);
        double y0 = 0.0;
        double x = injection_command.distance;
        double y = injection_command.height;
        return -m/k*std::log(std::cosh(k*omega/m*(std::exp(k*(x-X)/m)-1)*m/(k*v0*std::cos(angle)))*std::cos(std::atan(v0*std::sin(angle)/omega)))+y0 -y;
    }

    double InjectionParamCalculator::diff(double v0){
        return (f(v0 + eps) - f(v0 - eps)) / (2.0 * eps);
    }

    void InjectionParamCalculator::callback_param(const std_msgs::msg::Float64::SharedPtr msg){
        air_resistance = msg->data;
        RCLCPP_INFO(get_logger(), "air:%f",air_resistance);
    }

    void InjectionParamCalculator::callback_gain(const std_msgs::msg::Float64::SharedPtr msg){
        vel_gain = msg->data;
        RCLCPP_INFO(get_logger(), "gain:%f",vel_gain);
    }
}