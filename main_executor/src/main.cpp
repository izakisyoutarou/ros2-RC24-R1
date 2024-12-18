#include <rclcpp/rclcpp.hpp>
#include <rcl/rcl.h>
#include <iostream>
#include "controller_interface/controller_interface_node.hpp"
#include "controller_interface/unity_interface_node.hpp"
#include "controller_interface/dualsense_interface_node.hpp"
#include "injection_interface/injection_interface_node.hpp"
#include "injection_param_calculator/injection_param_calculator.hpp"
#include "ransac_localization/ransac_localization.hpp"
#include "sequencer/sequencer_node.hpp"
#include "socketcan_interface/socketcan_interface_node.hpp"
#include "spline_pid/spline_pid_node.hpp"
#include "spline_pid/spin_turn_node.hpp"
#include "logger_converter/logger_converter_node.hpp"

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    rclcpp::executors::MultiThreadedExecutor exec;

    rclcpp::NodeOptions nodes_option;
    nodes_option.allow_undeclared_parameters(true);
    nodes_option.automatically_declare_parameters_from_overrides(true);

    auto controller_node = std::make_shared<controller_interface::SmartphoneGamepad>(nodes_option);
    auto DualSense_node = std::make_shared<controller_interface::DualSense>(nodes_option); 
    auto untiy_node = std::make_shared<controller_interface::Unity>(nodes_option);
    auto injection_interface_node = std::make_shared<injection_interface::InjectionInterface>(nodes_option);
    auto injection_param_calculator_node = std::make_shared<injection_param_calculator::InjectionParamCalculator>(nodes_option);
    auto ransac_localization = std::make_shared<self_localization::ransaclocalization>(nodes_option);
    auto sequencer_node = std::make_shared<sequencer::Sequencer>(nodes_option);
    auto socketcan_node = std::make_shared<socketcan_interface::SocketcanInterface>(nodes_option);
    auto spline_pid_node = std::make_shared<spline_pid::SplinePid>(nodes_option);
    auto spin_turn_node = std::make_shared<spline_pid::SpinTurn>(nodes_option);
    // auto logger_converter_node = std::make_shared<logger_converter::LoggerConverter>(nodes_option);
    
    // exec.add_node(controller_node);
    exec.add_node(DualSense_node);
    exec.add_node(untiy_node);
    exec.add_node(injection_interface_node);
    exec.add_node(injection_param_calculator_node);
    exec.add_node(ransac_localization);
    exec.add_node(sequencer_node);
    exec.add_node(socketcan_node);
    exec.add_node(spline_pid_node);
    exec.add_node(spin_turn_node);
    // exec.add_node(logger_converter_node);

    exec.spin();
    rclcpp::shutdown();
    return 0;
}
