#include "controller_interface/Gamebtn.hpp"
#include <rclcpp/rclcpp.hpp>

Gamebtn::Gamebtn(){}

void Gamebtn::injection(bool is_injection_convergence,bool is_injection_mech_stop_m,int16_t can_inject_id,
                        rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    if(is_injection_convergence && !is_injection_mech_stop_m){
        auto msg_inject = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_inject->canid = can_inject_id;
        msg_inject->candlc = 0;
        _pub_canusb->publish(*msg_inject);
    }
}

bool Gamebtn::injection_spining_start(std::string move_node,bool is_backside,rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_injection,int16_t can_inject_spinning_id,
                                        bool is_move_autonomous,bool is_injection_mech_stop_m,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    if(move_node.at(0) == 'H'){
    //injectionで入れる
        is_backside = true;

        return is_backside;
    }
    else if(move_node.at(0) == 'I'){
        auto msg_injection = std::make_shared<std_msgs::msg::Bool>();
        msg_injection->data = false;
        _pub_injection->publish(*msg_injection);
        is_backside = false;
        return is_backside;
    }
    auto msg_inject_spinning = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_inject_spinning->canid = can_inject_spinning_id;
    msg_inject_spinning->candlc = 1;
    msg_inject_spinning->candata[0] = true;
    _pub_canusb->publish(*msg_inject_spinning);
}

void Gamebtn::injection_spining_stop(int16_t can_inject_spinning_id,bool is_injection_mech_stop_m,bool is_move_autonomous,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    auto msg_inject_spinning = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_inject_spinning->canid = can_inject_spinning_id;
    msg_inject_spinning->candlc = 1;
    msg_inject_spinning->candata[0] = false;
    _pub_canusb->publish(*msg_inject_spinning);
}

void Gamebtn::steer_reset(int16_t can_steer_reset_id,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    auto msg_steer_reset = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_steer_reset->canid = can_steer_reset_id;
    msg_steer_reset->candlc = 0;
    _pub_canusb->publish(*msg_steer_reset);
}

void Gamebtn::calibrate(int16_t can_calibrate_id,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    auto msg_calibrate = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_calibrate->canid = can_calibrate_id;
    msg_calibrate->candlc = 0;
    _pub_canusb->publish(*msg_calibrate);
}

void Gamebtn::main_reset(int16_t can_reset_id,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    auto msg_main_reset = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_main_reset->canid = can_reset_id;
    msg_main_reset->candlc = 1;
    msg_main_reset->candata[0] = 0;
    _pub_canusb->publish(*msg_main_reset);
}

void Gamebtn::io_reset(int16_t can_reset_id,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    auto msg_io_reset = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_io_reset->canid = can_reset_id;
    msg_io_reset->candlc = 1;
    msg_io_reset->candata[0] = 1;
    _pub_canusb->publish(*msg_io_reset);
}

void Gamebtn::paddy_install_right(bool is_ballhand_convergence,int16_t can_paddy_install_id,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    if(is_ballhand_convergence){
        auto msg_paddy_install = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_paddy_install->candlc = 1;
        msg_paddy_install->candata[0] = true;
        msg_paddy_install->canid = can_paddy_install_id;
        _pub_canusb->publish(*msg_paddy_install);
    }
}

void Gamebtn::paddy_install_left(rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_injection,int16_t can_inject_spinning_id,
                                rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    auto msg_injection = std::make_shared<std_msgs::msg::Bool>();
    msg_injection->data = false;
    _pub_injection->publish(*msg_injection);
    auto msg_inject_spinning = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_inject_spinning->canid = can_inject_spinning_id;
    msg_inject_spinning->candlc = 1;
    msg_inject_spinning->candata[0] = true;
    _pub_canusb->publish(*msg_inject_spinning);
}

void Gamebtn::paddy_collect_right(rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_injection,int16_t can_inject_spinning_id,
                                    rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    auto msg_injection = std::make_shared<std_msgs::msg::Bool>();
    msg_injection->data = true;
    _pub_injection->publish(*msg_injection);
    auto msg_inject_spinning = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_inject_spinning->canid = can_inject_spinning_id;
    msg_inject_spinning->candlc = 1;
    msg_inject_spinning->candata[0] = true;
    _pub_canusb->publish(*msg_inject_spinning);
}

void Gamebtn::paddy_collect_left(bool is_ballhand_convergence,int16_t can_paddy_collect_id,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    if(is_ballhand_convergence){
        auto msg_paddy_collect = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_paddy_collect->candlc = 1;
        msg_paddy_collect->candata[0] = false;
        msg_paddy_collect->canid = can_paddy_collect_id;
        _pub_canusb->publish(*msg_paddy_collect);
    }
}

void Gamebtn::initial_sequense(std::string initial_pickup_state,rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub_initial_sequense){
    auto initial_sequense_pickup = std::make_shared<std_msgs::msg::String>();
    initial_sequense_pickup->data = initial_pickup_state;
    _pub_initial_sequense->publish(*initial_sequense_pickup);
}

void Gamebtn::seedling_collect_debug(int16_t can_seedling_collect_id,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    auto msg_seedling_collect = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_seedling_collect->candlc = 1;
    msg_seedling_collect->candata[0] = 0;
    msg_seedling_collect->canid = can_seedling_collect_id;
    _pub_canusb->publish(*msg_seedling_collect);
}

void Gamebtn::seedling_collect_debug_1(int16_t can_seedling_collect_id,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    auto msg_seedling_collect = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_seedling_collect->candlc = 1;
    msg_seedling_collect->candata[0] = 1;
    msg_seedling_collect->canid = can_seedling_collect_id;
    _pub_canusb->publish(*msg_seedling_collect);
}

void Gamebtn::seedling_install_debug(int16_t can_seedling_install_id,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    auto msg_seedling_install = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_seedling_install->candlc = 1;
    msg_seedling_install->candata[0] = 0;
    msg_seedling_install->canid = can_seedling_install_id;
    _pub_canusb->publish(*msg_seedling_install);
}

void Gamebtn::seedling_install_debug_1(int16_t can_seedling_install_id,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    auto msg_seedling_install = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_seedling_install->candlc = 1;
    msg_seedling_install->candata[0] = 1;
    msg_seedling_install->canid = can_seedling_install_id;
    _pub_canusb->publish(*msg_seedling_install);
}

void Gamebtn::seedling_install_debug_2(int16_t can_seedling_install_id,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    auto msg_seedling_install = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_seedling_install->candlc = 1;
    msg_seedling_install->candata[0] = 2;
    msg_seedling_install->canid = can_seedling_install_id;
    _pub_canusb->publish(*msg_seedling_install);
}

void Gamebtn::seedling_install_debug_3(int16_t can_seedling_install_id,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    auto msg_seedling_install = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_seedling_install->candlc = 1;
    msg_seedling_install->candata[0] = 3;
    msg_seedling_install->canid = can_seedling_install_id;
    _pub_canusb->publish(*msg_seedling_install);
}