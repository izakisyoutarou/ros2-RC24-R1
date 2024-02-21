#include "controller_interface/Gamebtn.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>

Gamebtn::Gamebtn(){}

void Gamebtn::calibrate(rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    cout<<"calibrate"<<endl;
    auto msg_calibrate = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_calibrate->canid = canid.calibrate;
    msg_calibrate->candlc = 0;
    _pub_canusb->publish(*msg_calibrate);
}

void Gamebtn::main_reset(rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    cout<<"main_reset"<<endl;
    auto msg_main_reset = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_main_reset->canid = canid.reset;
    msg_main_reset->candlc = 1;
    msg_main_reset->candata[0] = 0;
    _pub_canusb->publish(*msg_main_reset);
}

void Gamebtn::io_reset(rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    cout<<"io_reset"<<endl;
    auto msg_io_reset = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_io_reset->canid = canid.reset;
    msg_io_reset->candlc = 1;
    msg_io_reset->candata[0] = 1;
    _pub_canusb->publish(*msg_io_reset);
}

void Gamebtn::steer_reset(rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    cout<<"steer_reset"<<endl;
    auto msg_steer_reset = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_steer_reset->canid = canid.steer_reset;
    msg_steer_reset->candlc = 0;
    _pub_canusb->publish(*msg_steer_reset);
}

void Gamebtn::injection_spining_start(std::string move_node,rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr _pub_backspin_injection,rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_injection,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    cout<<"spining_start"<<endl;    
    auto msg_inject_spinning = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_inject_spinning->canid = canid.inject_spinning;
    msg_inject_spinning->candlc = 1;
    msg_inject_spinning->candata[0] = true;
    _pub_canusb->publish(*msg_inject_spinning);
}

void Gamebtn::injection_spining_stop(rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    cout<<"spining_stop"<<endl;
    auto msg_inject_spinning = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_inject_spinning->canid = canid.inject_spinning;
    msg_inject_spinning->candlc = 1;
    msg_inject_spinning->candata[0] = false;
    _pub_canusb->publish(*msg_inject_spinning);
}

void Gamebtn::injection(bool is_injection_convergence,bool is_injection_mech_stop_m, rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    if(is_injection_convergence && !is_injection_mech_stop_m){
        cout<<"injection"<<endl;        
        auto msg_inject = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_inject->canid = canid.inject;
        msg_inject->candlc = 0;
        _pub_canusb->publish(*msg_inject);
    }
}

void Gamebtn::paddy_collect_right(bool is_ballhand_convergence,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    if(is_ballhand_convergence){
        cout<<"paddy_collect_right"<<endl;
        auto msg_paddy_collect = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_paddy_collect->candlc = 1;
        msg_paddy_collect->candata[0] = 0;
        msg_paddy_collect->canid = canid.paddy_collect;
        _pub_canusb->publish(*msg_paddy_collect);
    }
}

void Gamebtn::paddy_collect_left(bool is_ballhand_convergence,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    if(is_ballhand_convergence){
        cout<<"paddy_collect_left"<<endl;
        auto msg_paddy_collect = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_paddy_collect->candlc = 1;
        msg_paddy_collect->candata[0] = 1;
        msg_paddy_collect->canid = canid.paddy_collect;
        _pub_canusb->publish(*msg_paddy_collect);
    }
}

void Gamebtn::paddy_install_right(bool is_ballhand_convergence,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    if(is_ballhand_convergence){
        cout<<"paddy_install_right"<<endl;
        auto msg_paddy_install = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_paddy_install->candlc = 1;
        msg_paddy_install->candata[0] = 0;
        msg_paddy_install->canid = canid.paddy_install;
        _pub_canusb->publish(*msg_paddy_install);
    }
}

void Gamebtn::paddy_install_left(bool is_ballhand_convergence,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    if(is_ballhand_convergence){
        cout<<"paddy_install_left"<<endl;
        auto msg_paddy_install = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_paddy_install->candlc = 1;
        msg_paddy_install->candata[0] = 1;
        msg_paddy_install->canid = canid.paddy_install;
        _pub_canusb->publish(*msg_paddy_install);
    }
}

void Gamebtn::seedling_collect_0(bool is_seedlinghand_convergence, rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    if(is_seedlinghand_convergence){
        cout<<"seedling_collect_0"<<endl;
        auto msg_seedling_collect = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_seedling_collect->candlc = 1;
        msg_seedling_collect->candata[0] = 0;
        msg_seedling_collect->canid = canid.seedling_collect;
        _pub_canusb->publish(*msg_seedling_collect);
    }
}

void Gamebtn::seedling_collect_1(bool is_seedlinghand_convergence, rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    if(is_seedlinghand_convergence){
        cout<<"seedling_collect_1"<<endl;
        auto msg_seedling_collect = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_seedling_collect->candlc = 1;
        msg_seedling_collect->candata[0] = 1;
        msg_seedling_collect->canid = canid.seedling_collect;
        _pub_canusb->publish(*msg_seedling_collect);
    }
}

void Gamebtn::seedling_install_0(bool is_seedlinghand_convergence, rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    if(is_seedlinghand_convergence){
        cout<<"seedling_install_0"<<endl;    
        auto msg_seedling_install = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_seedling_install->candlc = 1;
        msg_seedling_install->candata[0] = 0;
        msg_seedling_install->canid = canid.seedling_install;
        _pub_canusb->publish(*msg_seedling_install);
    }
}

void Gamebtn::seedling_install_1(bool is_seedlinghand_convergence, rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    if(is_seedlinghand_convergence){
        cout<<"seedling_install_1"<<endl; 
        auto msg_seedling_install = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_seedling_install->candlc = 1;
        msg_seedling_install->candata[0] = 1;
        msg_seedling_install->canid = canid.seedling_install;
        _pub_canusb->publish(*msg_seedling_install);
    }
}

void Gamebtn::seedling_install_2(bool is_seedlinghand_convergence, rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    if(is_seedlinghand_convergence){
        cout<<"seedling_install_2"<<endl; 
        auto msg_seedling_install = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_seedling_install->candlc = 1;
        msg_seedling_install->candata[0] = 2;
        msg_seedling_install->canid = canid.seedling_install;
        _pub_canusb->publish(*msg_seedling_install);
    }
}

void Gamebtn::seedling_install_3(bool is_seedlinghand_convergence, rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    if(is_seedlinghand_convergence){
        cout<<"seedling_install_3"<<endl;     
        auto msg_seedling_install = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_seedling_install->candlc = 1;
        msg_seedling_install->candata[0] = 3;
        msg_seedling_install->canid = canid.seedling_install;
        _pub_canusb->publish(*msg_seedling_install);
    }
}

void Gamebtn::initial_sequense(std::string initial_pickup_state,rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub_initial_sequense){
    auto initial_sequense_pickup = std::make_shared<std_msgs::msg::String>();
    initial_sequense_pickup->data = initial_pickup_state;
    _pub_initial_sequense->publish(*initial_sequense_pickup);
}