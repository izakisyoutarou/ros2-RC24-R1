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
    paddy_flag = true; 
    seed_right_flag = 0;
    seed_left_flag = 0;
}

void Gamebtn::board_reset(rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    cout<<"board_reset"<<endl;
    auto msg_board_reset = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_board_reset->canid = canid.reset;
    msg_board_reset->candlc = 1;
    _pub_canusb->publish(*msg_board_reset);
}

void Gamebtn::steer_reset(rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    cout<<"steer_reset"<<endl;
    auto msg_steer_reset = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_steer_reset->canid = canid.steer_reset;
    msg_steer_reset->candlc = 0;
    _pub_canusb->publish(*msg_steer_reset);
}

// void Gamebtn::injection_spining_start(std::string move_node,rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr _pub_backspin_injection,rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_is_backside,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
//     cout<<"spining_start"<<endl;    
//     auto msg_inject_spinning = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
//     msg_inject_spinning->canid = canid.inject_spinning;
//     msg_inject_spinning->candlc = 1;
//     msg_inject_spinning->candata[0] = true;
//     _pub_canusb->publish(*msg_inject_spinning);
// }

// void Gamebtn::injection_spining_stop(rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
//     cout<<"spining_stop"<<endl;
//     auto msg_inject_spinning = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
//     msg_inject_spinning->canid = canid.inject_spinning;
//     msg_inject_spinning->candlc = 1;
//     msg_inject_spinning->candata[0] = false;
//     _pub_canusb->publish(*msg_inject_spinning);
// }

// void Gamebtn::injection(bool is_injection_convergence,bool is_injection_mech_stop_m, rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
//     if(is_injection_convergence && !is_injection_mech_stop_m){
//         cout<<"injection"<<endl;        
//         auto msg_inject = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
//         msg_inject->canid = canid.inject;
//         msg_inject->candlc = 0;
//         _pub_canusb->publish(*msg_inject);
//     }
// }

void Gamebtn::injection_frontside_vel(rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_is_backside){
    cout<<"frontside_vel"<<endl;    
    auto msg_is_backside = std::make_shared<std_msgs::msg::Bool>();
    msg_is_backside->data = false;
    _pub_is_backside->publish(*msg_is_backside);
}

void Gamebtn::injection_backside_vel(rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_is_backside){
    cout<<"backside_vel"<<endl;    
    auto msg_is_backside = std::make_shared<std_msgs::msg::Bool>();
    msg_is_backside->data = true;
    _pub_is_backside->publish(*msg_is_backside);
}

void Gamebtn::injection(bool is_injection_convergence, bool injection_calculator_convergence, rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    if(is_injection_convergence && injection_calculator_convergence){
        cout<<"injection"<<endl;        
        auto msg_inject = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_inject->canid = canid.inject;
        msg_inject->candlc = 0;
        _pub_canusb->publish(*msg_inject);
    }
}

void Gamebtn::seedling_collect_right(bool is_seedlinghand_convergence, rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    if(is_seedlinghand_convergence && seed_right_flag == 0){
        cout<<"seedling_collect_right"<<endl;
        auto msg_seedling_collect = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_seedling_collect->canid = canid.seedling_collect;
        msg_seedling_collect->candlc = 1;
        msg_seedling_collect->candata[0] = 0;
        _pub_canusb->publish(*msg_seedling_collect);
        seed_right_flag = 1;
    }
}

void Gamebtn::seedling_collect_left(bool is_seedlinghand_convergence, rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    if(is_seedlinghand_convergence && seed_left_flag == 0){
        cout<<"seedling_collect_left"<<endl;
        auto msg_seedling_collect = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_seedling_collect->canid = canid.seedling_collect;
        msg_seedling_collect->candlc = 1;
        msg_seedling_collect->candata[0] = 1;
        _pub_canusb->publish(*msg_seedling_collect);
        seed_left_flag = 1;
    }
}

void Gamebtn::seedling_install_right(bool is_seedlinghand_convergence, rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    if(is_seedlinghand_convergence){
        if(seed_right_flag == 1){
            seedling_install_right0(is_seedlinghand_convergence,_pub_canusb);
            seed_right_flag = 2;
        }
        else if(seed_right_flag == 2){
            seedling_install_right1(is_seedlinghand_convergence,_pub_canusb);
            seed_right_flag = 0;
        }
    }
}

void Gamebtn::seedling_install_right0(bool is_seedlinghand_convergence, rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    if(is_seedlinghand_convergence){
        cout<<"seedling_install_right0"<<endl;    
        auto msg_seedling_install = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_seedling_install->canid = canid.seedling_install;
        msg_seedling_install->candlc = 1;
        msg_seedling_install->candata[0] = 0;
        _pub_canusb->publish(*msg_seedling_install);
    }
}

void Gamebtn::seedling_install_right1(bool is_seedlinghand_convergence, rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    if(is_seedlinghand_convergence){
        cout<<"seedling_install_right1"<<endl; 
        auto msg_seedling_install = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_seedling_install->canid = canid.seedling_install;
        msg_seedling_install->candlc = 1;
        msg_seedling_install->candata[0] = 1;
        _pub_canusb->publish(*msg_seedling_install);
    }
}

void Gamebtn::seedling_install_left(bool is_seedlinghand_convergence, rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    if(is_seedlinghand_convergence){
        if(seed_left_flag == 1){
            seedling_install_left0(is_seedlinghand_convergence,_pub_canusb);
            seed_left_flag = 2;
        }
        else if(seed_left_flag == 2){
            seedling_install_left1(is_seedlinghand_convergence,_pub_canusb);
            seed_left_flag = 0;
        }
    }
}

void Gamebtn::seedling_install_left0(bool is_seedlinghand_convergence, rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    if(is_seedlinghand_convergence){
        cout<<"seedling_install_left0"<<endl; 
        auto msg_seedling_install = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_seedling_install->canid = canid.seedling_install;
        msg_seedling_install->candlc = 1;
        msg_seedling_install->candata[0] = 2;
        _pub_canusb->publish(*msg_seedling_install);
    }
}

void Gamebtn::seedling_install_left1(bool is_seedlinghand_convergence, rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    if(is_seedlinghand_convergence){
        cout<<"seedling_install_left1"<<endl;     
        auto msg_seedling_install = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_seedling_install->canid = canid.seedling_install;
        msg_seedling_install->candlc = 1;
        msg_seedling_install->candata[0] = 3;
        _pub_canusb->publish(*msg_seedling_install);
    }
}

void Gamebtn::paddy_control(bool is_ballhand_convergence,rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    if(is_ballhand_convergence){
        if(paddy_flag) paddy_collect(_pub_canusb);
        else paddy_install(_pub_canusb);
    }
}

void Gamebtn::paddy_collect(rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
        cout<<"paddy_collect"<<endl;
        auto msg_paddy_collect = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_paddy_collect->canid = canid.paddy_collect;
        msg_paddy_collect->candlc = 0;
        _pub_canusb->publish(*msg_paddy_collect);
        paddy_flag = false;

}

void Gamebtn::paddy_install(rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
        cout<<"paddy_install"<<endl;
        auto msg_paddy_install = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_paddy_install->canid = canid.paddy_install;        
        msg_paddy_install->candlc = 0;
        _pub_canusb->publish(*msg_paddy_install);
        paddy_flag = true;
}

void Gamebtn::initial_sequense(std::string initial_pickup_state,rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub_initial_sequense){
    auto initial_sequense_pickup = std::make_shared<std_msgs::msg::String>();
    initial_sequense_pickup->data = initial_pickup_state;
    _pub_initial_sequense->publish(*initial_sequense_pickup);
}

void Gamebtn::arm_expansion(rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_canusb){
    cout<<"arm_expansion"<<endl;
    auto msg_arm_expansion = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_arm_expansion->canid = canid.arm_expansion;
    msg_arm_expansion->candlc = 1;
    _pub_canusb->publish(*msg_arm_expansion);
}