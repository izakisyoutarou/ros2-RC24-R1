#include "controller_interface/Gamepad_btn.hpp"
#include <rclcpp/rclcpp.hpp>

Gamepadbtn::Gamepadbtn(){}

std_msgs::msg::String Gamepadbtn::main_screen_btn(const int16_t can_inject_spinning_id,const std_msgs::msg::String::SharedPtr msg, std_msgs::msg::String msg_move_node){

    if(msg->data == "O") msg_move_node.data = "O";
    else if(msg->data == "S0") msg_move_node.data = "S0";
    else if(msg->data == "S1") msg_move_node.data = "S1";
    else if(msg->data == "S2") msg_move_node.data = "S2";
    else if(msg->data == "S3") msg_move_node.data = "S3";
    else if(msg->data == "P0") msg_move_node.data = "P0";
    else if(msg->data == "P1") msg_move_node.data = "P1";
    else if(msg->data == "P2") msg_move_node.data = "P2";
    else if(msg->data == "P3") msg_move_node.data = "P3";
    else if(msg->data == "P4") msg_move_node.data = "P4";
    else if(msg->data == "P5") msg_move_node.data = "P5";
    else if(msg->data == "P6") msg_move_node.data = "P6";
    else if(msg->data == "P7") msg_move_node.data = "P7";
    else if(msg->data == "H0") msg_move_node.data = "H0";
    else if(msg->data == "H1") msg_move_node.data = "H1";
    else if(msg->data == "H2") msg_move_node.data = "H2";
    else if(msg->data == "H3") msg_move_node.data = "H3";
    else if(msg->data == "H4") msg_move_node.data = "H4";
    else if(msg->data == "H5") msg_move_node.data = "H5";
    else if(msg->data == "H6") msg_move_node.data = "H6";
    else if(msg->data == "H7") msg_move_node.data = "H7";
    else if(msg->data == "H8") msg_move_node.data = "H8";
    else if(msg->data == "H9") msg_move_node.data = "H9";
    else if(msg->data == "H10") msg_move_node.data = "H10";
    else if(msg->data == "H11") msg_move_node.data = "H11";
    else if(msg->data == "IJ") msg_move_node.data = "IJ";

    return msg_move_node;

}
controller_interface_msg::msg::Colorball Gamepadbtn::sub_screen_btn(const std_msgs::msg::String::SharedPtr msg, controller_interface_msg::msg::Colorball msg_colorball_info){

    bool color_data[12];

    if(msg->data == "A_red"){
        color_data[0] = true;
        msg_colorball_info.color_info[0] = color_data[0];
    }
    if(msg->data == "A_purple"){
        color_data[0] = false;
        msg_colorball_info.color_info[0] = color_data[0];
    }
    if(msg->data == "B_red"){
        color_data[1] = true;
        msg_colorball_info.color_info[1] = color_data[1];                
    }
    if(msg->data == "B_purple"){
        color_data[1] = false;
        msg_colorball_info.color_info[1] = color_data[1];
    }
    if(msg->data == "C_red"){
        color_data[2] = true;
        msg_colorball_info.color_info[2] = color_data[2];
    }
    if(msg->data == "C_purple"){
        color_data[2] = false;
        msg_colorball_info.color_info[2] = color_data[2];
    }
    if(msg->data == "D_red"){
        color_data[3] = true;
        msg_colorball_info.color_info[3] = color_data[3];
    }
    if(msg->data == "D_purple"){
        color_data[3] = false;
        msg_colorball_info.color_info[3] = color_data[3];
    }
    if(msg->data == "E_red"){
        color_data[4] = true;
        msg_colorball_info.color_info[4] = color_data[4];
    }
    if(msg->data == "E_purple"){
        color_data[4] = false;
        msg_colorball_info.color_info[4] = color_data[4];
    }
    if(msg->data == "F_red"){
        color_data[5] = true;
        msg_colorball_info.color_info[5] = color_data[5];
    }
    if(msg->data == "F_purple"){
        color_data[5] = false;
        msg_colorball_info.color_info[5] = color_data[5];
    }
    if(msg->data == "G_red"){
        color_data[6] = true;
        msg_colorball_info.color_info[6] = color_data[6];
    }
    if(msg->data == "G_purple"){
        color_data[6] = false;
        msg_colorball_info.color_info[6] = color_data[6];
    }
    if(msg->data == "H_red"){
        color_data[7] = true;
        msg_colorball_info.color_info[7] = color_data[7];
    }
    if(msg->data == "H_purple"){
        color_data[7] = false;
        msg_colorball_info.color_info[7] = color_data[7];
    }
    if(msg->data == "I_red"){
        color_data[8] = true;
        msg_colorball_info.color_info[8] = color_data[8];
    }
    if(msg->data == "I_purple"){
        color_data[8] = false;
        msg_colorball_info.color_info[8] = color_data[8];
    }
    if(msg->data == "J_red"){
        color_data[9] = true;
        msg_colorball_info.color_info[9] = color_data[9];
    }
    if(msg->data == "J_purple"){
        color_data[9] = false;
        msg_colorball_info.color_info[9] = color_data[9];
    }
    if(msg->data == "K_red"){
        color_data[10] = true;
        msg_colorball_info.color_info[10] = color_data[10];
    }
    if(msg->data == "K_purple"){
        color_data[10] = false;
        msg_colorball_info.color_info[10] = color_data[10];
    }
    if(msg->data == "L_red"){
        color_data[11] = true;
        msg_colorball_info.color_info[11] = color_data[11];
    }
    if(msg->data == "L_purple"){
        color_data[11] = false;
        msg_colorball_info.color_info[11] = color_data[11];
    }

    return msg_colorball_info;
}

bool Gamepadbtn::main_physics_btn_emergency(bool robotcontrol_flag,bool is_emergency){
    robotcontrol_flag = true;
    is_emergency = true;
    return robotcontrol_flag,is_emergency;
}

bool Gamepadbtn::main_physics_btn_r1(bool is_injection_convergence,bool is_injection_mech_stop_m,const int16_t can_inject_id){
    if(is_injection_convergence && !is_injection_mech_stop_m){
        auto msg_inject = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_inject->canid = can_inject_id;
        msg_inject->candlc = 0;
    }
}

