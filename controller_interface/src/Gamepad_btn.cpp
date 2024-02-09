#include "controller_interface/Gamepad_btn.hpp"
#include <rclcpp/rclcpp.hpp>

Gamepadbtn::Gamepadbtn(){}

std_msgs::msg::String Gamepadbtn::main_screen_btn(const int16_t can_inject_spinning_id,const std_msgs::msg::String::SharedPtr msg, std_msgs::msg::String msg_move_node){
    auto msg_inject_spinning_screen = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_inject_spinning_screen->canid = can_inject_spinning_id;
    msg_inject_spinning_screen->candlc = 1;

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

// void main_physics_btn(Variable variable){
//     //リスタートの変数宣言
//     //msg_restartにリスタートする際のcanidとcandlcのパラメータを格納
//     auto msg_restart = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
//     msg_restart->canid = variable.can_restart_id;
//     msg_restart->candlc = 0;

//     //緊急停止の変数宣言
//     //msg_emergencyにリスタートする際のcanidとcandlcのパラメータを格納
//     auto msg_emergency = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
//     msg_emergency->canid = variable.can_emergency_id;
//     msg_emergency->candlc = 1;

//     //ボタンの変数宣言
//     //msg_btnにリスタートする際のcanidとcandlcのパラメータを格納
//     auto msg_btn = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
//     msg_btn->canid = variable.can_main_button_id;
//     msg_btn->candlc = 8;

//     uint8_t _candata_btn[8];
//     bool flag_restart = false;

//     //緊急停止
//     if(variable.msg->data == "g"){
//         variable.robotcontrol_flag = true;
//         variable.is_emergency = true;
//     }

//     //sはリスタート。緊急と手自動のboolをfalseにしてリセットしている。
//     //msgがsだったときのみ以下の変数にパラメータが代入される
//     if(variable.msg->data == "s")
//     {
//         variable.robotcontrol_flag = true;
//         flag_restart = true;
//         variable.is_emergency = false;
//         variable.is_injection_mech_stop_m = true;
//         variable.is_move_autonomous = variable.defalt_move_autonomous_flag;
//         variable.is_injection_autonomous = variable.defalt_injection_autonomous_flag;
//         variable.is_slow_speed = variable.defalt_slow_speed_flag;
//         variable.initial_state = "O";

//         variable.is_spline_convergence = variable.defalt_spline_convergence;
//         variable.is_injection_calculator_convergence = variable.defalt_injection_calculator_convergence;
//         variable.is_injection_convergence = variable.defalt_injection_convergence;
//         variable.is_seedlinghand_convergence = variable.defalt_seedlinghand_convergence;
//         variable.is_ballhand_convergence = variable.defalt_ballhand_convergence;                
//     }

//     //射出
//     if(variable.msg->data == "r1"){
//         if(variable.is_injection_convergence && !variable.is_injection_mech_stop_m){
//             auto msg_inject = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
//             msg_inject->canid = variable.can_inject_id;
//             msg_inject->candlc = 0;
//             variable._pub_canusb->publish(*msg_inject);
//         }
//     }

//     //回転停止
//     if(variable.msg->data == "r2"){
//         auto msg_inject_spinning = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
//         msg_inject_spinning->canid = variable.can_inject_spinning_id;
//         msg_inject_spinning->candlc = 1;
//         msg_inject_spinning->candata[0] = false;
//         variable._pub_canusb->publish(*msg_inject_spinning);
//         variable.is_injection_mech_stop_m = true;
//         variable.is_move_autonomous = false;
//     }

//     //射出パラメータ&回転開始
//     if(variable.msg->data == "l1"){
//         if(variable.is_backside){
//             auto msg_injection = std::make_shared<std_msgs::msg::Bool>();
//             msg_injection->data = true;
//             variable._pub_injection->publish(*msg_injection);
//             auto msg_inject_spinning = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
//             msg_inject_spinning->canid = variable.can_inject_spinning_id;
//             msg_inject_spinning->candlc = 1;
//             msg_inject_spinning->candata[0] = true;
//             variable._pub_canusb->publish(*msg_inject_spinning);
//             variable.is_backside = true;
//         }
//         else {
//             auto msg_injection = std::make_shared<std_msgs::msg::Bool>();
//             msg_injection->data = true;
//             variable._pub_injection->publish(*msg_injection);
//             auto msg_inject_spinning = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
//             msg_inject_spinning->canid = variable.can_inject_spinning_id;
//             msg_inject_spinning->candlc = 1;
//             msg_inject_spinning->candata[0] = false;
//             variable._pub_canusb->publish(*msg_inject_spinning);
//             variable.is_backside = false;
//         }
//         variable.is_move_autonomous = true;
//         variable.is_injection_mech_stop_m = false;
//     }

//     //高速低速モードの切り替え
//     if(variable.msg->data == "l2"){
//         if(variable.is_slow_speed) variable.is_slow_speed = false;
//         else variable.is_slow_speed = true;
//     }

//     //ステアリセット
//     if(variable.msg->data == "up"){
//         auto msg_steer_reset = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
//         msg_steer_reset->canid = variable.can_steer_reset_id;
//         msg_steer_reset->candlc = 0;
//         variable._pub_canusb->publish(*msg_steer_reset);
//     }

//     //キャリブレーション
//     if(variable.msg->data == "down"){
//         auto msg_calibrate = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
//         msg_calibrate->canid = variable.can_calibrate_id;
//         msg_calibrate->candlc = 0;
//         variable._pub_canusb->publish(*msg_calibrate);
//     }

//     //IO基盤リセット
//     if(variable.msg->data == "left"){
//         auto msg_main_reset = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
//         msg_main_reset->canid = variable.can_reset_id;
//         msg_main_reset->candlc = 1;
//         msg_main_reset->candata[0] = 0;
//         variable._pub_canusb->publish(*msg_main_reset);
//     }

//     //mian基盤リセット
//     if(variable.msg->data == "right"){
//         auto msg_io_reset = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
//         msg_io_reset->canid = variable.can_reset_id;
//         msg_io_reset->candlc = 1;
//         msg_io_reset->candata[0] = 1;
//         variable._pub_canusb->publish(*msg_io_reset);
//     }

//     //右ハンド籾の装填
//     if(variable.msg->data == "a"){
//         if(variable.is_ballhand_convergence){
//             auto msg_paddy_install = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
//             msg_paddy_install->candlc = 1;
//             msg_paddy_install->candata[0] = true;
//             msg_paddy_install->canid = variable.can_paddy_install_id;
//             variable._pub_canusb->publish(*msg_paddy_install);
//         }
//     }

//     //左ハンド籾の装填
//     if(variable.msg->data == "b"){
//         if(variable.is_ballhand_convergence){
//             auto msg_paddy_install = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
//             msg_paddy_install->candlc = 1;
//             msg_paddy_install->candata[0] = false;
//             msg_paddy_install->canid = variable.can_paddy_install_id;
//             variable._pub_canusb->publish(*msg_paddy_install);
//         }
//     }

//     //右ハンド籾の回収
//     if(variable.msg->data == "x"){
//         if(variable.is_ballhand_convergence){
//             auto msg_paddy_collect = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
//             msg_paddy_collect->candlc = 1;
//             msg_paddy_collect->candata[0] = true;
//             msg_paddy_collect->canid = variable.can_paddy_collect_id;
//             variable._pub_canusb->publish(*msg_paddy_collect);
//         }
//     }
    
//     //左ハンド籾の回収
//     if(variable.msg->data == "y"){
//         if(variable.is_ballhand_convergence){
//             auto msg_paddy_collect = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
//             msg_paddy_collect->candlc = 1;
//             msg_paddy_collect->candata[0] = false;
//             msg_paddy_collect->canid = variable.can_paddy_collect_id;
//             variable._pub_canusb->publish(*msg_paddy_collect);
//         }
//     }

//     //r3は足回りの手自動の切り替え。is_move_autonomousを使って、トグルになるようにしてる。R1の上物からもらう必要はない。
//     if(variable.msg->data == "r3")
//     {
//         variable.robotcontrol_flag = true;
//         if(variable.is_move_autonomous == false) variable.is_move_autonomous = true;
//         else variable.is_move_autonomous = false;
//     }

//     //l3を押すと射出情報をpublishする
//     if(variable.msg->data == "l3")
//     {
//         auto initial_sequense_pickup = std::make_shared<std_msgs::msg::String>();
//         initial_sequense_pickup->data = variable.initial_pickup_state;
//         variable._pub_initial_sequense->publish(*initial_sequense_pickup);
//     }

//     //リセットボタンを押しているか確認する
//     variable.is_reset = variable.msg->data == "s";

//     //base_controlへ代入
//     variable.msg_base_control.is_restart = variable.is_reset;
//     variable.msg_base_control.is_emergency = variable.is_emergency;
//     variable.msg_base_control.is_move_autonomous = variable.is_move_autonomous;
//     variable.msg_base_control.is_injection_autonomous = variable.is_injection_autonomous;
//     variable.msg_base_control.is_slow_speed = variable.is_slow_speed;
//     variable.msg_base_control.initial_state = variable.initial_state;
//     variable.msg_base_control.is_injection_mech_stop_m = variable.is_injection_mech_stop_m;

//     msg_emergency->candata[0] = variable.is_emergency;

//     if(variable.msg->data=="g"){
//         variable._pub_canusb->publish(*msg_emergency);
//     }

//     if(variable.robotcontrol_flag == true)
//     {
//         variable._pub_base_control->publish(variable.msg_base_control);

//         variable.msg_unity_control.data = variable.is_reset;
//         variable._pub_base_restart->publish(variable.msg_unity_control);

//         variable.msg_unity_control.data = variable.is_emergency;
//         variable._pub_base_emergency->publish(variable.msg_unity_control);

//         variable.msg_unity_control.data = variable.is_move_autonomous;
//         variable._pub_move_auto->publish(variable.msg_unity_control);

//         variable.msg_unity_control.data = variable.is_injection_autonomous;
//         variable._pub_base_injection->publish(variable.msg_unity_control);
//     }

//     if(variable.msg->data == "s"){
//         variable._pub_canusb->publish(*msg_restart);
//         variable._pub_canusb->publish(*msg_emergency);
//     }

//     if(flag_restart == true){
//         variable.msg_base_control.is_restart = false;
//         variable._pub_base_control->publish(variable.msg_base_control);
//     }
// }

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

// void sub_physics_btn(Sub_variable sub_variable){
//     if(sub_variable.msg->data == "a")
//             {
//                 if(sub_variable.is_seedlinghand_convergence){
//                     auto msg_seedling_collect = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
//                     msg_seedling_collect->candlc = 1;
//                     msg_seedling_collect->candata[0] = 0;
//                     msg_seedling_collect->canid = sub_variable.can_seedling_collect_id;
//                     sub_variable._pub_canusb->publish(*msg_seedling_collect);
//                 }
//             }

//             if(sub_variable.msg->data == "b")
//             {
//                 if(sub_variable.is_seedlinghand_convergence){
//                     auto msg_seedling_collect = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
//                     msg_seedling_collect->candlc = 1;
//                     msg_seedling_collect->candata[0] = 1;
//                     msg_seedling_collect->canid = sub_variable.can_seedling_collect_id;
//                     sub_variable._pub_canusb->publish(*msg_seedling_collect);
//                 }
//             }

//             if(sub_variable.msg->data == "x")
//             {

//             }

//             if(sub_variable.msg->data == "y")
//             {

//             }

//             if(sub_variable.msg->data == "up")
//             {
//                 if(sub_variable.is_seedlinghand_convergence){
//                     auto msg_seedling_install = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
//                     msg_seedling_install->candlc = 1;
//                     msg_seedling_install->candata[0] = 0;
//                     msg_seedling_install->canid = sub_variable.can_seedling_install_id;
//                     sub_variable._pub_canusb->publish(*msg_seedling_install);
//                 }
//             }

//              if(sub_variable.msg->data == "right")
//             {
//                 if(sub_variable.is_seedlinghand_convergence){
//                     auto msg_seedling_install = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
//                     msg_seedling_install->candlc = 1;
//                     msg_seedling_install->candata[0] = 1;
//                     msg_seedling_install->canid = sub_variable.can_seedling_install_id;
//                     sub_variable._pub_canusb->publish(*msg_seedling_install);
//                 }
//             }

//             if(sub_variable.msg->data == "down")
//             {
//                 if(sub_variable.is_seedlinghand_convergence){
//                     auto msg_seedling_install = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
//                     msg_seedling_install->candlc = 1;
//                     msg_seedling_install->candata[0] = 2;
//                     msg_seedling_install->canid = sub_variable.can_seedling_install_id;
//                     sub_variable._pub_canusb->publish(*msg_seedling_install);
//                 }
//             }

//             if(sub_variable.msg->data == "left")
//             {
//                 if(sub_variable.is_seedlinghand_convergence){
//                     auto msg_seedling_install = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
//                     msg_seedling_install->candlc = 1;
//                     msg_seedling_install->candata[0] = 3;
//                     msg_seedling_install->canid = sub_variable.can_seedling_install_id;
//                     sub_variable._pub_canusb->publish(*msg_seedling_install);
//                 }
//             }
//}