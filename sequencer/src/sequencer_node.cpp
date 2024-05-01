#include "sequencer/sequencer_node.hpp"
#include <queue>

namespace sequencer{

Sequencer::Sequencer(const rclcpp::NodeOptions &options) : Sequencer("", options) {}

Sequencer::Sequencer(const std::string &name_space, const rclcpp::NodeOptions &options)
: rclcpp::Node("sequencer_node", name_space, options),
    canid_inject(get_parameter("canid.inject").as_int()),
    canid_seedling_collect(get_parameter("canid.seedling_collect").as_int()),
    canid_seedling_install(get_parameter("canid.seedling_install").as_int()),
    canid_paddy_collect(get_parameter("canid.paddy_collect").as_int()),
    canid_paddy_install(get_parameter("canid.paddy_install").as_int())
    
{

    _subscription_convergence = this->create_subscription<controller_interface_msg::msg::Convergence>(
        "convergence",
        _qos,
        std::bind(&Sequencer::callback_convergence, this, std::placeholders::_1)
    );

    // _subscription_color_information = this->create_subscription<controller_interface_msg::msg::Colorball>(
    //     "color_information",
    //     _qos,
    //     std::bind(&Sequencer::callback_color_information, this, std::placeholders::_1)
    // );

    _subscription_base_control = this->create_subscription<controller_interface_msg::msg::BaseControl>(
        "base_control",
        _qos,
        std::bind(&Sequencer::callback_base_control, this, std::placeholders::_1)
    );

    _publisher_move_node = this->create_publisher<std_msgs::msg::String>("move_node", _qos);
    _publisher_canusb = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);

}

void Sequencer::callback_convergence(const controller_interface_msg::msg::Convergence::SharedPtr msg){
    int n  = 0;
    if(sequence_mode == SEQUENCE_MODE::seedling){

    }
    else if(sequence_mode == SEQUENCE_MODE::planting){

    }
    else if(sequence_mode == SEQUENCE_MODE::harvesting){

    }
    else if(sequence_mode == SEQUENCE_MODE::injection){
        
    }
}

// void Sequencer::callback_color_information(const controller_interface_msg::msg::Colorball::SharedPtr msg){
//     if(select_algorithm == 0){
//         int count = 0;
//         bool color = false;
//         std::queue<int> own_color;
//         std::queue<int> purple_color;
//         for(int i = 6; i < 12; i++){
//             if(msg->color_info[i]) own_color.push(i);
//             else purple_color.push(i);
//         }
//         for(int i = 0; i < 6; i++){
//             color = msg->color_info[i];
//             harvesting_order[count] = "H" + std::to_string(i);
//             color_order[count] = msg->color_info[i];
//             count++;
//             if(color) {
//                 harvesting_order[count] = "H" + std::to_string(purple_color.front());
//                 color_order[count] = msg->color_info[purple_color.front()];
//                 purple_color.pop();
//             }
//             else {
//                 harvesting_order[count] = "H" + std::to_string(own_color.front());
//                 color_order[count] = msg->color_info[own_color.front()];
//                 own_color.pop();        
//             }
//             count++;
//         }
//         RCLCPP_INFO(this->get_logger(), "籾の回収順");
//         for(int i = 0; i < 12; i++) RCLCPP_INFO(this->get_logger(), "%s,%d", harvesting_order[i].c_str(),color_order[i]);
//     }
// }

void Sequencer::callback_base_control(const controller_interface_msg::msg::BaseControl::SharedPtr msg){
    if(msg->is_restart){
        sequence_mode = SEQUENCE_MODE::stop;
    }
}

void Sequencer::command_move_node(const std::string node){
    auto msg_move_node = std::make_shared<std_msgs::msg::String>();
    msg_move_node->data = node;
    _publisher_move_node->publish(*msg_move_node);
};

void Sequencer::command_canusb_empty(const int16_t id){
    auto msg_canusb = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_canusb->canid = id;
    msg_canusb->candlc = 0;
    _publisher_canusb->publish(*msg_canusb);  
};

void Sequencer::command_canusb_uint8(const int16_t id, const uint8_t data){
    auto msg_canusb = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_canusb->canid = id;
    msg_canusb->candlc = 1;
    msg_canusb->candata[0] = data;
    _publisher_canusb->publish(*msg_canusb);
};


void Sequencer::command_seedling_collect_right(){ command_canusb_uint8(canid_seedling_collect, 0); }
void Sequencer::command_seedling_collect_left(){ command_canusb_uint8(canid_seedling_collect, 1); }
void Sequencer::command_seedling_install_right_0(){ command_canusb_uint8(canid_seedling_install, 0); }
void Sequencer::command_seedling_install_right_1(){ command_canusb_uint8(canid_seedling_install, 1); }
void Sequencer::command_seedling_install_left_0(){ command_canusb_uint8(canid_seedling_install, 2); }
void Sequencer::command_seedling_install_left_1(){ command_canusb_uint8(canid_seedling_install, 3); }
void Sequencer::command_paddy_collect(){ command_canusb_empty(canid_paddy_collect); } 
void Sequencer::command_paddy_install(){ command_canusb_empty(canid_paddy_install); }

}  // namespace sequencer