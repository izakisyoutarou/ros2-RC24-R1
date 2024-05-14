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

    _subscription_base_control = this->create_subscription<controller_interface_msg::msg::BaseControl>(
        "base_control",
        _qos,
        std::bind(&Sequencer::callback_base_control, this, std::placeholders::_1)
    );

    _subscription_target_node = this->create_subscription<std_msgs::msg::String>(
        "target_node",
        _qos,
        std::bind(&Sequencer::callback_target_node, this, std::placeholders::_1)
    );

    _publisher_move_node = this->create_publisher<std_msgs::msg::String>("move_node", _qos);
    _publisher_canusb = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);

}

void Sequencer::callback_convergence(const controller_interface_msg::msg::Convergence::SharedPtr msg){
    int n  = 0;
    if(sequence_mode == SEQUENCE_MODE::planting){
        // if(progress == n++){
        //     command_move_node('A');
        //     progress++;
        // } 
        // else if(progress == n++ && tof[0] && msg->seedlinghand){
        //     command_move_node('B');
        //     progress++;
        // }
        // else if(progress == n++ && tof[1] && && msg->seedlinghand){
        //     command_move_node('C');
        //     progress++;
        // }
        // else if(progress == n++ && way_point = 'C'){
        //     command_seedling_install_left_0();
        //     progress++;
        // }
        // else if(progress == n++ && && msg->seedlinghand){
        //     command_move_node('D');
        //     progress++;       
        // }
        // else if(progress == n++ && way_point = 'D'){
        //     command_seedling_install_left_1();
        //     progress++;
        // }
        // else if(progress == n++ && && msg->seedlinghand){
        //     command_move_node('E');
        //     progress++;       
        // }
        // else if(progress == n++ && way_point = 'E'){
        //     command_seedling_install_right_0();
        //     progress++;
        // }
        // else if(progress == n++ && && msg->seedlinghand){
        //     command_move_node('F');
        //     progress++;       
        // }
        // else if(progress == n++ && way_point = 'F'){
        //     command_seedling_install_right_1();
        //     progress++;
        // }
        // else if(progress == n++ && && msg->seedlinghand){
        //     command_move_node('G');
        //     progress++;       
        // }
        // else if(progress == n++ && tof[0] && msg->seedlinghand){
        //     command_move_node('H');
        //     progress++;
        // }
        // else if(progress == n++ && way_point = 'H'){
        //     command_seedling_install_left_0();
        //     progress++;
        // }
        // else if(progress == n++ && && msg->seedlinghand){
        //     command_move_node('I');
        //     progress++;       
        // }
        // else if(progress == n++ && way_point = 'I'){
        //     command_seedling_install_left_1();
        //     progress++;
        // }
        // else if(progress == n++ && && msg->seedlinghand){
        //     command_move_node('IJ');
        //     command_sequence(SEQUENCE_MODE::stop);       
        // }
    }
    else if(sequence_mode == SEQUENCE_MODE::harvesting){
        // if(progress == n++){             
        //     command_move_node(std::to_string(std::stoi(msg->data.substr(1)) % 6 + 3));
        //     progress++;
        // } 
        // else if(progress == n++ && way_point[0] == 'b){
        //     command_move_node(target_data);  
        //     command_sequence(SEQUENCE_MODE::stop);    
        // } 
    }
}

void Sequencer::callback_base_control(const controller_interface_msg::msg::BaseControl::SharedPtr msg){
    if(msg->is_restart){
        sequence_mode = SEQUENCE_MODE::stop;
    }
}

void Sequencer::callback_target_node(const std_msgs::msg::String::SharedPtr msg){
    if(sequence_mode == SEQUENCE_MODE::stop){
        if(msg->data[0] == 'H') command_sequence(SEQUENCE_MODE::harvesting);
        target_node = msg->data;
    }
};

void Sequencer::command_sequence(const SEQUENCE_MODE sequence){
    sequence_mode = sequence;
    progress = 0;
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