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
    canid_paddy_install(get_parameter("canid.paddy_install").as_int()),
    canid_arm_expansion(get_parameter("canid.arm_expansion").as_int()),
    canid_arm_up(get_parameter("canid.arm_up").as_int()),
    court_color(get_parameter("court_color").as_string())
    
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

    _subscription_is_start = this->create_subscription<std_msgs::msg::UInt8>(
        "is_start",
        _qos,
        std::bind(&Sequencer::callback_is_start, this, std::placeholders::_1)
    );

    _subscription_way_point = this->create_subscription<std_msgs::msg::String>(
        "way_point",
        _qos,
        std::bind(&Sequencer::callback_way_point, this, std::placeholders::_1)
    );

    _subscription_self_pose = this->create_subscription<geometry_msgs::msg::Vector3>(
        "self_pose",
        rclcpp::SensorDataQoS(),
        std::bind(&Sequencer::_subscriber_callback_self_pose, this, std::placeholders::_1)
    );

    _publisher_move_node = this->create_publisher<std_msgs::msg::String>("move_node", _qos);
    _publisher_canusb = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);
    _publisher_move_autonomous = this->create_publisher<std_msgs::msg::Bool>("move_autonomous", _qos);
    _publisher_move_interrupt_node = this->create_publisher<std_msgs::msg::String>("move_interrupt_node", _qos);

}

void Sequencer::callback_convergence(const controller_interface_msg::msg::Convergence::SharedPtr msg){
    int n  = 0;
    if(sequence_mode == SEQUENCE_MODE::planting){
        if(sequence_count == 1){
            if(progress == n++){
                command_move_autonomous(true);
                command_arm_expansion();
                progress++;
            }
            else if(progress == n++ && move_autonomous){
                command_move_node("a0");
                progress++;
            }
            else if(progress == n++ && way_point == "a0"){
                command_move_node("A");
                progress++;
            }
            else if(progress == n++ && way_point == "A"){
                command_move_autonomous(false);
                command_sequence(SEQUENCE_MODE::stop);   
            }
        }
        else if(sequence_count == 2){
            if(progress == n++){
                if(court_color == "blue") command_seedling_collect_left();
                else if(court_color == "red") command_seedling_collect_right();
                progress++;
            } 
            else if( progress == n++ && msg->seedlinghand){
                timer(800);
                progress++;
            }
            else if(progress == n++ && timer()){
                command_move_autonomous(true);
                progress++;
            }
            else if(progress == n++ && move_autonomous){
                command_move_node("a1");
                progress++;
            }
            else if(progress == n++ && way_point == "a1"){
                command_move_node("a2");
                progress++;
            }
            else if(progress == n++ && way_point == "a2"){
                command_move_node("B");
                progress++;
            }
            else if(progress == n++ && way_point == "B"){
                command_move_autonomous(false);
                command_sequence(SEQUENCE_MODE::stop);   
            }
        }
        else if(sequence_count == 3){
            if(progress == n++){
                if(court_color == "blue") command_seedling_collect_right();
                else if(court_color == "red") command_seedling_collect_left();
                progress++;
            } 
            else if(progress == n++ && msg->seedlinghand){
                timer(800);
                progress++;
            }
            else if(progress == n++ && timer()){
                command_move_autonomous(true);
                progress++;
            }
            else if(progress == n++ && move_autonomous){
                command_move_node("C");
                progress++;
            }
            else if(progress == n++ && !msg->spline_convergence){
                if(court_color == "blue") command_seedling_install_right_0();
                else if(court_color == "red") command_seedling_install_left_0();
                progress++;
            }
            else if(progress == n++ && msg->seedlinghand){
                timer(300);
                progress++;
            }
            else if(progress == n++ && timer()){
                command_move_node("D");
                progress++;
            }
            else if(progress == n++ && !msg->spline_convergence){
                if(court_color == "blue") command_seedling_install_right_1();
                else if(court_color == "red") command_seedling_install_left_1();
                progress++;
            }
            else if( progress == n++ && msg->seedlinghand){
                timer(300);
                progress++;
            }
            else if(progress == n++ && timer()){
                command_move_node("E");
                progress++;
            }
            else if(progress == n++ && !msg->spline_convergence){
                if(court_color == "blue") command_seedling_install_left_0();
                else if(court_color == "red") command_seedling_install_right_0();
                progress++;
            }
            else if( progress == n++ && msg->seedlinghand){
                timer(300);
                progress++;
            }
            else if(progress == n++ && timer()){
                command_move_node("F");
                progress++;
            }
            else if(progress == n++ && !msg->spline_convergence){
                if(court_color == "blue") command_seedling_install_left_1();
                else if(court_color == "red") command_seedling_install_right_1();
                progress++;
            }
            else if(progress == n++ && msg->seedlinghand){
                timer(300);
                progress++;
            }
            else if(progress == n++ && timer()){
                command_move_node("G");
                progress++;     
            }
            else if(progress == n++ && way_point == "G"){
                command_move_autonomous(false);
                command_sequence(SEQUENCE_MODE::stop);       
            }
        }
        else if(sequence_count == 4){
            if(progress == n++){
                if(court_color == "blue") command_seedling_collect_left();
                else if(court_color == "red") command_seedling_collect_right();
                progress++;
            }
            else if(progress == n++ && msg->seedlinghand){
                timer(800);
                progress++;
            }
            else if(progress == n++ && timer()){
                command_move_autonomous(true);
                progress++;
            }
            else if(progress == n++ && move_autonomous){
                command_move_node("H");
                progress++;
            }
            else if(progress == n++ && !msg->spline_convergence){
                if(court_color == "blue") command_seedling_install_right_0();
                else if(court_color == "red") command_seedling_install_left_0();
                progress++;
            }
            else if(progress == n++ && msg->seedlinghand){
                timer(300);
                progress++;
            }
            else if(progress == n++ && timer()){
                command_move_node("I");
                progress++;       
            }
            else if(progress == n++ && !msg->spline_convergence){
                if(court_color == "blue") command_seedling_install_right_1();
                else if(court_color == "red") command_seedling_install_left_1();
                progress++;
            }
            else if(progress == n++ && msg->seedlinghand){
                timer(300);
                progress++;
            }   
            else if(progress == n++ && timer()){
                command_move_node("IJ0");
                command_sequence(SEQUENCE_MODE::stop);       
            }
        }
    }
    else if(sequence_mode == SEQUENCE_MODE::harvesting){
        if(progress == n++){
            if(target_node == "H0" || target_node == "H6") way = "b3";  
            else if(target_node == "H1" || target_node == "H7") way = "b4";
            else if(target_node == "H2" || target_node == "H8") way = "b5";
            else if(target_node == "H3" || target_node == "H9") way = "b6";
            else if(target_node == "H4" || target_node == "H10") way = "b7";
            else if(target_node == "H5" || target_node == "H11") way = "b8";
            else if(target_node == "H5" || target_node == "H11") way = "b8";
            command_move_node(way);  
            progress++;
        } 
        else if(progress == n++ && way_point == way){
            command_move_node(target_node);  
            command_sequence(SEQUENCE_MODE::stop);    
        } 
    }
    else if(sequence_mode == SEQUENCE_MODE::comeback){
            if(progress == n++){
                command_move_autonomous(true);
                progress++;
            }
            else if(progress == n++ && move_autonomous){
                command_move_interrupt_node("a8");
                command_sequence(SEQUENCE_MODE::stop);    
            }
    }
    else if(sequence_mode == SEQUENCE_MODE::evacuate){
        // if(progress == n++){
        //     if(pre_target_node == "H0" || pre_target_node == "H6") way = "b3";  
        //     else if(pre_target_node == "H1" || pre_target_node == "H7") way = "b4";
        //     else if(pre_target_node == "H2" || pre_target_node == "H8") way = "b5";
        //     else if(pre_target_node == "H3" || pre_target_node == "H9") way = "b6";
        //     else if(pre_target_node == "H4" || pre_target_node == "H10") way = "b7";
        //     else if(pre_target_node == "H5" || pre_target_node == "H11") way = "b8";
        //     else if(pre_target_node == "H12") way = "IJ0";
        //     else if(pre_target_node == "IJ0"){
        //         command_move_node("EV0");
        //         command_sequence(SEQUENCE_MODE::stop);    
        //     }
        //     command_move_node(way);  
        //     progress++;
        // } 
        // else if(progress == n++ && way_point == way){
        //     command_move_node("EV0");  
        //     command_sequence(SEQUENCE_MODE::stop);    
        // } 
    }
}

void Sequencer::callback_base_control(const controller_interface_msg::msg::BaseControl::SharedPtr msg){
    if(msg->is_restart){
        sequence_mode = SEQUENCE_MODE::stop;
        sequence_count = 0;
    }
    move_autonomous = msg->is_move_autonomous;
}

void Sequencer::callback_target_node(const std_msgs::msg::String::SharedPtr msg){
    pre_target_node = target_node;
    target_node = msg->data;
    if(msg->data[0] == 'H') {
        if((pre_target_node == "H0" && target_node == "H6") || (pre_target_node == "H1" && target_node == "H7") || (pre_target_node == "H2" && target_node == "H8") || (pre_target_node == "H3" && target_node == "H9") || (pre_target_node == "H4" && target_node == "H10") || (pre_target_node == "H5" && target_node == "H11") || (target_node == "H12")){
            command_move_node(target_node); 
        }
        else command_sequence(SEQUENCE_MODE::harvesting);
        command_arm_up();
    }
    else if(msg->data[0] == 'I') {
        command_arm_up();
        command_move_node(target_node);  
    }
    else if(msg->data == "a8") command_sequence(SEQUENCE_MODE::comeback);
    else command_move_node(target_node);  
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

void Sequencer::command_move_interrupt_node(const std::string node){
    auto msg_move_interrupt_node = std::make_shared<std_msgs::msg::String>();
    msg_move_interrupt_node->data = node;
    _publisher_move_interrupt_node->publish(*msg_move_interrupt_node);
}

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

void Sequencer::command_move_autonomous(bool flag){
    auto msg_move_autonomous = std::make_shared<std_msgs::msg::Bool>();
    msg_move_autonomous->data = flag;
    _publisher_move_autonomous->publish(*msg_move_autonomous);
}

void Sequencer::callback_is_start(const std_msgs::msg::UInt8::SharedPtr msg){
    command_sequence(SEQUENCE_MODE::planting);
    sequence_count++;
}

void Sequencer::callback_way_point(const std_msgs::msg::String::SharedPtr msg){
    way_point = msg->data;
}

void Sequencer::_subscriber_callback_self_pose(const geometry_msgs::msg::Vector3::SharedPtr msg){
    this->self_pose.x = msg->x;
    this->self_pose.y = msg->y;
    this->self_pose.z = msg->z;
    if((self_pose.x > 4.0 || abs(self_pose.y) < 0.85) && !arm_up_flag) {
        command_arm_up();
        arm_up_flag = true;
    }
    else if((self_pose.x < 4.0 &&  abs(self_pose.y) > 2.00) && arm_up_flag) {
        command_arm_expansion();
        arm_up_flag = false;
    }
}

void Sequencer::timer(int ms){
    time = std::chrono::system_clock::now();
    check_time = ms;
}

bool Sequencer::timer(){
    if(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - time).count() > check_time) return true;
    else return false;
}

void Sequencer::command_seedling_collect_right(){ command_canusb_uint8(canid_seedling_collect, 1); }
void Sequencer::command_seedling_collect_left(){ command_canusb_uint8(canid_seedling_collect, 0); }
void Sequencer::command_seedling_install_right_0(){ command_canusb_uint8(canid_seedling_install, 0); }
void Sequencer::command_seedling_install_right_1(){ command_canusb_uint8(canid_seedling_install, 1); }
void Sequencer::command_seedling_install_left_0(){ command_canusb_uint8(canid_seedling_install, 3); }
void Sequencer::command_seedling_install_left_1(){ command_canusb_uint8(canid_seedling_install, 2); }
void Sequencer::command_paddy_collect(){ command_canusb_empty(canid_paddy_collect); } 
void Sequencer::command_paddy_install(){ command_canusb_empty(canid_paddy_install); }
void Sequencer::command_arm_expansion(){ command_canusb_empty(canid_arm_expansion); }
void Sequencer::command_arm_up(){ command_canusb_empty(canid_arm_up); }

}  // namespace sequencer