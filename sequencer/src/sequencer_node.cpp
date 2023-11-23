#include "sequencer/sequencer_node.hpp"
#include <queue>

namespace sequencer{

Sequencer::Sequencer(const rclcpp::NodeOptions &options) : Sequencer("", options) {}

Sequencer::Sequencer(const std::string &name_space, const rclcpp::NodeOptions &options)
: rclcpp::Node("sequencer_node", name_space, options),

    canid_inject_spinning(get_parameter("canid.inject_spinning").as_int()),
    canid_inject(get_parameter("canid.inject").as_int()),
    canid_seedling_collect(get_parameter("canid.seedling_collect").as_int()),
    canid_seedling_install(get_parameter("canid.seedling_install").as_int()),
    canid_paddy_collect(get_parameter("canid.paddy_collect").as_int()),
    canid_paddy_install(get_parameter("canid.paddy_install").as_int()),

    select_algorithm(get_parameter("select_algorithm").as_int()),
    seedling_order(get_parameter("seedling_order").as_string_array()),
    planting_order(get_parameter("planting_order").as_string_array()),
    harvesting_order(get_parameter("harvesting_order").as_string_array())
    
{

    _subscription_seedling_collection = this->create_subscription<std_msgs::msg::Bool>(
        "Seedling_Collection",
        _qos,
        std::bind(&Sequencer::callback_seedling_collection, this, std::placeholders::_1)
    );

    _subscription_seedling_installation = this->create_subscription<std_msgs::msg::Bool>(
        "Seedling_Installation",
        _qos,
        std::bind(&Sequencer::callback_seedling_installation, this, std::placeholders::_1)
    );

    _subscription_ball_collection = this->create_subscription<std_msgs::msg::Bool>(
        "Ball_Collection",
        _qos,
        std::bind(&Sequencer::callback_ball_collection, this, std::placeholders::_1)
    );

    _subscription_convergence = this->create_subscription<controller_interface_msg::msg::Convergence>(
        "convergence",
        _qos,
        std::bind(&Sequencer::callback_convergence, this, std::placeholders::_1)
    );

    _subscription_color_information = this->create_subscription<controller_interface_msg::msg::Colorball>(
        "color_information",
        _qos,
        std::bind(&Sequencer::callback_color_information, this, std::placeholders::_1)
    );

    _subscription_base_control = this->create_subscription<controller_interface_msg::msg::BaseControl>(
        "base_control",
        _qos,
        std::bind(&Sequencer::callback_base_control, this, std::placeholders::_1)
    );

    _publisher_in_process = this->create_publisher<std_msgs::msg::Bool>("in_process", _qos);
    _publisher_move_node = this->create_publisher<std_msgs::msg::String>("move_node", _qos);
    _publisher_is_backside = this->create_publisher<std_msgs::msg::Bool>("is_backside", _qos);
    _publisher_canusb = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);

    set_in_process(false);

}

void Sequencer::callback_seedling_collection(const std_msgs::msg::Bool::SharedPtr msg){
    if(!in_process){
        set_in_process(true);
        sequence_mode = SEQUENCE_MODE::seedling;
    }
}

void Sequencer::callback_seedling_installation(const std_msgs::msg::Bool::SharedPtr msg){
    if(!in_process){
        set_in_process(true);
        sequence_mode = SEQUENCE_MODE::planting;
    }
}

void Sequencer::callback_ball_collection(const std_msgs::msg::Bool::SharedPtr msg){
    if(!in_process){
        set_in_process(true);
        sequence_mode = SEQUENCE_MODE::harvesting;
    }
}

void Sequencer::callback_convergence(const controller_interface_msg::msg::Convergence::SharedPtr msg){

    if(sequence_mode == SEQUENCE_MODE::seedling && seedling_step < 4){
        int n  = 0;
        if(sequence_process == n++) {
            RCLCPP_INFO(this->get_logger(), "苗回収シーケンス[%s]_起動", seedling_order[seedling_step].c_str());
            command_move_node(seedling_order[seedling_step]);
            sequence_process++;
        }
        else if(sequence_process == n++ && msg->spline_convergence){
            RCLCPP_INFO(this->get_logger(), "苗回収シーケンス[%s]_終了", seedling_order[seedling_step].c_str());
            seedling_step++;
            set_in_process(false);
            sequence_process = 0;
            sequence_mode = SEQUENCE_MODE::stop;
        }
    }

    else if(sequence_mode == SEQUENCE_MODE::planting && planting_step < 8){
        int n  = 0;
        if(sequence_process == n++) {
            RCLCPP_INFO(this->get_logger(), "苗設置シーケンス[%s]_起動", planting_order[planting_step].c_str());
            command_move_node(planting_order[planting_step]);
            sequence_process++;
        }
        else if(sequence_process == n++ && msg->spline_convergence){
            RCLCPP_INFO(this->get_logger(), "苗設置シーケンス[%s]_終了", planting_order[planting_step].c_str());
            planting_step++;
            set_in_process(false);
            sequence_process = 0;
            sequence_mode = SEQUENCE_MODE::stop;
        }
    }

    else if(sequence_mode == SEQUENCE_MODE::harvesting && harvesting_step < 12){
        int n  = 0;
        if(sequence_process == n++) {
            RCLCPP_INFO(this->get_logger(), "籾回収シーケンス[%s]_起動", harvesting_order[harvesting_step].c_str());
            command_move_node(harvesting_order[harvesting_step]);
            sequence_process++;
        }
        else if(sequence_process == n++ && msg->spline_convergence){
            RCLCPP_INFO(this->get_logger(), "籾回収シーケンス[%s]_終了", harvesting_order[harvesting_step].c_str());
            harvesting_step++;
            set_in_process(false);
            sequence_process = 0;
            sequence_mode = SEQUENCE_MODE::stop;
        }
    }

    else if(sequence_mode == SEQUENCE_MODE::injection){
        int n  = 0;
        if(sequence_process == n++) {
            RCLCPP_INFO(this->get_logger(), "射出シーケンス_起動");
            RCLCPP_INFO(this->get_logger(), "「移動開始」");
                command_move_node("IJ");
            RCLCPP_INFO(this->get_logger(), "「回転開始」");           
                command_inject_spinning(true);
            RCLCPP_INFO(this->get_logger(), "「空籾速度演算開始」");           
                command_is_backside(true);
            sequence_process++;
        }
        else if(sequence_process == n++ && msg->injection){
            RCLCPP_INFO(this->get_logger(), "「空籾装填」");
                if(color_order[inject_step]) is_right = true;
                else is_right = false;
                if(is_right) command_paddy_install(0);
                else command_paddy_install(1);
            sequence_process++;
        }
        else if(sequence_process == n++ && msg->spline_convergence && msg->injection_calculator && msg->injection && msg->ballhand/* && 射出命令*/){
            RCLCPP_INFO(this->get_logger(), "「空籾射出」"); 
                command_inject();
            sequence_process++;
        } 
        else if(sequence_process == n++ && !msg->injection){
            RCLCPP_INFO(this->get_logger(), "「籾速度演算開始」");
                command_is_backside(false);  
            sequence_process++;
        }
        else if(sequence_process == n++){
            RCLCPP_INFO(this->get_logger(), "「籾装填」");
                if(!is_right) command_paddy_install(1);
                else command_paddy_install(0);
            sequence_process++;
        }
        else if(sequence_process == n++ && msg->spline_convergence && msg->injection_calculator && msg->injection && msg->ballhand/* && 射出命令*/){
            RCLCPP_INFO(this->get_logger(), "「籾射出」");
                command_is_backside(false); 
            sequence_process++;
        }
        else if(sequence_process == n++ && !msg->injection){
            RCLCPP_INFO(this->get_logger(), "「回転停止」");           
                command_inject_spinning(false);
            RCLCPP_INFO(this->get_logger(), "射出シーケンス_終了");
            sequence_process = 0;
            sequence_mode = SEQUENCE_MODE::stop;
        } 
    }
}

void Sequencer::callback_color_information(const controller_interface_msg::msg::Colorball::SharedPtr msg){
    if(select_algorithm == 0){
        int count = 0;
        bool color = false;
        std::queue<int> own_color;
        std::queue<int> purple_color;
        for(int i = 6; i < 12; i++){
            if(msg->color_info[i]) own_color.push(i);
            else purple_color.push(i);
        }
        for(int i = 0; i < 6; i++){
            color = msg->color_info[i];
            harvesting_order[count] = "H" + std::to_string(i);
            color_order[count] = msg->color_info[i];
            count++;
            if(color) {
                harvesting_order[count] = "H" + std::to_string(purple_color.front());
                color_order[count] = msg->color_info[purple_color.front()];
                purple_color.pop();
            }
            else {
                harvesting_order[count] = "H" + std::to_string(own_color.front());
                color_order[count] = msg->color_info[own_color.front()];
                own_color.pop();        
            }
            count++;
        }
        RCLCPP_INFO(this->get_logger(), "籾の回収順");
        for(int i = 0; i < 12; i++) RCLCPP_INFO(this->get_logger(), "%s,%d", harvesting_order[i].c_str(),color_order[i]);
    }
}

void Sequencer::callback_base_control(const controller_interface_msg::msg::BaseControl::SharedPtr msg){
    if(msg->is_restart){
        set_in_process(false);
        sequence_mode = SEQUENCE_MODE::stop;
    }
}

void Sequencer::set_in_process(const bool flag){
    auto msg_in_process = std::make_shared<std_msgs::msg::Bool>();
    msg_in_process->data = flag;
    _publisher_in_process->publish(*msg_in_process);
    in_process = flag;
    sequence_process = 0;
}

void Sequencer::command_move_node(const std::string node){
    auto msg_move_node = std::make_shared<std_msgs::msg::String>();
    msg_move_node->data = node;
    _publisher_move_node->publish(*msg_move_node);
};

void Sequencer::command_is_backside(const bool flag){
    auto msg_is_backside = std::make_shared<std_msgs::msg::Bool>();
    msg_is_backside->data = flag;
    _publisher_is_backside->publish(*msg_is_backside);    
};

void Sequencer::command_canusb_empty(const int16_t id, const uint8_t dlc){
    auto msg_canusb = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_canusb->canid = id;
    msg_canusb->candlc = dlc;
    _publisher_canusb->publish(*msg_canusb);  
};

void Sequencer::command_canusb_uint8(const int16_t id, const uint8_t dlc, const uint8_t data[8]){
    auto msg_canusb = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_canusb->canid = id;
    msg_canusb->candlc = dlc;
    for(int i = 0; i < dlc; i++) msg_canusb->candata[i] = data[i];
    _publisher_canusb->publish(*msg_canusb);
};

void Sequencer::command_inject_spinning(const bool flag){
    uint8_t data[8];
    data[0] = flag; 
    command_canusb_uint8(canid_inject_spinning, 1, data);
};

void Sequencer::command_inject(){
    command_canusb_empty(canid_inject_spinning, 1);
};

void Sequencer::command_seedling_collect(const uint8_t num){
    uint8_t data[8];
    data[0] = num; 
    command_canusb_uint8(canid_seedling_collect, 1, data);
};

void Sequencer::command_seedling_install(const uint8_t num){
    uint8_t data[8];
    data[0] = num; 
    command_canusb_uint8(canid_seedling_install, 1, data);
};

void Sequencer::command_paddy_collect(const uint8_t num){
    uint8_t data[8];
    data[0] = num; 
    command_canusb_uint8(canid_paddy_collect, 1, data);
};

void Sequencer::command_paddy_install(const uint8_t num){
    uint8_t data[8];
    data[0] = num; 
    command_canusb_uint8(canid_paddy_install, 1, data);
};

}  // namespace sequencer