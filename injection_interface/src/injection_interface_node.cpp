#include "injection_interface/injection_interface_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <iostream>

#include "utilities/can_utils.hpp"

using namespace std;
using namespace utils;

namespace injection_interface{
    InjectionInterface::InjectionInterface(const rclcpp::NodeOptions &options) : InjectionInterface("", options) {}
    InjectionInterface::InjectionInterface(const std::string &name_space, const rclcpp::NodeOptions &options)
        : rclcpp::Node("injection_interface_node", name_space, options),
        tf_injection2robot(get_parameter("tf_injection2robot").as_double_array()),
        strage_backside_0(get_parameter("strage_backside_0").as_double_array()),
        strage_backside_1(get_parameter("strage_backside_1").as_double_array()),
        strage_front(get_parameter("strage_front").as_double_array()),
        linear_pitch(get_parameter("linear_pitch").as_double_array()),
        linear_tf(get_parameter("linear_tf").as_double_array()),
        can_inject_pitch_id(get_parameter("canid.inject_pitch").as_int()),
        injectionpoint_tolerance(get_parameter("injectionpoint_tolerance").as_double_array())
        
        {
            _subscriber_injection_calculate = this->create_subscription<std_msgs::msg::Empty>(
                "injection_calculate",
                _qos,
                std::bind(&InjectionInterface::_callback_injection_calculate, this, std::placeholders::_1)
            );

            _subscriber_is_move_tracking = this->create_subscription<std_msgs::msg::Bool>(
                "is_move_tracking",
                _qos,
                std::bind(&InjectionInterface::_callback_is_move_tracking, this, std::placeholders::_1)
            );

            _subscriber_self_pose = this->create_subscription<geometry_msgs::msg::Vector3>(
                "self_pose",
                rclcpp::SensorDataQoS(),
                std::bind(&InjectionInterface::_callback_self_pose, this, std::placeholders::_1)
            );

            _subscriber_move_target_pose = this->create_subscription<geometry_msgs::msg::Vector3>(
                "move_target_pose",
                _qos,
                std::bind(&InjectionInterface::_callback_move_target_pose, this, std::placeholders::_1)
            );

            _subscriber_target_node = this->create_subscription<std_msgs::msg::String>(
                "target_node",
                _qos,
                std::bind(&InjectionInterface::_callback_target_node, this, std::placeholders::_1)
            );

            _subscription_base_control = this->create_subscription<controller_interface_msg::msg::BaseControl>(
                "base_control",
                _qos,
                std::bind(&InjectionInterface::_subscriber_callback_base_control, this, std::placeholders::_1)
            );

            _publisher_injection = this->create_publisher<injection_interface_msg::msg::InjectionCommand>("injection_command", 10);
            _publisher_spin_position = this->create_publisher<path_msg::msg::Turning>("spin_position", 10);
            _publisher_canusb = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);

            const std::string court_color = this->get_parameter("court_color").as_string();
            const auto initial_pose = this->get_parameter("initial_pose").as_double_array();
            const int coord_sign = (court_color == "blue") ? 1 : -1; 
            self_pose.x = initial_pose[0];
            self_pose.y = coord_sign * initial_pose[1];
            self_pose.z = coord_sign * initial_pose[2];
            strage_front[1] *= coord_sign;
            strage_backside_0[1] *= coord_sign;
            strage_backside_1[1] *= coord_sign;

            std::ifstream ifs0(ament_index_cpp::get_package_share_directory("main_executor") + "/config/injection_interface/injection_gain.cfg");
            std::string str0;
            while(getline(ifs0, str0)){
                std::string token;
                std::istringstream stream(str0);
                int count = 0;
                while(getline(stream, token, ' ')){
                    if(count==1) injection_gain.push_back(std::stod(token));
                    count++;
                }
            }

            std::ifstream ifs1(ament_index_cpp::get_package_share_directory("main_executor") + "/config/spline_pid/nodelist.cfg");
            std::string str1;
            while(getline(ifs1, str1)){
                std::string token;
                std::istringstream stream(str1);
                int count = 0;
                Node node;
                while(getline(stream, token, ' ')){
                    if(count==0) node.name = token;
                    else if(count==1) node.x = std::stold(token);
                    else if(count==2) node.y = std::stold(token) * coord_sign;
                    count++;
                }
                node_list.push_back(node);
            }
        }

        void InjectionInterface::_callback_injection_calculate(const std_msgs::msg::Empty::SharedPtr msg){
            set_calculate_vel();
        }

        void InjectionInterface::_callback_is_move_tracking(const std_msgs::msg::Bool::SharedPtr msg){
            is_move_tracking = msg->data;
            if(is_correction_required && !msg->data) set_calculate_vel();
        }

        void InjectionInterface::_callback_self_pose(const geometry_msgs::msg::Vector3::SharedPtr msg){
            self_pose.x = msg->x;
            self_pose.y = msg->y;
            self_pose.z = msg->z;
            // RCLCPP_INFO(get_logger(),"x:%f y:%f z:%f",self_pose.x,self_pose.y,self_pose.z);
            for(int i = 0; i < node_list.size(); i++){
                if(abs(node_list[i].x - self_pose.x) <= injectionpoint_tolerance[0] && abs(node_list[i].y - self_pose.y) <= injectionpoint_tolerance[1]){
                    if(old_injectionpoint ==  node_list[i].name) break;
                    injection_point = node_list[i].name;
                    old_injectionpoint = injection_point;
                    if(injection_point.length() >= 2 && !is_move_autonomous){
                        if(injection_point[0] == 'H') {
                            command_injection_pitch(linear_pitch[0]);
                            injection_num = std::stoi(injection_point.substr(1));              
                        }
                        else if(injection_point[0] == 'I') {
                            command_injection_pitch(linear_pitch[1]);
                            injection_num = 13;
                        }
                        // RCLCPP_INFO(get_logger(),"%d",injection_num);
                    }
                }
            }
        }

        void InjectionInterface::_callback_move_target_pose(const geometry_msgs::msg::Vector3::SharedPtr msg){
            move_target_pose.x = msg->x;
            move_target_pose.y = msg->y;
            move_target_pose.z = msg->z;
            RCLCPP_INFO(get_logger(),"x:%f y:%f z:%f",move_target_pose.x,move_target_pose.y,move_target_pose.z);
        }

        void InjectionInterface::_callback_target_node(const std_msgs::msg::String::SharedPtr msg){
            if(msg->data.length() >= 2){
                if(msg->data[0] == 'H') {
                    command_injection_pitch(linear_pitch[0]);
                    injection_num = std::stoi(msg->data.substr(1));              
                }
                else if(msg->data[0] == 'I') {
                    command_injection_pitch(linear_pitch[1]);
                    injection_num = 13;
                }
            }
        }

        void InjectionInterface::set_calculate_vel(){
            bool target_input = false;
            TwoVector injection_pos; //ロボットの本体座標と射出機構のずれを補正した数字
            geometry_msgs::msg::Vector3 robot_pose;
            is_correction_required = false;
            robot_pose = self_pose;     

            switch (injection_num){
                case 0:
                case 1:
                case 2:
                case 3:
                case 6:
                case 7:
                case 8:
                case 9:
                case 12:
                    RCLCPP_INFO(this->get_logger(), "backside_vel_0");
                    target_pos.x = strage_backside_0[0];
                    target_pos.y = strage_backside_0[1];
                    target_height = strage_backside_0[2];
                    injection_pos.x = robot_pose.x + linear_tf[0]*cos(robot_pose.z);
                    injection_pos.y = robot_pose.y + linear_tf[0]*sin(robot_pose.z);
                    command_injection_pitch(linear_pitch[0]);
                    target_input = true;         
                    break;
                case 4:
                case 5:
                case 10:
                case 11:
                    RCLCPP_INFO(this->get_logger(), "backside_vel_1");
                    target_pos.x = strage_backside_1[0];
                    target_pos.y = strage_backside_1[1];
                    target_height = strage_backside_1[2];
                    injection_pos.x = robot_pose.x + linear_tf[0]*cos(robot_pose.z);
                    injection_pos.y = robot_pose.y + linear_tf[0]*sin(robot_pose.z);
                    command_injection_pitch(linear_pitch[0]);
                    target_input = true;
                    break;
                case 13:
                    RCLCPP_INFO(this->get_logger(), "frontside_vel");
                    target_pos.x = strage_front[0];
                    target_pos.y = strage_front[1];
                    target_height = strage_front[2];
                    injection_pos.x = robot_pose.x + linear_tf[1]*cos(robot_pose.z);
                    injection_pos.y = robot_pose.y + linear_tf[1]*sin(robot_pose.z); 
                    command_injection_pitch(linear_pitch[1]);
                    target_input = true;
                    break;
            }
            
            if(!target_input) {
                RCLCPP_INFO(this->get_logger(), "射出位置の入力ができませんでした");
                return;
            }

            diff = target_pos - injection_pos;
            command_calculation_vel();
            command_injection_turn();    
        }

        void InjectionInterface::command_calculation_vel(){
            auto injection_command = std::make_shared<injection_interface_msg::msg::InjectionCommand>();
            injection_command->distance = diff.length();
            cout<<diff.length()<<endl;
            injection_command->height = target_height;
            injection_command->pitch = pitch;
            injection_command->gain = injection_gain[injection_num];
            _publisher_injection->publish(*injection_command);
        }

        void InjectionInterface::command_injection_turn(){
            float self_z = self_pose.z;
            auto injection_angle = std::make_shared<path_msg::msg::Turning>();
            injection_angle->angle_pos = atan2(target_pos.y - self_pose.y, target_pos.x - self_pose.x);// - area(self_z, -f_pi, f_pi);
            injection_angle->accurate_convergence = true;
            _publisher_spin_position->publish(*injection_angle);
        }

        void InjectionInterface::command_injection_pitch(double linear_pitch){
            pitch = linear_pitch;
            // cout<<"linear_pitch:"<<linear_pitch<<endl;
            uint8_t _candata[8];
            float_to_bytes(_candata, static_cast<float>(linear_pitch));
            auto msg_injection_pitch = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_injection_pitch->canid = can_inject_pitch_id;
            msg_injection_pitch->candlc = 4;
            for(int i=0; i<msg_injection_pitch->candlc; i++) msg_injection_pitch->candata[i] = _candata[i];
            _publisher_canusb->publish(*msg_injection_pitch);
        }

        void InjectionInterface::_subscriber_callback_base_control(const controller_interface_msg::msg::BaseControl::SharedPtr msg){
            is_move_autonomous = msg->is_move_autonomous;
        }

}  // namespace injection_interface