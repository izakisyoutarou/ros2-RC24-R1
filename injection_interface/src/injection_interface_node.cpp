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
        strage_backside(get_parameter("strage_backside").as_double_array()),
        strage_front(get_parameter("strage_front").as_double_array()),
        court_color_(get_parameter("court_color").as_string()),
        can_backspin_vel_id(get_parameter("canid.backspin_vel").as_int())
        
        {
            _sub_is_backside = this->create_subscription<std_msgs::msg::Bool>(
                "is_backside",
                _qos,
                std::bind(&InjectionInterface::_callback_is_backside, this, std::placeholders::_1)
            );

            _sub_is_move_tracking = this->create_subscription<std_msgs::msg::Bool>(
                "is_move_tracking",
                _qos,
                std::bind(&InjectionInterface::_callback_is_move_tracking, this, std::placeholders::_1)
            );

            _sub_self_pose = this->create_subscription<geometry_msgs::msg::Vector3>(
                "self_pose",
                _qos,
                std::bind(&InjectionInterface::_callback_self_pose, this, std::placeholders::_1)
            );

            _sub_move_target_pose = this->create_subscription<geometry_msgs::msg::Vector3>(
                "move_target_pose",
                _qos,
                std::bind(&InjectionInterface::_callback_move_target_pose, this, std::placeholders::_1)
            );

            _sub_backspin_vel = this->create_subscription<std_msgs::msg::Int16MultiArray>(
                "backspin_vel",
                _qos,
                std::bind(&InjectionInterface::_callback_backspin_vel, this, std::placeholders::_1)
            );

            _pub_injection = this->create_publisher<injection_interface_msg::msg::InjectionCommand>("injection_command", 10);
            _pub_spin_position = this->create_publisher<path_msg::msg::Turning>("spin_position", 10);
            _pub_canusb = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);

            const auto initial_pose = this->get_parameter("initial_pose").as_double_array();
            if(court_color_ == "blue"){
                self_pose.x = initial_pose[0];
                self_pose.y = initial_pose[1];
                self_pose.z = initial_pose[2];
            }else if(court_color_ == "red"){
                self_pose.x = initial_pose[0];
                self_pose.y = -initial_pose[1];
                self_pose.z = initial_pose[2];
            }

            std::ifstream ifs(ament_index_cpp::get_package_share_directory("main_executor") + "/config/injection_interface/injection_vel.cfg");
            std::string str;
            while(getline(ifs, str)){
                std::string token;
                std::istringstream stream(str);
                int count = 0;
                Vel vel;
                while(getline(stream, token, ' ')){
                    if(count==0) vel.name = token;
                    else if(count==1) vel.vel[0] = std::stoi(token);
                    else if(count==2) vel.vel[1] = std::stoi(token);
                    else if(count==3) vel.vel[2] = std::stoi(token);
                    count++;
                }
                vel_list.push_back(vel);
            }
        }

        void InjectionInterface::_callback_is_backside(const std_msgs::msg::Bool::SharedPtr msg){
           
            TwoVector target_pos;
            double target_height;
            bool target_input = false;
            last_target = msg;

            if(msg->data){
                if(court_color_ == "blue"){
                    target_pos.x = strage_backside[0];
                    target_pos.y = strage_backside[1];
                    target_height = strage_backside[2];
                    target_input = true;
                }else if(court_color_ == "red"){
                    target_pos.x = strage_backside[0];
                    target_pos.y = -strage_backside[1];
                    target_height = strage_backside[2];
                    target_input = true;
                }
            }
            else{
                if(court_color_ == "blue"){
                    target_pos.x = strage_front[0];
                    target_pos.y = strage_front[1];
                    target_height = strage_front[2];
                    target_input = true;
                }else if(court_color_ == "red"){
                    target_pos.x = strage_front[0];
                    target_pos.y = -strage_front[1];
                    target_height = strage_front[2];
                    target_input = true;
                }
            }

            if(!target_input){
                RCLCPP_INFO(this->get_logger(), "射出位置の入力ができませんでした");
                return;
            }

            geometry_msgs::msg::Vector3 robot_pose;

            if(is_move_tracking){
                is_correction_required = true;
                robot_pose = move_target_pose;
            }
            else{
                is_correction_required = false;
                robot_pose = self_pose;
            }

            //ロボットの本体座標と射出機構のずれを補正した数字
            TwoVector injection_pos;
            injection_pos.x = robot_pose.x + tf_injection2robot[0]*cos(robot_pose.z) - tf_injection2robot[1]*sin(robot_pose.z);
            injection_pos.y = robot_pose.y + tf_injection2robot[0]*sin(robot_pose.z) + tf_injection2robot[1]*cos(robot_pose.z);

            RCLCPP_INFO(this->get_logger(), "射出位置 x:%lf y:%lf", injection_pos.x, injection_pos.y);

            TwoVector diff = target_pos - injection_pos;

            auto injection_command = std::make_shared<injection_interface_msg::msg::InjectionCommand>();
            injection_command->distance = diff.length();
            injection_command->height = target_height;
            _pub_injection->publish(*injection_command);

            float self_z = self_pose.z;

            auto injection_angle = std::make_shared<path_msg::msg::Turning>();
            injection_angle->angle_pos = atan2(target_pos.y - self_pose.y , target_pos.x - self_pose.x) - area(self_z, -f_pi, f_pi);
            injection_angle->accurate_convergence = true;
            _pub_spin_position->publish(*injection_angle);
        }

        void InjectionInterface::_callback_is_move_tracking(const std_msgs::msg::Bool::SharedPtr msg){
            is_move_tracking = msg->data;
            if(is_correction_required && !msg->data) _callback_is_backside(last_target);
        }

        void InjectionInterface::_callback_self_pose(const geometry_msgs::msg::Vector3::SharedPtr msg){
            self_pose.x = msg->x;
            self_pose.y = msg->y;
            self_pose.z = msg->z;
        }

        void InjectionInterface::_callback_move_target_pose(const geometry_msgs::msg::Vector3::SharedPtr msg){
            move_target_pose.x = msg->x;
            move_target_pose.y = msg->y;
            move_target_pose.z = msg->z;
        }

        void InjectionInterface::_callback_backspin_vel(const std_msgs::msg::Int16MultiArray::SharedPtr msg){
            int16_t backspin_vel[3] = {0, 0, 0};
            backspin_vel[0] = msg->data[0];
            backspin_vel[1] = msg->data[1];
            backspin_vel[2] = msg->data[2];
            command_backspin(backspin_vel);
            RCLCPP_INFO(this->get_logger(), "%d, %d, %d", backspin_vel[0], backspin_vel[1], backspin_vel[2]);
        }

        void InjectionInterface::command_backspin(int16_t vel[3]){
            uint8_t _candata[8];
            auto msg_backspin_vel = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_backspin_vel->canid = can_backspin_vel_id;
            msg_backspin_vel->candlc = 6;
            short_to_bytes(_candata, static_cast<short>(vel[0]));
            short_to_bytes(_candata+2, static_cast<short>(vel[1]));
            short_to_bytes(_candata+4, static_cast<short>(vel[2]));
            for(int i=0; i<msg_backspin_vel->candlc; i++) msg_backspin_vel->candata[i] = _candata[i];
            _pub_canusb->publish(*msg_backspin_vel);
        }

}  // namespace injection_interface