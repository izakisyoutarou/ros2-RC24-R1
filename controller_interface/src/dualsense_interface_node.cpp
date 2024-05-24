#include "controller_interface/dualsense_interface_node.hpp"
#include <sys/time.h>
#include <sys/types.h>
#include <chrono>
#include <iostream>
#include <future>
#include <boost/format.hpp>

namespace controller_interface
{

    using std::string;

    DualSense::DualSense(const rclcpp::NodeOptions &options) : DualSense("", options){}
    DualSense::DualSense(const std::string &name_space, const rclcpp::NodeOptions &options)
        : rclcpp::Node("controller_interface_node", name_space, options),
        
        high_limit_linear(DBL_MAX,
        get_parameter("high_linear_max_vel").as_double(),
        get_parameter("high_linear_max_acc").as_double(),
        get_parameter("high_linear_max_dec").as_double() ),
        slow_limit_linear(DBL_MAX,
        get_parameter("slow_linear_max_vel").as_double(),
        get_parameter("slow_linear_max_acc").as_double(),
        get_parameter("slow_linear_max_dec").as_double() ),
        limit_angular(DBL_MAX,
        dtor(get_parameter("angular_max_vel").as_double()),
        dtor(get_parameter("angular_max_acc").as_double()),
        dtor(get_parameter("angular_max_dec").as_double()) ),

        high_manual_linear_max_vel(static_cast<float>(get_parameter("high_linear_max_vel").as_double())),
        slow_manual_linear_max_vel(static_cast<float>(get_parameter("slow_linear_max_vel").as_double())),
        manual_angular_max_vel(dtor(static_cast<float>(get_parameter("angular_max_vel").as_double()))),

        defalt_restart_flag(get_parameter("defalt_restart_flag").as_bool()),
        defalt_emergency_flag(get_parameter("defalt_emergency_flag").as_bool()),
        defalt_move_autonomous_flag(get_parameter("defalt_move_autonomous_flag").as_bool()),
        defalt_injection_autonomous_flag(get_parameter("defalt_injection_autonomous_flag").as_bool()),
        defalt_slow_speed_flag(get_parameter("defalt_slow_speed_flag").as_bool()),

        defalt_spline_convergence(get_parameter("defalt_spline_convergence").as_bool()),
        defalt_injection_calculator_convergence(get_parameter("defalt_injection_calculator_convergence").as_bool()),
        defalt_injection_convergence(get_parameter("defalt_injection_convergence").as_bool()),
        defalt_seedlinghand_convergence(get_parameter("defalt_seedlinghand_convergence").as_bool()),
        defalt_ballhand_convergence(get_parameter("defalt_ballhand_convergence").as_bool()),

        can_emergency_id(get_parameter("canid.emergency").as_int()),
        can_heartbeat_id(get_parameter("canid.heartbeat").as_int()),
        can_restart_id(get_parameter("canid.restart").as_int()),
        can_calibrate_id(get_parameter("canid.calibrate").as_int()),
        can_reset_id(get_parameter("canid.reset").as_int()),
        can_emergency_state_id(get_parameter("canid.emergency_state").as_int()),
        can_linear_id(get_parameter("canid.linear").as_int()),
        can_angular_id(get_parameter("canid.angular").as_int()),
        can_steer_reset_id(get_parameter("canid.steer_reset").as_int()),
        can_inject_id(get_parameter("canid.inject").as_int()),
        can_inject_convergence_id(get_parameter("canid.inject_convergence").as_int()),
        can_inject_calibration_id(get_parameter("canid.inject_calibration").as_int()),
        can_seedling_collect_id(get_parameter("canid.seedling_collect").as_int()),
        can_seedling_install_id(get_parameter("canid.seedling_install").as_int()),
        can_seedling_convergence_id(get_parameter("canid.seedling_convergence").as_int()),
        can_paddy_collect_id(get_parameter("canid.paddy_collect").as_int()),
        can_paddy_install_id(get_parameter("canid.paddy_install").as_int()),
        can_paddy_convergence_id(get_parameter("canid.paddy_convergence").as_int()),
        can_arm_expansion_id(get_parameter("canid.arm_expansion").as_int()),
        can_led_id(get_parameter("canid.led").as_int()),
        connection_check(get_parameter("connection_check").as_bool())

        {

            const auto heartbeat_ms = this->get_parameter("heartbeat_ms").as_int();
            const auto convergence_ms = this->get_parameter("convergence_ms").as_int();
            const auto controller_ms = this->get_parameter("controller_ms").as_int();
            const auto mainboard_ms = this->get_parameter("mainboard_ms").as_int();

            gamebtn.canid.calibrate = can_calibrate_id;
            gamebtn.canid.reset = can_reset_id;
            gamebtn.canid.steer_reset = can_steer_reset_id;
            gamebtn.canid.inject = can_inject_id;
            gamebtn.canid.paddy_collect = can_paddy_collect_id;
            gamebtn.canid.paddy_install = can_paddy_install_id;
            gamebtn.canid.seedling_collect = can_seedling_collect_id;
            gamebtn.canid.seedling_install = can_seedling_install_id;
            gamebtn.canid.arm_expansion = can_arm_expansion_id;
            gamebtn.canid.inject_calibration = can_inject_calibration_id;

            _sub_dualsense = this->create_subscription<sensor_msgs::msg::Joy>(
                "/joy",
                _qos,
                std::bind(&DualSense::callback_dualsense, this, std::placeholders::_1)
            );

            _sub_emergency_state = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
                "can_rx_"+ (boost::format("%x") % can_emergency_state_id).str(),
                _qos,
                std::bind(&DualSense::callback_emergency_state, this, std::placeholders::_1)
            );
            _sub_inject_convergence = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
                "can_rx_"+ (boost::format("%x") %  can_inject_convergence_id).str(),
                _qos,
                std::bind(&DualSense::callback_inject_convergence, this, std::placeholders::_1)
            );
            _sub_seedling_convergence = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
                "can_rx_"+ (boost::format("%x") %  can_seedling_convergence_id).str(),
                _qos,
                std::bind(&DualSense::callback_seedling_convergence, this, std::placeholders::_1)
            );
            _sub_paddy_convergence = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
                "can_rx_"+ (boost::format("%x") %  can_paddy_convergence_id).str(),
                _qos,
                std::bind(&DualSense::callback_paddy_convergence, this, std::placeholders::_1)
            );

            _sub_is_move_tracking = this->create_subscription<std_msgs::msg::Bool>(
                "is_move_tracking",
                _qos,
                std::bind(&DualSense::callback_is_move_tracking, this, std::placeholders::_1)
            );

            _sub_calculator_convergence = this->create_subscription<std_msgs::msg::Bool>(
                "calculator_convergenced_",
                _qos,
                std::bind(&DualSense::callback_calculator_convergence, this, std::placeholders::_1)
            );
            _sub_inject_calibration = this->create_subscription<std_msgs::msg::Empty>(
                "Injection_Calibration",
                _qos,
                std::bind(&DualSense::callback_inject_calibration, this, std::placeholders::_1)
            );
            _sub_move_autonomous = this->create_subscription<std_msgs::msg::Bool>(
                "move_autonomous",
                _qos,
                std::bind(&DualSense::callback_move_autonomous, this, std::placeholders::_1)
            );

            _pub_canusb = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);
            _pub_base_control = this->create_publisher<controller_interface_msg::msg::BaseControl>("base_control",_qos);
            _pub_convergence = this->create_publisher<controller_interface_msg::msg::Convergence>("convergence" , _qos);
            _pub_injection_calculate = this->create_publisher<std_msgs::msg::Empty>("injection_calculate", _qos);
            _pub_target_node = this->create_publisher<std_msgs::msg::String>("target_node", _qos);
            _pub_gazebo = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", _qos);
            _pub_is_start = this->create_publisher<std_msgs::msg::UInt8>("is_start", _qos);
            
            msg_base_control.is_restart = defalt_restart_flag;
            msg_base_control.is_emergency = defalt_emergency_flag;
            msg_base_control.is_move_autonomous = defalt_move_autonomous_flag;
            msg_base_control.is_injection_autonomous = defalt_injection_autonomous_flag;
            msg_base_control.is_slow_speed= defalt_slow_speed_flag;
            msg_base_control.initial_state= "O";
            _pub_base_control->publish(msg_base_control);

            auto msg_emergency = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_emergency->canid = can_emergency_id;
            msg_emergency->candlc = 1;
            msg_emergency->candata[0] = defalt_emergency_flag;
            _pub_canusb->publish(*msg_emergency);

            msg_convergence.spline_convergence = defalt_spline_convergence;
            msg_convergence.injection_calculator = defalt_injection_calculator_convergence;
            msg_convergence.injection = defalt_injection_convergence;
            msg_convergence.seedlinghand = defalt_seedlinghand_convergence;
            msg_convergence.ballhand = defalt_ballhand_convergence;
            _pub_convergence->publish(msg_convergence);

            //ハートビート
            _pub_heartbeat = this->create_wall_timer(
                std::chrono::milliseconds(heartbeat_ms),
                [this] {
                    auto msg_heartbeat = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();  
                    msg_heartbeat->canid = can_heartbeat_id;
                    msg_heartbeat->candlc = 0;
                    _pub_canusb->publish(*msg_heartbeat);
                }
            );

            //convergence
            _pub_timer_convergence = this->create_wall_timer(
                std::chrono::milliseconds(convergence_ms),
                [this] {
                    _pub_convergence->publish(msg_convergence);
                }
            );

            Joystick_timer = this->create_wall_timer(
                std::chrono::milliseconds(this->get_parameter("interval_ms").as_int()),
                std::bind(&DualSense::callback_Joystick, this) 
            );

            if(connection_check){
                check_mainboard_connection = this->create_wall_timer(
                    std::chrono::milliseconds(static_cast<int>(mainboard_ms)),
                    [this] { 
                        if(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - get_mainboard_time).count() > 500 * 10){
                            msg_base_control.is_restart = false;
                            msg_base_control.is_emergency = true;
                            _pub_base_control->publish(msg_base_control);
                            RCLCPP_INFO(get_logger(),"mainboard_connection_lost!!");
                        }
                    }
                );
            }

            //計画機にリミットを設定する
            high_velPlanner_linear_x.limit(high_limit_linear);
            high_velPlanner_linear_y.limit(high_limit_linear);

            slow_velPlanner_linear_x.limit(slow_limit_linear);
            slow_velPlanner_linear_y.limit(slow_limit_linear);

            velPlanner_angular_z.limit(limit_angular);
        }

        void DualSense::callback_dualsense(const sensor_msgs::msg::Joy::SharedPtr msg){
            for(int i = 0; i < 13; i++) buttons[i] = upedge_buttons[i](msg->buttons[i]);

            for(int i = 0; i < 8; i++) axes[i] = msg->axes[i];
            
            if(axes[LR] == 1.0){
                LRUD[left] = true;
                LRUD[right] = false;
            }
            else if(axes[LR] == -1.0){
                LRUD[left] = false;
                LRUD[right] = true;
            }
            else {
                LRUD[left] = false;
                LRUD[right] = false;
            }

            if(axes[UD] == 1.0){
                LRUD[up] = true;
                LRUD[down] = false;
            }
            else if(axes[UD] == -1.0){
                LRUD[up] = false;
                LRUD[down] = true;
            }
            else {
                LRUD[up] = false;
                LRUD[down] = false;
            }
            for(int i = 0; i < 4; i++) LRUD[i] = upedge_LRUD[i](LRUD[i]);
  

            msg_base_control.is_restart = false;

            if(buttons[ps5_buttons::create]){
                cout<<"emergency"<<endl;
                auto msg_emergency = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                msg_emergency->canid = can_emergency_id;
                msg_emergency->candlc = 1;
                msg_emergency->candata[0] = true;
                _pub_canusb->publish(*msg_emergency);
            }
            else if(buttons[ps5_buttons::option]){
                cout<<"restart"<<endl;
                auto msg_emergency = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                msg_emergency->canid = can_emergency_id;
                msg_emergency->candlc = 1;
                auto msg_restart = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                msg_restart->canid = can_restart_id;
                msg_restart->candlc = 0;
                msg_base_control.is_move_autonomous = defalt_move_autonomous_flag;
                msg_base_control.is_slow_speed= defalt_slow_speed_flag;
                msg_convergence.spline_convergence = defalt_spline_convergence;
                msg_convergence.injection_calculator = defalt_injection_calculator_convergence;
                msg_convergence.injection  = defalt_injection_convergence;
                msg_convergence.seedlinghand = defalt_seedlinghand_convergence;
                msg_convergence.ballhand = defalt_ballhand_convergence; 
                arm_expansion_flag = false;
                msg_base_control.is_restart = true;
                _pub_canusb->publish(*msg_restart);
                _pub_canusb->publish(*msg_emergency);
                _pub_base_control->publish(msg_base_control);         
            }
            else if(buttons[ps5_buttons::circle] && arm_expansion_flag) gamebtn.seedling_collect_right(msg_convergence.seedlinghand,_pub_canusb);          
            else if(buttons[ps5_buttons::cross] && arm_expansion_flag) gamebtn.seedling_collect_left(msg_convergence.seedlinghand,_pub_canusb);               
            else if(buttons[ps5_buttons::triangle] && arm_expansion_flag) gamebtn.seedling_install_right(msg_convergence.seedlinghand,_pub_canusb);   
            else if(buttons[ps5_buttons::square] && arm_expansion_flag) gamebtn.seedling_install_left(msg_convergence.seedlinghand,_pub_canusb);
            else if(LRUD[ps5_LRUD::up]) gamebtn.steer_reset(_pub_canusb);
            else if(LRUD[ps5_LRUD::down]) gamebtn.calibrate(_pub_canusb);                
            else if(LRUD[ps5_LRUD::left]) gamebtn.board_reset(_pub_canusb);
            else if(LRUD[ps5_LRUD::right]){
                auto msg_is_start = std::make_shared<std_msgs::msg::UInt8>();
                msg_is_start->data = 0;
                _pub_is_start->publish(*msg_is_start);
            }
            else if(buttons[ps5_buttons::r1]) gamebtn.injection(msg_convergence.injection ,msg_convergence.injection_calculator,_pub_canusb); 
            else if(buttons[ps5_buttons::r2]) gamebtn.injection_calculate(_pub_injection_calculate);
            else if(buttons[ps5_buttons::r3]){
                msg_base_control.is_move_autonomous = !msg_base_control.is_move_autonomous;
                _pub_base_control->publish(msg_base_control);
                 if(msg_base_control.is_move_autonomous) gamebtn.led(3,_pub_canusb);
            }
            else if(buttons[ps5_buttons::l1]) gamebtn.paddy_control(msg_convergence.ballhand,_pub_canusb); 
            else if(buttons[ps5_buttons::l2]) {
                msg_base_control.is_slow_speed = !msg_base_control.is_slow_speed;
                _pub_base_control->publish(msg_base_control);
                if(!msg_base_control.is_move_autonomous && !msg_base_control.is_slow_speed) gamebtn.led(1,_pub_canusb);
                else if(!msg_base_control.is_move_autonomous && msg_base_control.is_slow_speed)gamebtn.led(2,_pub_canusb);
            }
            else if(buttons[ps5_buttons::l3]) {
                gamebtn.arm_expansion(_pub_canusb);
                arm_expansion_flag = true;
            }
        }

        void DualSense::callback_Joystick()
        {
            auto msg_linear = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_linear->canid = can_linear_id;
            msg_linear->candlc = 8;
            auto msg_angular = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_angular->canid = can_angular_id;
            msg_angular->candlc = 4;
            auto msg_gazebo = std::make_shared<geometry_msgs::msg::Twist>();
            uint8_t _candata_joy[8];

            if(msg_base_control.is_slow_speed){
                slow_velPlanner_linear_x.vel(axes[ps5_axes::ly]);
                slow_velPlanner_linear_y.vel(axes[ps5_axes::lx]);
                velPlanner_angular_z.vel(axes[ps5_axes::rx]);

                slow_velPlanner_linear_x.cycle();
                slow_velPlanner_linear_y.cycle();
                velPlanner_angular_z.cycle();

                float_to_bytes(_candata_joy, static_cast<float>(slow_velPlanner_linear_x.vel()) * slow_manual_linear_max_vel);
                float_to_bytes(_candata_joy+4, static_cast<float>(slow_velPlanner_linear_y.vel()) * slow_manual_linear_max_vel);
                for(int i=0; i<msg_linear->candlc; i++) msg_linear->candata[i] = _candata_joy[i];

                float_to_bytes(_candata_joy, static_cast<float>(velPlanner_angular_z.vel()) * manual_angular_max_vel);
                for(int i=0; i<msg_angular->candlc; i++) msg_angular->candata[i] = _candata_joy[i];

                msg_gazebo->linear.x = slow_velPlanner_linear_x.vel();
                msg_gazebo->linear.y = slow_velPlanner_linear_y.vel();
                msg_gazebo->angular.z = velPlanner_angular_z.vel();
            }
            else{
                high_velPlanner_linear_x.vel(axes[ps5_axes::ly]);
                high_velPlanner_linear_y.vel(axes[ps5_axes::lx]);
                velPlanner_angular_z.vel(axes[ps5_axes::rx]);

                high_velPlanner_linear_x.cycle();
                high_velPlanner_linear_y.cycle();
                velPlanner_angular_z.cycle();

                float_to_bytes(_candata_joy, static_cast<float>(high_velPlanner_linear_x.vel()) * high_manual_linear_max_vel);
                float_to_bytes(_candata_joy+4, static_cast<float>(high_velPlanner_linear_y.vel()) * high_manual_linear_max_vel);
                for(int i=0; i<msg_linear->candlc; i++) msg_linear->candata[i] = _candata_joy[i];

                float_to_bytes(_candata_joy, static_cast<float>(velPlanner_angular_z.vel()) * manual_angular_max_vel);
                for(int i=0; i<msg_angular->candlc; i++) msg_angular->candata[i] = _candata_joy[i];

                msg_gazebo->linear.x = high_velPlanner_linear_x.vel();
                msg_gazebo->linear.y = high_velPlanner_linear_y.vel();
                msg_gazebo->angular.z = velPlanner_angular_z.vel();

            }

            _pub_canusb->publish(*msg_linear);
            _pub_canusb->publish(*msg_angular);
            _pub_gazebo->publish(*msg_gazebo);
        }

        void DualSense::callback_inject_calibration(const std_msgs::msg::Empty::SharedPtr msg){
            gamebtn.inject_calibration(_pub_canusb);
        }

        void DualSense::callback_emergency_state(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg){
            get_mainboard_time = std::chrono::system_clock::now();
            if(msg_base_control.is_emergency != static_cast<bool>(msg->candata[0])) {
                msg_base_control.is_emergency = static_cast<bool>(msg->candata[0]);
                _pub_base_control->publish(msg_base_control);
            }
            if(msg_base_control.is_emergency)gamebtn.led(0,_pub_canusb);
        }

        void DualSense::callback_inject_convergence(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg){
            msg_convergence.injection  = static_cast<bool>(msg->candata[0]);
        }

        void DualSense::callback_seedling_convergence(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg){
            msg_convergence.seedlinghand = static_cast<bool>(msg->candata[0]);
        }

        void DualSense::callback_paddy_convergence(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg){
            msg_convergence.ballhand = static_cast<bool>(msg->candata[0]);
        }

        void DualSense::callback_is_move_tracking(const std_msgs::msg::Bool::SharedPtr msg){
            msg_convergence.spline_convergence = msg->data;           
        }

        void DualSense::callback_calculator_convergence(const std_msgs::msg::Bool::SharedPtr msg){
            msg_convergence.injection_calculator = msg->data;
        }
       
        void DualSense::callback_move_autonomous(const std_msgs::msg::Bool::SharedPtr msg){
            msg_base_control.is_move_autonomous = msg->data;
            _pub_base_control->publish(msg_base_control);
        }
}