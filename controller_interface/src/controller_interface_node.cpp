#include "controller_interface/controller_interface_node.hpp"
#include <sys/time.h>
#include <sys/types.h>
#include <chrono>
#include <iostream>
#include <future>
#include <boost/format.hpp>

using namespace utils;

namespace controller_interface
{
    using std::string;

    //下記2文は共有ライブラリを書くのに必要なプログラム
    SmartphoneGamepad::SmartphoneGamepad(const rclcpp::NodeOptions &options) : SmartphoneGamepad("", options) {}
    SmartphoneGamepad::SmartphoneGamepad(const std::string &name_space, const rclcpp::NodeOptions &options): rclcpp::Node("controller_interface_node", name_space, options),
        
        //mainexecutorのyamlで設定したパラメータを設定している。
        //この場合はhigh_limit_linearクラスにDEL_MAX=poslimit,vel,acc,decのパラメータを引数として持たせている。
        //as_doubleは引用先のパラメータの型を示している。
        high_limit_linear(DBL_MAX,
        get_parameter("high_linear_max_vel").as_double(),
        get_parameter("high_linear_max_acc").as_double(),
        get_parameter("high_linear_max_dec").as_double() ),
        //以下の2つの関数も上記と同様の処理をしている
        slow_limit_linear(DBL_MAX,
        get_parameter("slow_linear_max_vel").as_double(),
        get_parameter("slow_linear_max_acc").as_double(),
        get_parameter("slow_linear_max_dec").as_double() ),
        limit_angular(DBL_MAX,
        dtor(get_parameter("angular_max_vel").as_double()),
        dtor(get_parameter("angular_max_acc").as_double()),
        dtor(get_parameter("angular_max_dec").as_double()) ),

        joy_main(get_parameter("port.joy_main").as_int()),

        //high_linear_max_velの型をdoubleからfloatにstatic_castを用いて変換している
        //数値を保存するだけならfloatの方がdoubleより処理が速いため
        high_manual_linear_max_vel(static_cast<float>(get_parameter("high_linear_max_vel").as_double())),
        slow_manual_linear_max_vel(static_cast<float>(get_parameter("slow_linear_max_vel").as_double())),
        manual_angular_max_vel(dtor(static_cast<float>(get_parameter("angular_max_vel").as_double()))),

        //リスタート
        defalt_restart_flag(get_parameter("defalt_restart_flag").as_bool()),
        //緊急停止
        defalt_emergency_flag(get_parameter("defalt_emergency_flag").as_bool()),
        //自動化
        defalt_move_autonomous_flag(get_parameter("defalt_move_autonomous_flag").as_bool()),
        //自動射出
        defalt_injection_autonomous_flag(get_parameter("defalt_injection_autonomous_flag").as_bool()),
        //低速モード
        defalt_slow_speed_flag(get_parameter("defalt_slow_speed_flag").as_bool()),

        //収束の確認
        defalt_spline_convergence(get_parameter("defalt_spline_convergence").as_bool()),
        defalt_injection_calculator_convergence(get_parameter("defalt_injection_calculator_convergence").as_bool()),
        defalt_injection_convergence(get_parameter("defalt_injection_convergence").as_bool()),
        defalt_seedlinghand_convergence(get_parameter("defalt_seedlinghand_convergence").as_bool()),
        defalt_ballhand_convergence(get_parameter("defalt_ballhand_convergence").as_bool()),

        //canidの取得
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
        can_motor_calibration_id(get_parameter("canid.motor_calibrattion").as_int()),
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
            //収束の状態を確認するための周期
            const auto heartbeat_ms = this->get_parameter("heartbeat_ms").as_int();
            const auto convergence_ms = this->get_parameter("convergence_ms").as_int();
            const auto base_state_communication_ms = this->get_parameter("base_state_communication_ms").as_int();
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
            gamebtn.canid.motor_calibration = can_motor_calibration_id;
            gamebtn.canid.led = can_led_id;
            
            //controller_mainからsub
            _sub_main_pad = this->create_subscription<std_msgs::msg::String>(
                "main_pad",
                _qos,
                std::bind(&SmartphoneGamepad::callback_main_pad, this, std::placeholders::_1)
            );
            _sub_screen_pad = this->create_subscription<std_msgs::msg::String>(
                "SCRN_info",
                _qos,
                std::bind(&SmartphoneGamepad::callback_screen_mainpad, this, std::placeholders::_1)
            );
            _sub_connection_state = this->create_subscription<std_msgs::msg::Empty>(
                "connection_state",
                _qos,
                std::bind(&SmartphoneGamepad::callback_connection_state, this, std::placeholders::_1)
            );

            //controller_subからsub
            _sub_pad = this->create_subscription<std_msgs::msg::String>(
                "sub_pad",
                _qos,
                std::bind(&SmartphoneGamepad::callback_subpad, this, std::placeholders::_1)
            );

            //mainからsub
            _sub_emergency_state = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
                "can_rx_"+ (boost::format("%x") % can_emergency_state_id).str(),
                _qos,
                std::bind(&SmartphoneGamepad::callback_emergency_state, this, std::placeholders::_1)
            );
            _sub_inject_convergence = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
                "can_rx_"+ (boost::format("%x") %  can_inject_convergence_id).str(),
                _qos,
                std::bind(&SmartphoneGamepad::callback_inject_convergence, this, std::placeholders::_1)
            );
            _sub_seedling_convergence = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
                "can_rx_"+ (boost::format("%x") %  can_seedling_convergence_id).str(),
                _qos,
                std::bind(&SmartphoneGamepad::callback_seedling_convergence, this, std::placeholders::_1)
            );
            _sub_paddy_convergence = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
                "can_rx_"+ (boost::format("%x") %  can_paddy_convergence_id).str(),
                _qos,
                std::bind(&SmartphoneGamepad::callback_paddy_convergence, this, std::placeholders::_1)
            );

            //spline_pidからsub
            _sub_is_move_tracking = this->create_subscription<std_msgs::msg::Bool>(
                "is_move_tracking",
                _qos,
                std::bind(&SmartphoneGamepad::callback_is_move_tracking, this, std::placeholders::_1)
            );

            //injection_param_calculatorからsub
            _sub_calculator_convergence = this->create_subscription<std_msgs::msg::Bool>(
                "calculator_convergenced_",
                _qos,
                std::bind(&SmartphoneGamepad::callback_calculator_convergence, this, std::placeholders::_1)
            );

            _sub_inject_calibration = this->create_subscription<std_msgs::msg::Empty>(
                "Injection_Calibration",
                _qos,
                std::bind(&SmartphoneGamepad::callback_inject_calibration, this, std::placeholders::_1)
            );

            _sub_motor_calibration = this->create_subscription<std_msgs::msg::Empty>(
                "motor_calibrate",
                _qos,
                std::bind(&SmartphoneGamepad::callback_motor_calibration, this, std::placeholders::_1)
            );

            _sub_move_autonomous = this->create_subscription<std_msgs::msg::Bool>(
                "move_autonomous",
                _qos,
                std::bind(&SmartphoneGamepad::callback_move_autonomous, this, std::placeholders::_1)
            );

            //canusbへpub
            _pub_canusb = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);
            //各nodeへ共有。
            _pub_base_control = this->create_publisher<controller_interface_msg::msg::BaseControl>("base_control",_qos);
            _pub_convergence = this->create_publisher<controller_interface_msg::msg::Convergence>("convergence" , _qos);
            _pub_injection_calculate = this->create_publisher<std_msgs::msg::Empty>("injection_calculate", _qos);

            //sprine_pid
            _pub_target_node = this->create_publisher<std_msgs::msg::String>("target_node", _qos);

            //gazebo用のpub
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

            _socket_timer = this->create_wall_timer(
                std::chrono::milliseconds(this->get_parameter("interval_ms").as_int()),
                [this] { _recv_callback(); }
            );

            if(connection_check){
                check_controller_connection = this->create_wall_timer(
                    std::chrono::milliseconds(static_cast<int>(controller_ms)),
                    [this] {
                        std::chrono::system_clock::time_point now_time = std::chrono::system_clock::now();
                        if(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - get_controller_time).count() > 100 * 10){
                            auto msg_emergency = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                            msg_emergency->canid = can_emergency_id;
                            msg_emergency->candlc = 1;
                            msg_emergency->candata[0] = 1;
                            _pub_canusb->publish(*msg_emergency);
                            RCLCPP_INFO(get_logger(),"controller_connection_lost!!");
                        }
                    }
                );

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

        void SmartphoneGamepad::callback_main_pad(const std_msgs::msg::String::SharedPtr msg)    {

            uint8_t _candata_btn[8];
            msg_base_control.is_restart = false;

            if(msg->data == "g"){
                cout<<"emergency"<<endl;
                auto msg_emergency = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                msg_emergency->canid = can_emergency_id;
                msg_emergency->candlc = 1;
                msg_emergency->candata[0] = true;
                _pub_canusb->publish(*msg_emergency);
            }
            else if(msg->data == "s"){
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
            else if(msg->data == "a"&&arm_expansion_flag) gamebtn.seedling_collect_right(msg_convergence.seedlinghand,_pub_canusb);          
            else if(msg->data == "b"&&arm_expansion_flag) gamebtn.seedling_collect_left(msg_convergence.seedlinghand,_pub_canusb);               
            else if(msg->data == "x"&&arm_expansion_flag) gamebtn.seedling_install_right(msg_convergence.seedlinghand,_pub_canusb);   
            else if(msg->data == "y"&&arm_expansion_flag) gamebtn.seedling_install_left(msg_convergence.seedlinghand,_pub_canusb);
            else if(msg->data == "up") gamebtn.steer_reset(_pub_canusb);
            else if(msg->data == "down") gamebtn.calibrate(_pub_canusb);                
            else if(msg->data == "left") gamebtn.board_reset(_pub_canusb);
            else if(msg->data == "right") {
                auto msg_is_start = std::make_shared<std_msgs::msg::UInt8>();
                msg_is_start->data = 0;
                _pub_is_start->publish(*msg_is_start);
            }
            else if(msg->data == "r1") gamebtn.injection(msg_convergence.injection ,msg_convergence.injection_calculator,_pub_canusb); 
            else if(msg->data == "r2") gamebtn.injection_calculate(_pub_injection_calculate);
            else if(msg->data == "r3"){
                msg_base_control.is_move_autonomous = !msg_base_control.is_move_autonomous;
                _pub_base_control->publish(msg_base_control);
                 if(msg_base_control.is_move_autonomous) gamebtn.led(3,_pub_canusb);
            }
            else if(msg->data == "l1") gamebtn.paddy_control(msg_convergence.ballhand,_pub_canusb); 
            else if(msg->data == "l2") {
                msg_base_control.is_slow_speed = !msg_base_control.is_slow_speed;
                _pub_base_control->publish(msg_base_control);
                if(!msg_base_control.is_move_autonomous && !msg_base_control.is_slow_speed) gamebtn.led(1,_pub_canusb);
                else if(!msg_base_control.is_move_autonomous && msg_base_control.is_slow_speed)gamebtn.led(2,_pub_canusb);
            }
            else if(msg->data == "l3") {
                    gamebtn.arm_expansion(_pub_canusb);
                    arm_expansion_flag = true;
            }
        }

        void SmartphoneGamepad::callback_screen_mainpad(const std_msgs::msg::String::SharedPtr msg){
            if(msg->data.length() <= 3){
                auto msg_target_node = std::make_shared<std_msgs::msg::String>();
                msg_target_node->data = msg->data;
                _pub_target_node->publish(*msg_target_node);
            } 
        }

        void SmartphoneGamepad::callback_screen_subpad(const std_msgs::msg::String::SharedPtr msg){

        }

        void SmartphoneGamepad::callback_inject_calibration(const std_msgs::msg::Empty::SharedPtr msg){
            gamebtn.inject_calibration(_pub_canusb);
        }

        void SmartphoneGamepad::callback_motor_calibration(const std_msgs::msg::Empty::SharedPtr msg){
            gamebtn.motor_calibration(_pub_canusb);
        }

        void SmartphoneGamepad::callback_subpad(const std_msgs::msg::String::SharedPtr msg){

        }

        void SmartphoneGamepad::callback_connection_state(const std_msgs::msg::Empty::SharedPtr msg){
            get_controller_time = std::chrono::system_clock::now();
        }

        void SmartphoneGamepad::callback_emergency_state(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg){
            get_mainboard_time = std::chrono::system_clock::now();
            if(msg_base_control.is_emergency != static_cast<bool>(msg->candata[0])) {
                msg_base_control.is_emergency = static_cast<bool>(msg->candata[0]);
                _pub_base_control->publish(msg_base_control);
            }
            if(msg_base_control.is_emergency)gamebtn.led(0,_pub_canusb);
        }

        //射出情報をsubsclib
        void SmartphoneGamepad::callback_inject_convergence(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg){
            msg_convergence.injection  = static_cast<bool>(msg->candata[0]);
        }

        void SmartphoneGamepad::callback_seedling_convergence(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg){
            msg_convergence.seedlinghand = static_cast<bool>(msg->candata[0]);
        }

        void SmartphoneGamepad::callback_paddy_convergence(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg){
            msg_convergence.ballhand = static_cast<bool>(msg->candata[0]);
        }

        //splineをsubsclib
        void SmartphoneGamepad::callback_is_move_tracking(const std_msgs::msg::Bool::SharedPtr msg){
            msg_convergence.spline_convergence = msg->data;           
        }

        //injection_param_calculatorをsubscribe
        void SmartphoneGamepad::callback_calculator_convergence(const std_msgs::msg::Bool::SharedPtr msg){
            msg_convergence.injection_calculator = msg->data;
        }

        void SmartphoneGamepad::callback_move_autonomous(const std_msgs::msg::Bool::SharedPtr msg){
            msg_base_control.is_move_autonomous = msg->data;
            _pub_base_control->publish(msg_base_control);
        }

        //スティックの値をsubscribしている
        void SmartphoneGamepad::_recv_callback(){
            if(joy_main.is_recved()){
                unsigned char data[16];
                _recv_joy_main(joy_main.data(data, sizeof(data)));
            }
        }

        //ジョイスティックの値
        void SmartphoneGamepad::_recv_joy_main(const unsigned char data[16]){

            //手動モード
            if(msg_base_control.is_move_autonomous == false){
                float values[4];
                //メモリをコピー
                memcpy(values, data, sizeof(float)*4);
                auto msg_linear = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                msg_linear->canid = can_linear_id;
                msg_linear->candlc = 8;

                auto msg_angular = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                msg_angular->canid = can_angular_id;
                msg_angular->candlc = 4;
                
                auto msg_gazebo = std::make_shared<geometry_msgs::msg::Twist>();
                
                uint8_t _candata_joy[8];

                //低速モード
                if(msg_base_control.is_slow_speed== true){

                    slow_velPlanner_linear_x.vel(static_cast<double>(values[1]));//unityとロボットにおける。xとyが違うので逆にしている。
                    slow_velPlanner_linear_y.vel(static_cast<double>(-values[0]));
                    velPlanner_angular_z.vel(static_cast<double>(-values[2]));

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
                //高速モード
                else{
                    high_velPlanner_linear_x.vel(static_cast<double>(values[1]));//unityとロボットにおける。xとyが違うので逆にしている。
                    high_velPlanner_linear_y.vel(static_cast<double>(-values[0]));
                    velPlanner_angular_z.vel(static_cast<double>(-values[2]));

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
        }
}