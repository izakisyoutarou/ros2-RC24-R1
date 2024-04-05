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
        //ボール
        defalt_color_information_flag(get_parameter("defalt_color_information_flag").as_bool()),

        //収束の確認
        defalt_spline_convergence(get_parameter("defalt_spline_convergence").as_bool()),

        defalt_injection_calculator_convergence(get_parameter("defalt_injection_calculator_convergence").as_bool()),
        //射出のyaw軸が目標の値まで移動して停止した状態を保存するための変数
        defalt_injection_convergence(get_parameter("defalt_injection_convergence").as_bool()),
        //苗ハンドの収束状況を保存するための変数
        defalt_seedlinghand_convergence(get_parameter("defalt_seedlinghand_convergence").as_bool()),
        //ボール回収ハンドの収束状況を保存するための変数
        defalt_ballhand_convergence(get_parameter("defalt_ballhand_convergence").as_bool()),
        //通信系
        udp_port_state(get_parameter("port.robot_state").as_int()),
        udp_port_pole(get_parameter("port.pole_share").as_int()),
        udp_port_spline_state(get_parameter("port.spline_state").as_int()),

        //can通信とは基盤に刺さっている2つの通信専用のピンの電位差で通信する方式
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
        can_main_button_id(get_parameter("canid.main_digital_button").as_int()),
        can_sub_button_id(get_parameter("canid.sub_digital_button").as_int()),
        can_inject_id(get_parameter("canid.inject").as_int()),
        can_inject_spinning_id(get_parameter("canid.inject_spinning").as_int()),
        can_inject_convergence_id(get_parameter("canid.inject_convergence").as_int()),
        can_inject_calibration_id(get_parameter("canid.inject_calibration").as_int()),
        can_seedling_collect_id(get_parameter("canid.seedling_collect").as_int()),
        can_seedling_install_id(get_parameter("canid.seedling_install").as_int()),
        can_seedling_convergence_id(get_parameter("canid.seedling_convergence").as_int()),
        can_paddy_collect_id(get_parameter("canid.paddy_collect").as_int()),
        can_paddy_install_id(get_parameter("canid.paddy_install").as_int()),
        can_paddy_convergence_id(get_parameter("canid.paddy_convergence").as_int()),
        can_arm_expansion_id(get_parameter("canid.arm_expansion").as_int()),
        
        //回収、射出機構のはじめの位置の値を取得
        initial_pickup_state(get_parameter("initial_pickup_state").as_string()),
        initial_inject_state(get_parameter("initial_inject_state").as_string())

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
            gamebtn.canid.inject_spinning = can_inject_spinning_id;
            gamebtn.canid.paddy_collect = can_paddy_collect_id;
            gamebtn.canid.paddy_install = can_paddy_install_id;
            gamebtn.canid.seedling_collect = can_seedling_collect_id;
            gamebtn.canid.seedling_install = can_seedling_install_id;
            gamebtn.canid.arm_expansion = can_arm_expansion_id;
            gamebtn.canid.inject_calibration = can_inject_calibration_id;
            
            //controller_mainからsub
            _sub_main_pad = this->create_subscription<std_msgs::msg::String>(
                "main_pad",
                _qos,
                std::bind(&SmartphoneGamepad::callback_mainpad, this, std::placeholders::_1)
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
            _sub_obj_color = this->create_subscription<std_msgs::msg::String>(
                "obj_color",
                _qos,
                std::bind(&SmartphoneGamepad::callback_screen_subpad, this, std::placeholders::_1)
            );
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

            //canusbへpub
            _pub_canusb = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);
            //各nodeへ共有。
            _pub_base_control = this->create_publisher<controller_interface_msg::msg::BaseControl>("base_control",_qos);
            _pub_convergence = this->create_publisher<controller_interface_msg::msg::Convergence>("convergence" , _qos);
            _pub_color_information = this->create_publisher<controller_interface_msg::msg::Colorball>("color_information", _qos);
            _pub_injection_calculate = this->create_publisher<std_msgs::msg::Empty>("injection_calculate", _qos);

            //sprine_pid
            pub_move_node = this->create_publisher<std_msgs::msg::String>("move_node", _qos);
            //gazebo用のpub
            _pub_gazebo = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", _qos);
            //sequenserへ
            _pub_initial_sequense = this->create_publisher<std_msgs::msg::String>("initial_sequense", _qos);
            //ボールと苗の回収&設置
            _pub_seedling_collection = this->create_publisher<std_msgs::msg::Bool>("Seedling_Collection", _qos);
            _pub_seedling_installation = this->create_publisher<std_msgs::msg::Bool>("Seedling_Installation", _qos);
            _pub_ball_collection = this->create_publisher<std_msgs::msg::Bool>("Ball_Collection", _qos);
            
            _pub_inject_info = this->create_publisher<std_msgs::msg::Bool>("inject_info", _qos);

            //base_controlのmsgを宣言
            auto msg_base_control = std::make_shared<controller_interface_msg::msg::BaseControl>();
            msg_base_control->is_restart = defalt_restart_flag;
            msg_base_control->is_emergency = defalt_emergency_flag;
            msg_base_control->is_move_autonomous = defalt_move_autonomous_flag;
            msg_base_control->is_injection_autonomous = defalt_injection_autonomous_flag;
            msg_base_control->is_slow_speed = defalt_slow_speed_flag;
            msg_base_control->initial_state = "O";
            _pub_base_control->publish(*msg_base_control);

            auto msg_emergency = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_emergency->canid = can_emergency_id;
            msg_emergency->candlc = 1;
            msg_emergency->candata[0] = defalt_emergency_flag;
            _pub_canusb->publish(*msg_emergency);

            auto msg_convergence = std::make_shared<controller_interface_msg::msg::Convergence>();
            msg_convergence->spline_convergence = defalt_spline_convergence;
            msg_convergence->injection_calculator = defalt_injection_calculator_convergence;
            msg_convergence->injection = defalt_injection_convergence;
            msg_convergence->seedlinghand = defalt_seedlinghand_convergence;
            msg_convergence->ballhand = defalt_ballhand_convergence;
            _pub_convergence->publish(*msg_convergence);

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
                    auto msg_convergence = std::make_shared<controller_interface_msg::msg::Convergence>();
                    msg_convergence->spline_convergence = is_spline_convergence;
                    msg_convergence->injection_calculator = is_injection_calculator_convergence;
                    msg_convergence->injection = is_injection_convergence;
                    msg_convergence->seedlinghand = is_seedlinghand_convergence;
                    msg_convergence->ballhand = is_ballhand_convergence;
                    _pub_convergence->publish(*msg_convergence);
                }
            );

            _socket_timer = this->create_wall_timer(
                std::chrono::milliseconds(this->get_parameter("interval_ms").as_int()),
                [this] { _recv_callback(); }
            );

            _start_timer = this->create_wall_timer(
                std::chrono::milliseconds(this->get_parameter("start_ms").as_int()),
                [this] {
                    auto initial_sequense_injection = std::make_shared<std_msgs::msg::String>();
                    initial_sequense_injection->data = initial_inject_state;
                    _pub_initial_sequense->publish(*initial_sequense_injection);
                }
            );

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
                    if(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - get_mainboard_time).count() > 200 * 10){
                        is_emergency = true;
                        is_restart = false; 
                        auto msg_base_control = std::make_shared<controller_interface_msg::msg::BaseControl>();   
                        msg_base_control->is_restart = is_restart;
                        msg_base_control->is_emergency = is_emergency;
                        msg_base_control->is_move_autonomous = is_move_autonomous;
                        msg_base_control->is_slow_speed = is_slow_speed;
                        msg_base_control->initial_state = initial_state;
                        _pub_base_control->publish(*msg_base_control);
                        RCLCPP_INFO(get_logger(),"mainboard_connection_lost!!");
                    }
                }
            );

            //計画機にリミットを設定する
            high_velPlanner_linear_x.limit(high_limit_linear);
            high_velPlanner_linear_y.limit(high_limit_linear);

            slow_velPlanner_linear_x.limit(slow_limit_linear);
            slow_velPlanner_linear_y.limit(slow_limit_linear);

            velPlanner_angular_z.limit(limit_angular);
        }

        void SmartphoneGamepad::callback_mainpad(const std_msgs::msg::String::SharedPtr msg)    {
            //リスタート
            auto msg_restart = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_restart->canid = can_restart_id;
            msg_restart->candlc = 0;

            //緊急停止
            auto msg_emergency = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_emergency->canid = can_emergency_id;
            msg_emergency->candlc = 1;

            //ボタン
            auto msg_btn = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_btn->canid = can_main_button_id;
            msg_btn->candlc = 8;

            uint8_t _candata_btn[8];
            bool flag_restart = false;
            is_restart = false;

            if(msg->data == "g"){
                cout<<"emergency"<<endl;
                robotcontrol_flag = true;
                msg_emergency->candata[0] = true;
            }
            else if(msg->data == "s"){
                cout<<"restart"<<endl;
                robotcontrol_flag = true;
                flag_restart = true;
                is_emergency = false;
                is_restart = true;
                is_injection_mech_stop_m = true;
                is_move_autonomous = defalt_move_autonomous_flag;
                is_injection_autonomous = defalt_injection_autonomous_flag;
                is_slow_speed = defalt_slow_speed_flag;
                initial_state = "O";
                is_spline_convergence = defalt_spline_convergence;
                is_injection_calculator_convergence = defalt_injection_calculator_convergence;
                is_injection_convergence = defalt_injection_convergence;
                is_seedlinghand_convergence = defalt_seedlinghand_convergence;
                is_ballhand_convergence = defalt_ballhand_convergence;                
            }
            else if(msg->data == "a") gamebtn.seedling_collect_right(is_seedlinghand_convergence,_pub_canusb);          
            else if(msg->data == "b") gamebtn.seedling_collect_left(is_seedlinghand_convergence,_pub_canusb);               
            else if(msg->data == "x") gamebtn.seedling_install_right(is_seedlinghand_convergence,_pub_canusb);   
            else if(msg->data == "y") gamebtn.seedling_install_left(is_seedlinghand_convergence,_pub_canusb);   
            else if(msg->data == "up") gamebtn.steer_reset(_pub_canusb);
            else if(msg->data == "down") gamebtn.calibrate(_pub_canusb);                
            else if(msg->data == "left") gamebtn.board_reset(_pub_canusb);
            // else if(msg->data == "right") //sequence
            else if(msg->data == "r1") gamebtn.injection(is_injection_convergence,is_injection_calculator_convergence,_pub_canusb); 
            else if(msg->data == "r2") gamebtn.injection_calculate(_pub_injection_calculate);
            else if(msg->data == "r3"){
                robotcontrol_flag = true;
                if(is_move_autonomous == false){
                    is_move_autonomous = true;
                    is_injection_autonomous = true;
                }
                else{
                    is_move_autonomous = false;
                    is_injection_autonomous = false;
                }
            }
            else if(msg->data == "l1") gamebtn.paddy_control(is_ballhand_convergence,_pub_canusb); 
            else if(msg->data == "l2") is_slow_speed = !is_slow_speed;
            else if(msg->data == "l3") gamebtn.arm_expansion(_pub_canusb);

            //base_controlへ代入
            msg_base_control.is_restart = is_restart;
            msg_base_control.is_emergency = is_emergency;
            msg_base_control.is_move_autonomous = is_move_autonomous;
            msg_base_control.is_injection_autonomous = is_injection_autonomous;
            msg_base_control.is_slow_speed = is_slow_speed;
            msg_base_control.initial_state = initial_state;
            msg_base_control.is_injection_mech_stop_m = is_injection_mech_stop_m;

            if(msg->data=="g") _pub_canusb->publish(*msg_emergency);

            if(robotcontrol_flag) _pub_base_control->publish(msg_base_control);

            if(msg->data == "s"){
                _pub_canusb->publish(*msg_restart);
                _pub_canusb->publish(*msg_emergency);
            }
        }

        void SmartphoneGamepad::callback_screen_mainpad(const std_msgs::msg::String::SharedPtr msg){
            if(msg->data.length() <= 3){
                move_node = msg->data;
                auto msg_move_node = std::make_shared<std_msgs::msg::String>();
                msg_move_node->data = msg->data;
                pub_move_node->publish(*msg_move_node);
            } 
            else if(msg->data == "Seedling_Collection" || msg->data == "Seedling_Installation" || msg->data == "ball_Collection"){
                auto msg_move_node_bool = std::make_shared<std_msgs::msg::Bool>();
                msg_move_node_bool->data = true;
                if(msg->data == "Seedling_Collection") _pub_seedling_collection->publish(*msg_move_node_bool);
                else if(msg->data == "Seedling_Installation") _pub_seedling_installation->publish(*msg_move_node_bool);
                else if(msg->data == "ball_Collection") _pub_ball_collection->publish(*msg_move_node_bool);
            }
        }

        void SmartphoneGamepad::callback_screen_subpad(const std_msgs::msg::String::SharedPtr msg){
            if(msg->data == "Btn_info_msg") _pub_color_information->publish(msg_color_information);
            else {
                for(int i = 0; i < 12; i++){
                    if(msg->data.at(0) == head_english[i]){
                        if(msg->data.find("red") != -1) {
                            msg_color_information.color_info[i] = true;
                            color_data[i] = true;
                        }
                        else if(msg->data.find("blue") != -1) {
                            msg_color_information.color_info[i] = true;
                            color_data[i] = true;
                        }
                        else if(msg->data.find("purple") != -1) {
                            msg_color_information.color_info[i] = false;
                            color_data[i] = false;
                        }
                        break;
                    }
                }
            }
        }

        void SmartphoneGamepad::callback_inject_calibration(const std_msgs::msg::Empty::SharedPtr msg){
            gamebtn.inject_calibration(_pub_canusb);
        }

        void SmartphoneGamepad::callback_subpad(const std_msgs::msg::String::SharedPtr msg){

        }

        //スタート地点情報をsubscribe
        void SmartphoneGamepad::callback_initial_state(const std_msgs::msg::String::SharedPtr msg){
            initial_state = msg->data[0];
        }


        void SmartphoneGamepad::callback_connection_state(const std_msgs::msg::Empty::SharedPtr msg){
            get_controller_time = std::chrono::system_clock::now();
        }

        void SmartphoneGamepad::callback_emergency_state(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg){
            get_mainboard_time = std::chrono::system_clock::now();
            if(is_emergency != static_cast<bool>(msg->candata[0])) {
                is_emergency = static_cast<bool>(msg->candata[0]);
                is_restart = false; 
                msg_base_control.is_restart = is_restart;
                msg_base_control.is_emergency = is_emergency;
                msg_base_control.is_move_autonomous = is_move_autonomous;
                msg_base_control.is_slow_speed = is_slow_speed;
                msg_base_control.initial_state = initial_state;
                _pub_base_control->publish(msg_base_control);
            }
        }

        //射出情報をsubsclib
        void SmartphoneGamepad::callback_inject_convergence(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg){
            is_injection_convergence = static_cast<bool>(msg->candata[0]);
        }

        void SmartphoneGamepad::callback_seedling_convergence(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg){
            is_seedlinghand_convergence = static_cast<bool>(msg->candata[1]);
        }

        void SmartphoneGamepad::callback_paddy_convergence(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg){
            is_ballhand_convergence = static_cast<bool>(msg->candata[2]);
        }

        //splineをsubsclib
        void SmartphoneGamepad::callback_is_move_tracking(const std_msgs::msg::Bool::SharedPtr msg){
            is_spline_convergence = msg->data;           
        }

        //injection_param_calculatorをsubscribe
        void SmartphoneGamepad::callback_calculator_convergence(const std_msgs::msg::Bool::SharedPtr msg){
             //injection_calculatorから上モノ指令値計算収束のsub。上物の指令値の収束情報。
            is_injection_calculator_convergence = msg->data;
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
            if(is_move_autonomous == false){
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

                bool flag_move_autonomous = false;
                
                uint8_t _candata_joy[8];
                //低速モード
                if(is_slow_speed == true){

                    slow_velPlanner_linear_x.vel(static_cast<double>(values[1]));//unityとロボットにおける。xとyが違うので逆にしている。
                    slow_velPlanner_linear_y.vel(static_cast<double>(-values[0]));
                    velPlanner_angular_z.vel(static_cast<double>(-values[2]));
                    //演算処理
                    slow_velPlanner_linear_x.cycle();
                    slow_velPlanner_linear_y.cycle();
                    velPlanner_angular_z.cycle();
                    //floatからバイト(メモリ)に変換
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