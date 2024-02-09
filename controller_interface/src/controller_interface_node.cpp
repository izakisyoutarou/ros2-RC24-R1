#include "controller_interface/controller_interface_node.hpp"
#include <sys/time.h>
#include <sys/types.h>
#include <chrono>
#include <iostream>
#include <future>

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

        //リスタートのパラメータ取得
        defalt_restart_flag(get_parameter("defalt_restart_flag").as_bool()),
        //緊急停止のパラメータを取得
        defalt_emergency_flag(get_parameter("defalt_emergency_flag").as_bool()),
        //自動化のパラメータを取得
        defalt_move_autonomous_flag(get_parameter("defalt_move_autonomous_flag").as_bool()),
        //自動射出パラメータを取得
        defalt_injection_autonomous_flag(get_parameter("defalt_injection_autonomous_flag").as_bool()),
        //低速モードのパラメータを取得
        defalt_slow_speed_flag(get_parameter("defalt_slow_speed_flag").as_bool()),
        //ボールの色情報を取得
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
        can_linear_id(get_parameter("canid.linear").as_int()),
        can_angular_id(get_parameter("canid.angular").as_int()),
        can_main_button_id(get_parameter("canid.main_digital_button").as_int()),
        can_sub_button_id(get_parameter("canid.sub_digital_button").as_int()),
        can_inject_id(get_parameter("canid.inject").as_int()),
        can_inject_spinning_id(get_parameter("canid.inject_spinning").as_int()),
        can_seedling_collect_id(get_parameter("canid.seedling_collect").as_int()),
        can_seedling_install_id(get_parameter("canid.seedling_install").as_int()),
        can_paddy_collect_id(get_parameter("canid.paddy_collect").as_int()),
        can_paddy_install_id(get_parameter("canid.paddy_install").as_int()),
        can_steer_reset_id(get_parameter("canid.steer_reset").as_int()),
        can_reset_id(get_parameter("canid.reset").as_int()),

        //ipアドレスの取得
        r1_pc(get_parameter("ip.r1_pc").as_string()),
        r2_pc(get_parameter("ip.r2_pc").as_string()),

        //回収、射出機構のはじめの位置の値を取得
        initial_pickup_state(get_parameter("initial_pickup_state").as_string()),
        initial_inject_state(get_parameter("initial_inject_state").as_string())

        {
            //収束の状態を確認するための周期
            const auto heartbeat_ms = this->get_parameter("heartbeat_ms").as_int();
            const auto convergence_ms = this->get_parameter("convergence_ms").as_int();
            const auto base_state_communication_ms = this->get_parameter("base_state_communication_ms").as_int();

            //controller_mainからsub
            _sub_main_pad = this->create_subscription<std_msgs::msg::String>(
                "main_pad",
                _qos,
                std::bind(&SmartphoneGamepad::callback_main_pad, this, std::placeholders::_1)
            );

            _sub_screen_pad = this->create_subscription<std_msgs::msg::String>(
                "SCRN_info",
                _qos,
                std::bind(&SmartphoneGamepad::callback_screen_pad, this, std::placeholders::_1)
            );

            //controller_subからsub
            _sub_pad = this->create_subscription<std_msgs::msg::String>(
                "sub_pad",
                _qos,
                std::bind(&SmartphoneGamepad::callback_sub_pad, this, std::placeholders::_1)
            );
            _sub_gamepad = this->create_subscription<std_msgs::msg::String>(
                "sub_gamepad",
                _qos,
                std::bind(&SmartphoneGamepad::callback_sub_gamepad, this, std::placeholders::_1)
            );

            //mainからsub
            _sub_main_injection_possible = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
                "can_rx_204",
                _qos,
                std::bind(&SmartphoneGamepad::callback_main_injection_possible, this, std::placeholders::_1)
            );
            _sub_main_Seedlinghand_possible = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
                "can_rx_212",
                _qos,
                std::bind(&SmartphoneGamepad::callback_main_Seedlinghand_possible, this, std::placeholders::_1)
            );
            _sub_main_ballhand_possible = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
                "can_rx_222",
                _qos,
                std::bind(&SmartphoneGamepad::callback_main_ballhand_possible, this, std::placeholders::_1)
            );
            //spline_pidからsub
            _sub_spline = this->create_subscription<std_msgs::msg::Bool>(
                "is_move_tracking",
                _qos,
                std::bind(&SmartphoneGamepad::callback_spline, this, std::placeholders::_1)
            );

            //injection_param_calculatorからsub
            _sub_injection_calculator = this->create_subscription<std_msgs::msg::Bool>(
                "calculator_convergenced_",
                _qos,
                std::bind(&SmartphoneGamepad::callback_injection_calculator, this, std::placeholders::_1)
            );

            //canusbへpub
            //txは送信でrxは受信
            _pub_canusb = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);
            //injectionへpub
            //各nodeへ共有。
            _pub_base_control = this->create_publisher<controller_interface_msg::msg::BaseControl>("base_control",_qos);
            _pub_convergence = this->create_publisher<controller_interface_msg::msg::Convergence>("convergence" , _qos);
            _pub_color_ball = this->create_publisher<controller_interface_msg::msg::Colorball>("color_information", _qos);
            _pub_injection = this->create_publisher<std_msgs::msg::Bool>("is_backside", _qos);
            //sprine_pid
            pub_move_node = this->create_publisher<std_msgs::msg::String>("move_node", _qos);
            //gazebo用のpub
            _pub_gazebo = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", _qos);
            //sequenserへ
            _pub_initial_sequense = this->create_publisher<std_msgs::msg::String>("initial_sequense", _qos);

            _pub_initial_state = this->create_publisher<std_msgs::msg::String>("initial_state_unity", _qos);
            _pub_base_restart = this->create_publisher<std_msgs::msg::Bool>("restart_unity", _qos);
            _pub_base_emergency = this->create_publisher<std_msgs::msg::Bool>("emergency_unity", _qos);
            _pub_move_auto = this->create_publisher<std_msgs::msg::Bool>("move_autonomous_unity", _qos);
            _pub_base_injection = this->create_publisher<std_msgs::msg::Bool>("injection_autonomous_unity", _qos);

            _pub_con_spline = this->create_publisher<std_msgs::msg::Bool>("spline_convergence_unity", _qos);
            _pub_con_colcurator = this->create_publisher<std_msgs::msg::Bool>("injection_calcurator_unity", _qos);
            _pub_con_injection = this->create_publisher<std_msgs::msg::Bool>("injection_convergence_unity", _qos);
            _pub_con_seedlinghand = this->create_publisher<std_msgs::msg::Bool>("seedlinghand_convergence_unity", _qos);
            _pub_con_ballhand = this->create_publisher<std_msgs::msg::Bool>("ballhand_convergence_unity", _qos);
            _pub_base_state_communication = this->create_publisher<std_msgs::msg::Empty>("state_communication_unity", _qos);

            //ボールと苗の回収&設置
            _pub_seedling_collection = this->create_publisher<std_msgs::msg::Bool>("Seedling_Collection", _qos);
            _pub_seedling_installation = this->create_publisher<std_msgs::msg::Bool>("Seedling_Installation", _qos);
            _pub_ball_collection = this->create_publisher<std_msgs::msg::Bool>("Ball_Collection", _qos);

            //base_controlのmsgを宣言
            auto msg_base_control = std::make_shared<controller_interface_msg::msg::BaseControl>();
            msg_base_control->is_restart = defalt_restart_flag;
            msg_base_control->is_emergency = defalt_emergency_flag;
            msg_base_control->is_move_autonomous = defalt_move_autonomous_flag;
            msg_base_control->is_injection_autonomous = defalt_injection_autonomous_flag;
            msg_base_control->is_slow_speed = defalt_slow_speed_flag;
            msg_base_control->initial_state = "O";
            this->is_reset = defalt_restart_flag;
            this->is_emergency = defalt_emergency_flag;
            this->is_move_autonomous = defalt_move_autonomous_flag;
            this->is_injection_autonomous = defalt_injection_autonomous_flag;
            this->is_slow_speed = defalt_slow_speed_flag;
            this->initial_state = "O";
            _pub_base_control->publish(*msg_base_control);

            //コンストラクタ限定
            auto msg_injection_con = std::make_shared<std_msgs::msg::Bool>();
            msg_injection_con->data = injection_flag;
            _pub_injection->publish(*msg_injection_con);

            auto msg_unity_control = std::make_shared<std_msgs::msg::Bool>();
            msg_unity_control->data = is_reset;
            _pub_base_restart->publish(*msg_unity_control);

            msg_unity_control->data = is_emergency;
            _pub_base_emergency->publish(*msg_unity_control);

            msg_unity_control->data = is_move_autonomous;
            _pub_move_auto->publish(*msg_unity_control);

            msg_unity_control->data = is_injection_autonomous;
            _pub_base_injection->publish(*msg_unity_control);

            auto msg_emergency = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            //get_parametorで取得したパラメータをrc24pkgsのmsgに格納
            msg_emergency->canid = can_emergency_id;
            //trueかfalseなので使用するバイトは1つ
            msg_emergency->candlc = 1;
            msg_emergency->candata[0] = defalt_emergency_flag;
            _pub_canusb->publish(*msg_emergency);

            //収束を確認するmsgの宣言
            auto msg_convergence = std::make_shared<controller_interface_msg::msg::Convergence>();
            //get_parametorで取得したパラメータをrc24pkgsのmsgに格納
            msg_convergence->spline_convergence = defalt_spline_convergence;
            msg_convergence->injection_calculator = defalt_injection_calculator_convergence;
            msg_convergence->injection = defalt_injection_convergence;
            msg_convergence->seedlinghand = defalt_seedlinghand_convergence;
            msg_convergence->ballhand = defalt_ballhand_convergence;

            this->spline_convergence = defalt_spline_convergence;
            this->injection_calculator = defalt_injection_calculator_convergence;
            this->injection = defalt_injection_convergence;
            this->seedlinghand = defalt_seedlinghand_convergence;
            this->ballhand = defalt_ballhand_convergence;

            _pub_convergence->publish(*msg_convergence);

            msg_unity_control->data = spline_convergence;
            _pub_con_spline->publish(*msg_unity_control);

            msg_unity_control->data = injection_calculator;
            _pub_con_colcurator->publish(*msg_unity_control);

            msg_unity_control->data = seedlinghand;
            _pub_con_seedlinghand->publish(*msg_unity_control);

            msg_unity_control->data = ballhand;
            _pub_con_ballhand->publish(*msg_unity_control);

            //ハートビート
            //コントローラの鼓動
            //一定周期で処理をしている。この場合は100ms間隔で処理をしている
            _pub_heartbeat = this->create_wall_timer(
                std::chrono::milliseconds(heartbeat_ms),
                [this] {
                    auto msg_heartbeat = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                    //get_parametorで取得したパラメータをrc24pkgsのmsgに格納
                    msg_heartbeat->canid = can_heartbeat_id;
                    msg_heartbeat->candlc = 0;
                    _pub_canusb->publish(*msg_heartbeat);
                }
            );
            //スマホコントローラとの通信状況を確認
            _pub_state_communication_timer = create_wall_timer(
                std::chrono::milliseconds(base_state_communication_ms),
                [this] {
                    auto msg_base_state_communication = std::make_shared<std_msgs::msg::Empty>();
                    _pub_base_state_communication->publish(*msg_base_state_communication);
                }
            );

            //convergence
            //収束状況
            //一定周期で処理をしている。この場合は100ms間隔で処理をしている
            _pub_timer_convergence = this->create_wall_timer(
                std::chrono::milliseconds(convergence_ms),
                [this] {
                    auto msg_convergence = std::make_shared<controller_interface_msg::msg::Convergence>();
                    //get_parametorで取得したパラメータをrc24pkgsのmsgに格納
                    msg_convergence->spline_convergence = is_spline_convergence;
                    msg_convergence->injection_calculator = is_injection_calculator_convergence;
                    msg_convergence->injection = is_injection_convergence;
                    msg_convergence->seedlinghand = is_seedlinghand_convergence;
                    msg_convergence->ballhand = is_ballhand_convergence;
                    _pub_convergence->publish(*msg_convergence);

                    auto msg_unity_control = std::make_shared<std_msgs::msg::Bool>();

                    msg_unity_control->data = is_spline_convergence;
                    _pub_con_spline->publish(*msg_unity_control);

                    msg_unity_control->data = is_injection_calculator_convergence;
                    _pub_con_colcurator->publish(*msg_unity_control);

                    msg_unity_control->data = is_injection_convergence;
                    _pub_con_injection->publish(*msg_unity_control);

                    msg_unity_control->data = is_seedlinghand_convergence;
                    _pub_con_seedlinghand->publish(*msg_unity_control);

                    msg_unity_control->data = is_ballhand_convergence;;
                    _pub_con_ballhand->publish(*msg_unity_control);

                }
            );

            //一定周期で処理をしている。この場合は50ms間隔で処理をしている
            //コントローラのデータを一定周期で届いているか確認する
            //UDP通信特有の書き方？
            _socket_timer = this->create_wall_timer(
                std::chrono::milliseconds(this->get_parameter("interval_ms").as_int()),
                [this] { _recv_callback(); }
            );

            //一定周期で処理をしている。この場合は3000ms間隔で処理をしている
            _start_timer = this->create_wall_timer(
                std::chrono::milliseconds(this->get_parameter("start_ms").as_int()),
                [this] {
                    auto initial_sequense_injection = std::make_shared<std_msgs::msg::String>();
                    initial_sequense_injection->data = initial_inject_state;
                    _pub_initial_sequense->publish(*initial_sequense_injection);
                }
            );

            //計画機にリミットを設定する
            high_velPlanner_linear_x.limit(high_limit_linear);
            high_velPlanner_linear_y.limit(high_limit_linear);

            slow_velPlanner_linear_x.limit(slow_limit_linear);
            slow_velPlanner_linear_y.limit(slow_limit_linear);

            velPlanner_angular_z.limit(limit_angular);
        }

        void SmartphoneGamepad::callback_main_pad(const std_msgs::msg::String::SharedPtr msg)
        {
            //リスタートの変数宣言
            //msg_restartにリスタートする際のcanidとcandlcのパラメータを格納
            auto msg_restart = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_restart->canid = can_restart_id;
            msg_restart->candlc = 0;

            //緊急停止の変数宣言
            //msg_emergencyにリスタートする際のcanidとcandlcのパラメータを格納
            auto msg_emergency = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_emergency->canid = can_emergency_id;
            msg_emergency->candlc = 1;

            //ボタンの変数宣言
            //msg_btnにリスタートする際のcanidとcandlcのパラメータを格納
            auto msg_btn = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_btn->canid = can_main_button_id;
            msg_btn->candlc = 8;

            uint8_t _candata_btn[8];
            bool flag_restart = false;

            //緊急停止
            if(msg->data == "g"){
                RCLCPP_INFO(this->get_logger(), "g");
                robotcontrol_flag = true;
                is_emergency = true;                
            }
            
            //sはリスタート。緊急と手自動のboolをfalseにしてリセットしている。
            //msgがsだったときのみ以下の変数にパラメータが代入される
            if(msg->data == "s")
            {
                RCLCPP_INFO(this->get_logger(), "s");
                robotcontrol_flag = true;
                flag_restart = true;
                is_emergency = false;
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

            //射出
            if(msg->data == "r1"){
                RCLCPP_INFO(this->get_logger(), "r1");
                if(is_injection_convergence && !is_injection_mech_stop_m){
                    auto msg_inject = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                    msg_inject->canid = can_inject_id;
                    msg_inject->candlc = 0;
                    _pub_canusb->publish(*msg_inject);
                }
            }

            //回転停止
            if(msg->data == "r2"){
                RCLCPP_INFO(this->get_logger(), "r2");
                auto msg_inject_spinning = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                msg_inject_spinning->canid = can_inject_spinning_id;
                msg_inject_spinning->candlc = 1;
                msg_inject_spinning->candata[0] = false;
                _pub_canusb->publish(*msg_inject_spinning);
                is_injection_mech_stop_m = true;
                is_move_autonomous = false;
            }

            //射出パラメータ&回転開始
            if(msg->data == "l1"){
                if(is_backside){
                    auto msg_injection = std::make_shared<std_msgs::msg::Bool>();
                    msg_injection->data = true;
                    _pub_injection->publish(*msg_injection);
                    auto msg_inject_spinning = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                    msg_inject_spinning->canid = can_inject_spinning_id;
                    msg_inject_spinning->candlc = 1;
                    msg_inject_spinning->candata[0] = true;
                    _pub_canusb->publish(*msg_inject_spinning);
                    is_backside = true;
                }
                else {
                    auto msg_injection = std::make_shared<std_msgs::msg::Bool>();
                    msg_injection->data = true;
                    _pub_injection->publish(*msg_injection);
                    auto msg_inject_spinning = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                    msg_inject_spinning->canid = can_inject_spinning_id;
                    msg_inject_spinning->candlc = 1;
                    msg_inject_spinning->candata[0] = false;
                    _pub_canusb->publish(*msg_inject_spinning);
                    is_backside = false;
                }
                RCLCPP_INFO(this->get_logger(), "l1");
                is_move_autonomous = true;
                is_injection_mech_stop_m = false;
            }

            //高速低速モードの切り替え
            if(msg->data == "l2"){
                if(is_slow_speed) is_slow_speed = false;
                else is_slow_speed = true;
            }

            //ステアリセット
            if(msg->data == "up"){
                RCLCPP_INFO(this->get_logger(), "up");
                auto msg_steer_reset = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                msg_steer_reset->canid = can_steer_reset_id;
                msg_steer_reset->candlc = 0;
                _pub_canusb->publish(*msg_steer_reset);
            }

            //キャリブレーション
            if(msg->data == "down"){
                RCLCPP_INFO(this->get_logger(), "down");
                auto msg_calibrate = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                msg_calibrate->canid = can_calibrate_id;
                msg_calibrate->candlc = 0;
                _pub_canusb->publish(*msg_calibrate);
            }

            //IO基盤リセット
            if(msg->data == "left"){
                RCLCPP_INFO(this->get_logger(), "left");
                auto msg_main_reset = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                msg_main_reset->canid = can_reset_id;
                msg_main_reset->candlc = 1;
                msg_main_reset->candata[0] = 0;
                _pub_canusb->publish(*msg_main_reset);
            }

            //mian基盤リセット
            if(msg->data == "right"){
                RCLCPP_INFO(this->get_logger(), "right");
                auto msg_io_reset = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                msg_io_reset->canid = can_reset_id;
                msg_io_reset->candlc = 1;
                msg_io_reset->candata[0] = 1;
                _pub_canusb->publish(*msg_io_reset);
            }

            //右ハンド籾の装填
            if(msg->data == "a"){
                if(is_ballhand_convergence){
                    auto msg_paddy_install = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                    msg_paddy_install->candlc = 1;
                    msg_paddy_install->candata[0] = true;
                    msg_paddy_install->canid = can_paddy_install_id;
                    _pub_canusb->publish(*msg_paddy_install);
                }
            }

            //左ハンド籾の装填
            if(msg->data == "b"){
                // if(is_ballhand_convergence){
                //     auto msg_paddy_install = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                //     msg_paddy_install->candlc = 1;
                //     msg_paddy_install->candata[0] = false;
                //     msg_paddy_install->canid = can_paddy_install_id;
                //     _pub_canusb->publish(*msg_paddy_install);
                // }
                is_backside = false;
                is_move_autonomous = true;
                is_injection_mech_stop_m = false;
                auto msg_injection = std::make_shared<std_msgs::msg::Bool>();
                msg_injection->data = false;
                _pub_injection->publish(*msg_injection);
                auto msg_inject_spinning = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                msg_inject_spinning->canid = can_inject_spinning_id;
                msg_inject_spinning->candlc = 1;
                msg_inject_spinning->candata[0] = true;
                _pub_canusb->publish(*msg_inject_spinning);
            }

            //右ハンド籾の回収
            if(msg->data == "x"){
                // if(is_ballhand_convergence){
                //     auto msg_paddy_collect = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                //     msg_paddy_collect->candlc = 1;
                //     msg_paddy_collect->candata[0] = true;
                //     msg_paddy_collect->canid = can_paddy_collect_id;
                //     _pub_canusb->publish(*msg_paddy_collect);
                // }
                is_backside = true;
                is_move_autonomous = true;
                is_injection_mech_stop_m = false;
                auto msg_injection = std::make_shared<std_msgs::msg::Bool>();
                msg_injection->data = true;
                _pub_injection->publish(*msg_injection);
                auto msg_inject_spinning = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                msg_inject_spinning->canid = can_inject_spinning_id;
                msg_inject_spinning->candlc = 1;
                msg_inject_spinning->candata[0] = true;
                _pub_canusb->publish(*msg_inject_spinning);

            }
            
            //左ハンド籾の回収
            if(msg->data == "y"){
                if(is_ballhand_convergence){
                    auto msg_paddy_collect = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                    msg_paddy_collect->candlc = 1;
                    msg_paddy_collect->candata[0] = false;
                    msg_paddy_collect->canid = can_paddy_collect_id;
                    _pub_canusb->publish(*msg_paddy_collect);
                }
            }

            //r3は足回りの手自動の切り替え。is_move_autonomousを使って、トグルになるようにしてる。R1の上物からもらう必要はない。
            if(msg->data == "r3")
            {
                RCLCPP_INFO(this->get_logger(), "r3");
                robotcontrol_flag = true;
                if(is_move_autonomous == false) is_move_autonomous = true;
                else is_move_autonomous = false;
            }

            //l3を押すと射出情報をpublishする
            if(msg->data == "l3")
            {
                RCLCPP_INFO(this->get_logger(), "l3");
                auto initial_sequense_pickup = std::make_shared<std_msgs::msg::String>();
                initial_sequense_pickup->data = initial_pickup_state;
                _pub_initial_sequense->publish(*initial_sequense_pickup);
            }

            //リセットボタンを押しているか確認する
            is_reset = msg->data == "s";

            //base_controlへ代入
            msg_base_control.is_restart = is_reset;
            msg_base_control.is_emergency = is_emergency;
            msg_base_control.is_move_autonomous = is_move_autonomous;
            msg_base_control.is_injection_autonomous = is_injection_autonomous;
            msg_base_control.is_slow_speed = is_slow_speed;
            msg_base_control.initial_state = initial_state;
            msg_base_control.is_injection_mech_stop_m = is_injection_mech_stop_m;

            msg_emergency->candata[0] = is_emergency;

            if(msg->data=="g"){
                _pub_canusb->publish(*msg_emergency);
            }

            if(robotcontrol_flag == true)
            {
                _pub_base_control->publish(msg_base_control);

                msg_unity_control.data = is_reset;
                _pub_base_restart->publish(msg_unity_control);

                msg_unity_control.data = is_emergency;
                _pub_base_emergency->publish(msg_unity_control);

                msg_unity_control.data = is_move_autonomous;
                _pub_move_auto->publish(msg_unity_control);

                msg_unity_control.data = is_injection_autonomous;
                _pub_base_injection->publish(msg_unity_control);
            }

            if(msg->data == "s"){
                _pub_canusb->publish(*msg_restart);
                _pub_canusb->publish(*msg_emergency);
            }

            if(flag_restart == true){
                msg_base_control.is_restart = false;
                _pub_base_control->publish(msg_base_control);
            }
        }

        void SmartphoneGamepad::callback_screen_pad(const std_msgs::msg::String::SharedPtr msg){
            
            //ボタンの処理
            //msg_btnにリスタートする際のcanidとcandlcのパラメータを格納
            auto msg_inject_spinning_screen = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_inject_spinning_screen->canid = can_inject_spinning_id;
            msg_inject_spinning_screen->candlc = 1;

            auto msg_move_node = std::make_shared<std_msgs::msg::String>();
            auto msg_move_node_bool = std::make_shared<std_msgs::msg::Bool>();

            if(msg->data == "O"){
                msg_move_node->data = "O";
                pub_move_node->publish(*msg_move_node);
            }

            if(msg->data == "S0"){
                msg_move_node->data = "S0";
                pub_move_node->publish(*msg_move_node);
            }
            if(msg->data == "S1"){
                msg_move_node->data = "S1";
                pub_move_node->publish(*msg_move_node);
            }
            if(msg->data == "S2"){
                msg_move_node->data = "S2";
                pub_move_node->publish(*msg_move_node);
            }
            if(msg->data == "S3"){
                msg_move_node->data = "S3";
                pub_move_node->publish(*msg_move_node);
            }

            if(msg->data == "P0"){
                msg_move_node->data = "P0";
                pub_move_node->publish(*msg_move_node);
            }
            if(msg->data == "P1"){
                msg_move_node->data = "P1";
                pub_move_node->publish(*msg_move_node);
            }
            if(msg->data == "P2"){
                msg_move_node->data = "P2";
                pub_move_node->publish(*msg_move_node);
            }
            if(msg->data == "P3"){
                msg_move_node->data = "P3";
                pub_move_node->publish(*msg_move_node);
            }
            if(msg->data == "P4"){
                msg_move_node->data = "P4";
                pub_move_node->publish(*msg_move_node);
            }
            if(msg->data == "P5"){
                msg_move_node->data = "P5";
                pub_move_node->publish(*msg_move_node);
            }
            if(msg->data == "P6"){
                msg_move_node->data = "P6";
                pub_move_node->publish(*msg_move_node);
            }
            if(msg->data == "P7"){
                msg_move_node->data = "P7";
                pub_move_node->publish(*msg_move_node);
            }

            if(msg->data == "H0"){
                msg_move_node->data = "H0";
                pub_move_node->publish(*msg_move_node);
            }
            if(msg->data == "H1"){
                msg_move_node->data = "H1";
                pub_move_node->publish(*msg_move_node);
            }
            if(msg->data == "H2"){
                msg_move_node->data = "H2";
                pub_move_node->publish(*msg_move_node);
            }
            if(msg->data == "H3"){
                msg_move_node->data = "H3";
                pub_move_node->publish(*msg_move_node);
            }
            if(msg->data == "H4"){
                msg_move_node->data = "H4";
                pub_move_node->publish(*msg_move_node);
            }
            if(msg->data == "H5"){
                msg_move_node->data = "H5";
                pub_move_node->publish(*msg_move_node);
            }
            if(msg->data == "H6"){
                msg_move_node->data = "H6";
                pub_move_node->publish(*msg_move_node);
            }
            if(msg->data == "H7"){
                msg_move_node->data = "H7";
                pub_move_node->publish(*msg_move_node);
            }
            if(msg->data == "H8"){
                msg_move_node->data = "H8";
                pub_move_node->publish(*msg_move_node);
            }
            if(msg->data == "H9"){
                msg_move_node->data = "H9";
                pub_move_node->publish(*msg_move_node);
            }
            if(msg->data == "H10"){
                msg_move_node->data = "H10";
                pub_move_node->publish(*msg_move_node);
            }
            if(msg->data == "H11"){
                msg_move_node->data = "H11";
                pub_move_node->publish(*msg_move_node);
            }

            if(msg->data == "IJ"){
                msg_move_node->data = "IJ";
                pub_move_node->publish(*msg_move_node);
            }

            if(msg->data == "Seedling_Collection"){
                RCLCPP_INFO(this->get_logger(), "Seedling_Collection");
                msg_move_node_bool->data = true;
                _pub_seedling_collection->publish(*msg_move_node_bool);
            }
            if(msg->data == "Seedling_Installation"){
                msg_move_node_bool->data = true;
                _pub_seedling_installation->publish(*msg_move_node_bool);
            }
            if(msg->data == "ball_Collection"){
                msg_move_node_bool->data = true;
                _pub_ball_collection->publish(*msg_move_node_bool);
            }

        }
        void SmartphoneGamepad::callback_sub_pad(const std_msgs::msg::String::SharedPtr msg){
            auto msg_unity_sub_control = std::make_shared<std_msgs::msg::Bool>();
            int colordlc = 12;
            bool color_data[12];

            if(msg->data == "A_red"){
                RCLCPP_INFO(this->get_logger(), "color_red_A");
                color_data[0] = true;
                msg_colorball_info.color_info[0] = color_data[0];
            }
            if(msg->data == "A_purple"){
                color_data[0] = false;
                RCLCPP_INFO(this->get_logger(), "color_purple_A");
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

            // for(int k=0; k<colordlc;k++){
            //     msg_colorball_info.color_info[k] = color_data[k];
            // }
            
            if(msg->data == "Btn_info_msg"){
                RCLCPP_INFO(this->get_logger(), "color_info_all");
                _pub_color_ball->publish(msg_colorball_info);
            }

        }
        void SmartphoneGamepad::callback_sub_gamepad(const std_msgs::msg::String::SharedPtr msg){
            
            if(msg->data == "a")
            {
                if(is_seedlinghand_convergence){
                    auto msg_seedling_collect = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                    msg_seedling_collect->candlc = 1;
                    msg_seedling_collect->candata[0] = 0;
                    msg_seedling_collect->canid = can_seedling_collect_id;
                    _pub_canusb->publish(*msg_seedling_collect);
                }
            }

            if(msg->data == "b")
            {
                if(is_seedlinghand_convergence){
                    auto msg_seedling_collect = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                    msg_seedling_collect->candlc = 1;
                    msg_seedling_collect->candata[0] = 1;
                    msg_seedling_collect->canid = can_seedling_collect_id;
                    _pub_canusb->publish(*msg_seedling_collect);
                }
            }

            if(msg->data == "x")
            {

            }

            if(msg->data == "y")
            {

            }

            if(msg->data == "up")
            {
                if(is_seedlinghand_convergence){
                    auto msg_seedling_install = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                    msg_seedling_install->candlc = 1;
                    msg_seedling_install->candata[0] = 0;
                    msg_seedling_install->canid = can_seedling_install_id;
                    _pub_canusb->publish(*msg_seedling_install);
                }
            }

             if(msg->data == "right")
            {
                if(is_seedlinghand_convergence){
                    auto msg_seedling_install = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                    msg_seedling_install->candlc = 1;
                    msg_seedling_install->candata[0] = 1;
                    msg_seedling_install->canid = can_seedling_install_id;
                    _pub_canusb->publish(*msg_seedling_install);
                }
            }

            if(msg->data == "down")
            {
                if(is_seedlinghand_convergence){
                    auto msg_seedling_install = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                    msg_seedling_install->candlc = 1;
                    msg_seedling_install->candata[0] = 2;
                    msg_seedling_install->canid = can_seedling_install_id;
                    _pub_canusb->publish(*msg_seedling_install);
                }
            }

            if(msg->data == "left")
            {
                if(is_seedlinghand_convergence){
                    auto msg_seedling_install = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                    msg_seedling_install->candlc = 1;
                    msg_seedling_install->candata[0] = 3;
                    msg_seedling_install->canid = can_seedling_install_id;
                    _pub_canusb->publish(*msg_seedling_install);
                }
            }

        }

            //コントローラからスタート地点情報をsubscribe
        void SmartphoneGamepad::callback_initial_state(const std_msgs::msg::String::SharedPtr msg)
        {
            initial_state = msg->data[0];
        }

        //コントローラから射出情報をsubsclib
        void SmartphoneGamepad::callback_main_injection_possible(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg)
        {
            ///mainから射出可能司令のsub。上物の収束状況。
            is_injection_convergence = static_cast<bool>(msg->candata[0]);
        }

        void SmartphoneGamepad::callback_main_Seedlinghand_possible(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg)
        {
            ///mainから射出可能司令のsub。上物の収束状況。
            is_seedlinghand_convergence = static_cast<bool>(msg->candata[1]);
        }

        void SmartphoneGamepad::callback_main_ballhand_possible(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg)
        {
            ///mainから射出可能司令のsub。上物の収束状況。
            is_ballhand_convergence = static_cast<bool>(msg->candata[2]);
        }

        //splineからの情報をsubsclib
        void SmartphoneGamepad::callback_spline(const std_msgs::msg::Bool::SharedPtr msg)
        {
            //spline_pidから足回り収束のsub。足回りの収束状況。
            is_spline_convergence = msg->data;
            
        }

        //injection_param_calculatorの情報をsubscribe
        //この関数が2つあるのは射出機構が2つあるため
        void SmartphoneGamepad::callback_injection_calculator(const std_msgs::msg::Bool::SharedPtr msg)
        {
            RCLCPP_INFO(this->get_logger(), "false");
             //injection_calculatorから上モノ指令値計算収束のsub。上物の指令値の収束情報。
            is_injection_calculator_convergence = msg->data;
        }

        //スティックの値をUDP通信でsubscribしている
        void SmartphoneGamepad::_recv_callback()
        {
            if(joy_main.is_recved())
            {
                //メモリの使用量を減らすためunsignedを使う
                unsigned char data[16];
                //sizeof関数でdataのメモリを取得
                _recv_joy_main(joy_main.data(data, sizeof(data)));
            }

        }

        //ジョイスティックの値
        void SmartphoneGamepad::_recv_joy_main(const unsigned char data[16])
        {
            float values[4];
            //memcpy関数で指定したバイト数分のメモリをコピー
            memcpy(values, data, sizeof(float)*4);
            //ジョイスティックのベクトル
            //スティックに入力されたXとYをcanidとcandlcのパラメータを格納
            auto msg_linear = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_linear->canid = can_linear_id;
            msg_linear->candlc = 8;
            //ジョイスティックの回転
            //回転の値をcanidとcandlcのパラメータを格納values
            auto msg_angular = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_angular->canid = can_angular_id;
            msg_angular->candlc = 4;
            //twistの型を変数に格納
            auto msg_gazebo = std::make_shared<geometry_msgs::msg::Twist>();

            bool flag_move_autonomous = false;
            
            uint8_t _candata_joy[8];
            //手動モードのとき
            if(is_move_autonomous == false)
            {
                //低速モードのとき
                if(is_slow_speed == true)
                {
                    //低速モード時の速度、加速度、回転をslow_velPlanner_linearに格納
                    slow_velPlanner_linear_x.vel(static_cast<double>(values[1]));//unityとロボットにおける。xとyが違うので逆にしている。
                    slow_velPlanner_linear_y.vel(static_cast<double>(-values[0]));
                    velPlanner_angular_z.vel(static_cast<double>(-values[2]));
                    //cycle関数で演算処理をかけている
                    slow_velPlanner_linear_x.cycle();
                    slow_velPlanner_linear_y.cycle();
                    velPlanner_angular_z.cycle();
                    //floatからバイト(メモリ)に変換
                    float_to_bytes(_candata_joy, static_cast<float>(slow_velPlanner_linear_x.vel()) * slow_manual_linear_max_vel);
                    float_to_bytes(_candata_joy+4, static_cast<float>(slow_velPlanner_linear_y.vel()) * slow_manual_linear_max_vel);
                    for(int i=0; i<msg_linear->candlc; i++) msg_linear->candata[i] = _candata_joy[i];
                    float_to_bytes(_candata_joy, static_cast<float>(velPlanner_angular_z.vel()) * manual_angular_max_vel);
                    for(int i=0; i<msg_angular->candlc; i++) msg_angular->candata[i] = _candata_joy[i];
                    
                    //msg_gazeboに速度計画機の値を格納
                    msg_gazebo->linear.x = slow_velPlanner_linear_x.vel();
                    msg_gazebo->linear.y = slow_velPlanner_linear_y.vel();
                    msg_gazebo->angular.z = velPlanner_angular_z.vel();
                    // RCLCPP_INFO(this->get_logger(), "%f",-slow_velPlanner_linear_y.vel());
                    // RCLCPP_INFO(this->get_logger(), "%f",-slow_velPlanner_linear_x.vel());
                    // RCLCPP_INFO(this->get_logger(),"%f",-velPlanner_angular_z.vel());
                    
                }
                //高速モードのとき
                else
                {
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
                    // RCLCPP_INFO(this->get_logger(), "y:%f",high_velPlanner_linear_y.vel());
                    // RCLCPP_INFO(this->get_logger(), "x:%f",high_velPlanner_linear_x.vel());
                    // RCLCPP_INFO(this->get_logger(),"z:%f",velPlanner_angular_z.vel());
                }
                _pub_canusb->publish(*msg_linear);
                _pub_canusb->publish(*msg_angular);
                _pub_gazebo->publish(*msg_gazebo);
            }
        }

}