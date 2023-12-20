#include "controller_interface/dualsense_interface_node.hpp"
#include <sys/time.h>
#include <sys/types.h>

using namespace utils;

namespace controller_interface
{
    #define BUFFER_NUM 3
    #define BUFSIZE 1024
    using std::string;

    DualSense::DualSense(const rclcpp::NodeOptions &options) : DualSense("", options){}
    DualSense::DualSense(const std::string &name_space, const rclcpp::NodeOptions &options): rclcpp::Node("controller_interface_node", name_space, options),
        //足回り
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
        //ロボットの状態
        defalt_restart_flag(get_parameter("defalt_restart_flag").as_bool()),
        defalt_emergency_flag(get_parameter("defalt_emergency_flag").as_bool()),
        defalt_move_autonomous_flag(get_parameter("defalt_move_autonomous_flag").as_bool()),
        defalt_injection_autonomous_flag(get_parameter("defalt_injection_autonomous_flag_controller").as_bool()),
        defalt_slow_speed_flag(get_parameter("defalt_slow_speed_flag").as_bool()),
        defalt_spline_convergence(get_parameter("defalt_spline_convergence").as_bool()),
        defalt_injection_calculator_convergence(get_parameter("defalt_injection_calculator_convergence").as_bool()),
        defalt_injection_convergence(get_parameter("defalt_injection_convergence").as_bool()),

        udp_port_state(get_parameter("port.robot_state").as_int()),
        //canid
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
            const auto heartbeat_ms = this->get_parameter("heartbeat_ms").as_int();
            const auto convergence_ms = this->get_parameter("convergence_ms").as_int();

            //controllerからsub
            _sub_dualsense_main = this->create_subscription<sensor_msgs::msg::Joy>(
                "dualsense_main/joy",
                _qos,
                std::bind(&DualSense::callback_dualsense_main, this, std::placeholders::_1)
            );

            _sub_dualsense_sub = this->create_subscription<sensor_msgs::msg::Joy>(
                "dualsense_sub/joy",
                _qos,
                std::bind(&DualSense::callback_dualsense_sub, this, std::placeholders::_1)
            );
            //mainからsub
            _sub_main_injection_possible = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
                "can_rx_203",
                _qos,
                std::bind(&DualSense::callback_main_injection_possible, this, std::placeholders::_1)
            );
            _sub_main_Seedlinghand_possible = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
                "can_rx_212",
                _qos,
                std::bind(&DualSense::callback_main_Seedlinghand_possible, this, std::placeholders::_1)
            );
            _sub_main_ballhand_possible = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
                "can_rx_222",
                _qos,
                std::bind(&DualSense::callback_main_ballhand_possible, this, std::placeholders::_1)
            );
            //spline_pidからsub
            _sub_spline = this->create_subscription<std_msgs::msg::Bool>(
                "is_move_tracking",
                _qos,
                std::bind(&DualSense::callback_spline, this, std::placeholders::_1)
            );

            //injection_param_calculatorからsub
            _sub_injection_calculator = this->create_subscription<std_msgs::msg::Bool>(
                "calculator_convergenced_",
                _qos,
                std::bind(&DualSense::callback_injection_calculator, this, std::placeholders::_1)
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

            auto msg_base_control = std::make_shared<controller_interface_msg::msg::BaseControl>();
            //get_parametorで取得したパラメータをrc23pkgsのmsgに格納
            msg_base_control->is_restart = defalt_restart_flag;
            msg_base_control->is_emergency = defalt_emergency_flag;
            msg_base_control->is_move_autonomous = defalt_move_autonomous_flag;
            msg_base_control->is_injection_autonomous = defalt_injection_autonomous_flag;
            msg_base_control->is_slow_speed = defalt_slow_speed_flag;
            this->is_reset = defalt_restart_flag;
            this->is_emergency = defalt_emergency_flag;
            this->is_move_autonomous = defalt_move_autonomous_flag;
            this->is_injection_autonomous = defalt_injection_autonomous_flag;
            this->is_slow_speed = defalt_slow_speed_flag;
            _pub_base_control->publish(*msg_base_control);

            auto msg_emergency = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_emergency->canid = can_emergency_id;
            msg_emergency->candlc = 1;
            msg_emergency->candata[0] = defalt_emergency_flag;
            _pub_canusb->publish(*msg_emergency);

            //収束を確認するmsgの宣言
            auto msg_convergence = std::make_shared<controller_interface_msg::msg::Convergence>();
            //get_parametorで取得したパラメータをrc23pkgsのmsgに格納
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

            //ハートビート
            //コントローラの鼓動
            //一定周期で処理をしている。この場合は100ms間隔で処理をしている
            _pub_heartbeat = this->create_wall_timer(
                std::chrono::milliseconds(heartbeat_ms),
                [this] {
                    auto msg_heartbeat = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                    //get_parametorで取得したパラメータをrc23pkgsのmsgに格納
                    msg_heartbeat->canid = can_heartbeat_id;
                    msg_heartbeat->candlc = 0;
                    _pub_canusb->publish(*msg_heartbeat);
                }
            );

            //convergence
            //収束状況
            //一定周期で処理をしている。この場合は100ms間隔で処理をしている
            _pub_timer_convergence = this->create_wall_timer(
                std::chrono::milliseconds(convergence_ms),
                [this] {
                    auto msg_convergence = std::make_shared<controller_interface_msg::msg::Convergence>();
                    //get_parametorで取得したパラメータをrc23pkgsのmsgに格納
                    msg_convergence->spline_convergence = is_spline_convergence;
                    msg_convergence->injection_calculator = is_injection_calculator_convergence;
                    msg_convergence->injection = is_injection_convergence;
                    msg_convergence->seedlinghand = is_seedlinghand_convergence;
                    msg_convergence->ballhand = is_ballhand_convergence;
                    _pub_convergence->publish(*msg_convergence);
                }
            );
            //一定周期で処理をしている。この場合は3000ms間隔で処理をしている
            _start_timer = this->create_wall_timer(
                std::chrono::milliseconds(this->get_parameter("start_ms").as_int()),
                [this] {
                    if(start_flag)
                    {
                        const string initial_inject_state_with_null = initial_inject_state + '\0';
                        //c_strがポインタを返すためアスタリスクをつける
                        const char* char_ptr2 = initial_inject_state_with_null.c_str();
                        //reinterpret_castでポインタ型の変換
                        //char_ptr2をconst unsigned charに置き換える
                        const unsigned char* inject = reinterpret_cast<const unsigned char*>(char_ptr2);
                        //commandクラスのudp通信で一番最初に回収するデータをコントローラーに送り、コントローラ側で処理が行われる
                        command.state_num_R1(inject, r1_pc,udp_port_state);
                        start_flag = false;
                    }
                }
            );
            //計画機
            slow_velPlanner_linear_x.limit(slow_limit_linear);
            slow_velPlanner_linear_y.limit(slow_limit_linear);
            velPlanner_angular_z.limit(limit_angular);

            upedge_autonomous_main(0);
            upedge_emergency_main(0);
            upedge_restart_main(0);
            upedge_autonomous_sub(0);
            upedge_emergency_sub(0);
            upedge_restart_sub(0);

        }
        void DualSense::callback_dualsense_main(const sensor_msgs::msg::Joy::SharedPtr msg)
        {
            //リスタートの処理
            //msg_restartにリスタートする際のcanidとcandlcのパラメータを格納
            auto msg_restart = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_restart->canid = can_restart_id;
            msg_restart->candlc = 0;

            //緊急停止の処理
            //msg_emergencyにリスタートする際のcanidとcandlcのパラメータを格納
            auto msg_emergency = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_emergency->canid = can_emergency_id;
            msg_emergency->candlc = 1;

            uint8_t _candata_btn[8];
            //base_control(手自動、緊急、リスタート)が押されたらpubする

            //resertがtureをpubした後にfalseをpubする
            bool flag_restart = false;
            bool move_node_flag = false;
            bool emergency_lock;
            bool restart_lock;

            //dualsenseのボタン配列メモ
            //button:  0:cross  1:circle  2:triangle  3:square  4:L1  5:R1  6:L2  7:R2  8:create  9:option  10:ps  11:joyL  12:joyR
            //axes:  0:joyL_x  1:joyL_y  2:L2  3:joyR_x  4:joyR_y  5:R2  6:Left(1),Right(-1)  7:Up(1),Down(-1)
            
            if(upedge_restart_main(msg->buttons[10])){  //PSボタン
                if(!move_node_lock){
                    move_node_lock = true;
                }
            }
            //button[12](joyR(押し込み))は足回りの手自動の切り替え。is_move_autonomousを使って、トグルになるようにしてる。ERの上物からもらう必要はない。
            if(upedge_r3_main(msg->buttons[12]))//joyR(押し込み)
            {
                robotcontrol_flag = true;
                if(is_move_autonomous == false){
                    is_move_autonomous = true;
                }else{
                    is_move_autonomous = false;
                }
            }
            //button[8](create)は緊急。is_emergencyを使って、トグルになるようにしてる。
            if(upedge_emergency_main(msg->buttons[8]))//create
            {
                robotcontrol_flag = true;
                is_emergency = true;
                emergency_lock = true;
            }
            //button[9](option)はリスタート。緊急と手自動のboolをfalseにしてリセットしている。
            if(upedge_restart_main(msg->buttons[9]))//option
            {
                robotcontrol_flag = true;
                flag_restart = true;
                is_emergency = false;
                is_injection_mech_stop_m = false;
                is_move_autonomous = defalt_move_autonomous_flag;
                is_injection_autonomous = defalt_injection_autonomous_flag;
                is_slow_speed = defalt_slow_speed_flag;

                move_node_string = "";
                restart_lock = true;
            }

            is_reset = msg->buttons[9];//option
            //basecontrolへの代入

            msg_base_control.is_restart = is_reset;
            msg_base_control.is_emergency = is_emergency;
            msg_base_control.is_move_autonomous = is_move_autonomous;
            msg_base_control.is_injection_autonomous = is_injection_autonomous;
            msg_base_control.is_slow_speed = is_slow_speed;
            msg_base_control.initial_state = initial_state;
            msg_base_control.is_injection_mech_stop_m = is_injection_mech_stop_m;

            //射出
            if(upedge_r1_main(msg->buttons[5])){    //R1
                RCLCPP_INFO(this->get_logger(), "r1");
                if(is_injection_convergence){
                    auto msg_inject = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                    msg_inject->canid = can_inject_id;
                    msg_inject->candlc = 0;
                    _pub_canusb->publish(*msg_inject);
                }
            }
            //回転停止
            if(upedge_r2_main(msg->buttons[7])){    //R2
                RCLCPP_INFO(this->get_logger(), "r2");
                auto msg_inject_spinning = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                msg_inject_spinning->canid = can_inject_spinning_id;
                msg_inject_spinning->candlc = 1;
                msg_inject_spinning->candata[0] = false;
                _pub_canusb->publish(*msg_inject_spinning);
                is_injection_mech_stop_m = true;
            }
            //射出パラメータ&回転開始
            if(upedge_l1_main(msg->buttons[4])){    //L1
                if(is_backside){
                    RCLCPP_INFO(this->get_logger(), "l1");
                    auto msg_injection = std::make_shared<std_msgs::msg::Bool>();
                    msg_injection->data = true;
                    _pub_injection->publish(*msg_injection);
                    auto msg_inject_spinning = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                    msg_inject_spinning->canid = can_inject_spinning_id;
                    msg_inject_spinning->candlc = 1;
                    msg_inject_spinning->candata[0] = false;
                    _pub_canusb->publish(*msg_inject_spinning);
                    is_injection_mech_stop_m = false;
                    is_backside = false;
                }
                else {
                    RCLCPP_INFO(this->get_logger(), "l2");
                    auto msg_injection = std::make_shared<std_msgs::msg::Bool>();
                    msg_injection->data = false;
                    _pub_injection->publish(*msg_injection);
                    auto msg_inject_spinning = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                    msg_inject_spinning->canid = can_inject_spinning_id;
                    msg_inject_spinning->candlc = 1;
                    msg_inject_spinning->candata[0] = true;
                    _pub_canusb->publish(*msg_inject_spinning);
                    is_injection_mech_stop_m = false;
                    is_backside = true;
                }
            }
            if(upedge_l2_main(msg->buttons[6])){    //L2
                if(is_slow_speed){
                    is_slow_speed = false;
                }
                else{
                    is_slow_speed = true;
                }
            }
            //ステアリセット
            if(msg->axes[7] == 1){   //UP
                RCLCPP_INFO(this->get_logger(), "up");
                auto msg_steer_reset = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                msg_steer_reset->canid = can_steer_reset_id;
                msg_steer_reset->candlc = 0;
                _pub_canusb->publish(*msg_steer_reset);
            }

            //キャリブレーション
            if(msg->axes[7] == -1){     //down
                RCLCPP_INFO(this->get_logger(), "down");
                auto msg_calibrate = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                msg_calibrate->canid = can_calibrate_id;
                msg_calibrate->candlc = 0;
                _pub_canusb->publish(*msg_calibrate);
            }
            //main基板リセット
            if(msg->axes[6] == -1){     //Right     
                RCLCPP_INFO(this->get_logger(), "right");
                auto msg_main_reset = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                msg_main_reset->canid = can_reset_id;
                msg_main_reset->candlc = 1;
                msg_main_reset->candata[0] = 0;
                _pub_canusb->publish(*msg_main_reset);
            }
            //IO基板リセット
            if(msg->axes[6] == 1){      //Left
                RCLCPP_INFO(this->get_logger(), "left");
                auto msg_io_reset = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                msg_io_reset->canid = can_reset_id;
                msg_io_reset->candlc = 1;
                msg_io_reset->candata[0] = 1;
                _pub_canusb->publish(*msg_io_reset);
            }
            //右ハンド籾の装填
            if(upedge_circle_main(msg->buttons[1])){    //circle
                if(is_ballhand_convergence){
                    auto msg_paddy_install = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                    msg_paddy_install->candlc = 1;
                    msg_paddy_install->candata[0] = true;
                    msg_paddy_install->canid = can_paddy_install_id;
                    _pub_canusb->publish(*msg_paddy_install);
                }
            }
            //左ハンド籾の装填
            if(upedge_cross_main(msg->buttons[0])){     //cross
                if(is_ballhand_convergence){
                    auto msg_paddy_install = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                    msg_paddy_install->candlc = 1;
                    msg_paddy_install->candata[0] = false;
                    msg_paddy_install->canid = can_paddy_install_id;
                    _pub_canusb->publish(*msg_paddy_install);
                }
            }
            //右ハンド籾の回収
            if(upedge_triangle_main(msg->buttons[2])){  //triangle
                if(is_ballhand_convergence){
                    auto msg_paddy_collect = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                    msg_paddy_collect->candlc = 1;
                    msg_paddy_collect->candata[0] = true;
                    msg_paddy_collect->canid = can_paddy_collect_id;
                    _pub_canusb->publish(*msg_paddy_collect);
                }
            }
            //左ハンド籾の回収
            if(upedge_square_main(msg->buttons[3])){    //square
                if(is_ballhand_convergence){
                    auto msg_paddy_collect = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                    msg_paddy_collect->candlc = 1;
                    msg_paddy_collect->candata[0] = false;
                    msg_paddy_collect->canid = can_paddy_collect_id;
                    _pub_canusb->publish(*msg_paddy_collect);
                }
            }
            if(upedge_l3_main(msg->buttons[11]))//joyL(押し込み)
            {
                //c_strがポインタ型を返すためアスタリスクをつける
                const char* char_ptr = initial_pickup_state.c_str();
                //reinterpret_castでポインタ型の変換
                //char_ptr1をconst unsigned charに置き換える
                const unsigned char* pickup = reinterpret_cast<const unsigned char*>(char_ptr);
                //commandクラスのudp通信で一番最初に回収するデータをコントローラーに送り、コントローラ側で処理が行われる
                command.state_num_R1(pickup, r1_pc,udp_port_state);
                //同じ処理が連続で起きないようにfalseでもとの状態に戻す
                start_flag = true;
                move_node_flag = true;
            }
            if(emergency_lock)//create
            {
                _pub_canusb->publish(*msg_emergency);
            }
            if(emergency_lock){
                _pub_canusb->publish(*msg_emergency);
            }
            if(restart_lock)
            {
                _pub_canusb->publish(*msg_restart);
                _pub_canusb->publish(*msg_emergency);
            }
            if(robotcontrol_flag){
                _pub_base_control->publish(msg_base_control);
            }
            if(flag_restart == true)
            {
                msg_base_control.is_restart = false;
                _pub_base_control->publish(msg_base_control);
            }
            //スティック情報
            analog_l_x_main = msg->axes[0];     //Joy_Left_X
            analog_l_y_main = msg->axes[1];     //Joy_Left_Y
            analog_r_x_main = msg->axes[3];     //Joy_Right_X
            analog_r_y_main = msg->axes[4];     //Joy_Right_X

        }

        //コントローラから射出情報をsubsclib
        void DualSense::callback_main_injection_possible(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg)
        {
            ///mainから射出可能司令のsub。上物の収束状況。
            is_injection_convergence = static_cast<bool>(msg->candata[0]);
        }

        void DualSense::callback_main_Seedlinghand_possible(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg)
        {
            ///mainから射出可能司令のsub。上物の収束状況。
            is_seedlinghand_convergence = static_cast<bool>(msg->candata[1]);
        }

        void DualSense::callback_main_ballhand_possible(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg)
        {
            ///mainから射出可能司令のsub。上物の収束状況。
            is_ballhand_convergence = static_cast<bool>(msg->candata[2]);
        }

        //splineからの情報をsubsclib
        void DualSense::callback_spline(const std_msgs::msg::Bool::SharedPtr msg)
        {
            //spline_pidから足回り収束のsub。足回りの収束状況。
            is_spline_convergence = msg->data;
            
        }

        //injection_param_calculatorの情報をsubscribe
        //この関数が2つあるのは射出機構が2つあるため
        void DualSense::callback_injection_calculator(const std_msgs::msg::Bool::SharedPtr msg)
        {
            RCLCPP_INFO(this->get_logger(), "false");
             //injection_calculatorから上モノ指令値計算収束のsub。上物の指令値の収束情報。
            is_injection_calculator_convergence = msg->data;
        }



}