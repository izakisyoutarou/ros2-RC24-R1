#include "controller_interface/controller_interface_node.hpp"

using namespace utils;

namespace controller_interface
{
    #define BUFSIZE 1024
    using std::string;
    
    SmartphoneGamepad::SmartphoneGamepad(const rclcpp::NodeOptions &options, const int tcp_endpoint_num, const int udp_port) : SmartphoneGamepad("", options, tcp_endpoint_num, udp_port) {}
    SmartphoneGamepad::SmartphoneGamepad(const std::string &name_space, const rclcpp::NodeOptions &options, const int tcp_endpoint_num, const int udp_port)
        : rclcpp::Node("controller_interface_node", name_space, options),
        limit_linear(DBL_MAX,
        get_parameter("linear_max_vel").as_double(),
        get_parameter("linear_max_acc").as_double(),
        get_parameter("linear_max_dec").as_double() ),
        limit_angular(DBL_MAX,
        dtor(get_parameter("angular_max_vel").as_double()),
        dtor(get_parameter("angular_max_acc").as_double()),
        dtor(get_parameter("angular_max_dec").as_double()) ),
        limit_injection(DBL_MAX,
        dtor(get_parameter("injection_max_vel").as_double()),
        dtor(get_parameter("injection_max_acc").as_double()),
        dtor(get_parameter("injection_max_dec").as_double()) ),
        
        udp_port(udp_port),
        tcp_endpoint_num(tcp_endpoint_num),
        manual_linear_max_vel(static_cast<float>(get_parameter("linear_max_vel").as_double())),
        manual_angular_max_vel(dtor(static_cast<float>(get_parameter("angular_max_vel").as_double()))),
        manual_injection_max_vel(dtor(static_cast<float>(get_parameter("injection_max_vel").as_double()))),
        defalt_restart_flag(get_parameter("defalt_restart_flag").as_bool()),
        defalt_wheel_autonomous_flag(get_parameter("defalt_wheel_autonomous_flag").as_bool()),
        defalt_injection_autonomous_flag(get_parameter("defalt_injection_autonomous_flag").as_bool()),
        defalt_emergency_flag(get_parameter("defalt_emergency_flag").as_bool())
        {  
            const auto heartbeat_ms = this->get_parameter("heartbeat_ms").as_int();

            //controllerからsub
            _sub_pad = this->create_subscription<controller_interface_msg::msg::SubPad>(
                "ros_tcp_endpoint_" + std::to_string(tcp_endpoint_num) + "/sub_pad",
                _qos,
                std::bind(&SmartphoneGamepad::callback_pad, this, std::placeholders::_1)
            );

            //mainからsub
            _sub_main = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
                "can_rx_121",
                _qos,
                std::bind(&SmartphoneGamepad::callback_main, this, std::placeholders::_1)
            );

            //spline_pidからsub
            _sub_spline = this->create_subscription<std_msgs::msg::Bool>(
                "is_move_tracking",
                _qos,
                std::bind(&SmartphoneGamepad::callback_spline, this, std::placeholders::_1)
            );

            //injection_param_calculatorからsub
            _sub_injection_calculator_er_left = this->create_subscription<std_msgs::msg::Bool>(
                "is_calculator_convergenced_left",
                _qos,
                std::bind(&SmartphoneGamepad::callback_injection_calculator_er_left, this, std::placeholders::_1)
            );

            _sub_injection_calculator_er_right = this->create_subscription<std_msgs::msg::Bool>(
                "is_calculator_convergenced_right",
                _qos,
                std::bind(&SmartphoneGamepad::callback_injection_calculator_er_right, this, std::placeholders::_1)
            );

            _sub_injection_calculator_rr = this->create_subscription<std_msgs::msg::Bool>(
                "is_calculator_convergenced_right",
                _qos,
                std::bind(&SmartphoneGamepad::callback_injection_calculator_rr, this, std::placeholders::_1)
            );

            //canusbへpub
            _pub_canusb = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);

            //controllerへpub
            _pub_convergence = this->create_publisher<controller_interface_msg::msg::Convergence>("ros_tcp_endpoint_" + std::to_string(tcp_endpoint_num) + "/pub_convergence" , _qos);

            //各nodeへリスタートと手自動の切り替えをpub。
            _pub_base_control = this->create_publisher<controller_interface_msg::msg::BaseControl>("ros_tcp_endpoint_" + std::to_string(tcp_endpoint_num) + "/base_control",_qos);

            //test用のpub
            _pub_test = this->create_publisher<std_msgs::msg::Bool>("is_move_tracking", _qos);

            //デフォルト値をpub.。各種、boolに初期値を代入。
            auto msg_base_control = std::make_shared<controller_interface_msg::msg::BaseControl>();
            msg_base_control->is_restart = defalt_restart_flag;
            msg_base_control->is_wheel_autonomous = defalt_wheel_autonomous_flag;
            msg_base_control->is_injection_autonomous = defalt_injection_autonomous_flag;
            msg_base_control->is_emergency = defalt_emergency_flag;
            this->is_reset = defalt_restart_flag;
            this->is_wheel_autonomous = defalt_wheel_autonomous_flag;
            this->is_injection_autonomous = defalt_injection_autonomous_flag;
            this->is_emergency = defalt_emergency_flag;
            _pub_base_control->publish(*msg_base_control);

            //ハートビート
            _pub_timer = this->create_wall_timer(
                std::chrono::milliseconds(heartbeat_ms),
                [this] { 
                    // auto msg_test = std::make_shared<std_msgs::msg::Bool>();
                    // msg_test->data = true;
                    // _pub_test->publish(*msg_test);
                    auto msg_heartbeat = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                    msg_heartbeat->canid = 0x001;
                    msg_heartbeat->candlc = 1;
                    //_pub_canusb->publish(*msg_heartbeat);
                }
            );

            //計画機
            velPlanner_linear_x.limit(limit_linear);
            velPlanner_linear_y.limit(limit_linear);
            velPlanner_angular_z.limit(limit_angular);
            velPlanner_injection_v.limit(limit_injection);

            //UDP
            // struct in_addr local_addr;
            // inet_pton(AF_INET, "192.168.1.4", &local_addr);
            sockfd = socket(AF_INET, SOCK_DGRAM, 0);
            memset(&servaddr, 0, sizeof(servaddr));
            servaddr.sin_family = AF_INET;
            //servaddr.sin_addr = local_addr;
            servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
            servaddr.sin_port = htons(udp_port);
            bind(sockfd, (struct sockaddr *) &servaddr, sizeof(servaddr));
            
            //UDPthread
            udp_thread_ = std::thread(&SmartphoneGamepad::callback_udp, this);
        }

        void SmartphoneGamepad::callback_pad(const controller_interface_msg::msg::SubPad::SharedPtr msg)
        {
            auto msg_restart = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_restart->canid = 0x002;
            msg_restart->candlc = 0;

            auto msg_emergency = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_emergency->canid = 0x000;
            msg_emergency->candlc = 1;

            auto msg_injection = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_injection->canid = 0x120;
            msg_injection->candlc = 2;

            uint8_t _candata_btn[2];

            bool robotcontrol_flag = false;//base_control(手自動、緊急、リスタート)が押されたらpubする
            bool flag_restart = false;//resertがtureをpubした後にfalseをpubする
            bool flag_injection0 = false;//左の発射機構の最終射出許可
            bool flag_injection1 = false;//右の発射機構の最終射出許可

            //r3は足回りの手自動の切り替え。is_wheel_autonomousを使って、トグルになるようにしてる。ERの上物からもらう必要はない。
            //ERの上物の場合は、上物の切り替えに当てている。
            
            if(msg->r3)
            {
                if(udp_port == 50000 || udp_port == 52000)
                {
                    robotcontrol_flag = true;
                    if(is_wheel_autonomous == false) is_wheel_autonomous = true;
                    else is_wheel_autonomous = false;
                }
                if(udp_port == 51000)
                {
                    robotcontrol_flag = true;
                    if(is_injection_0 == false) is_injection_0 = true;
                    else is_injection_0 = false;
                }
            }

            //l3は上物の手自動の切り替え。is_injection_autonomousを使って、トグルになるようにしてる。ERの足回りからもらう必要はない
            if(udp_port == 51000 || udp_port == 52000)
            {
                if(msg->l3)
                {
                    robotcontrol_flag = true;
                    if(is_injection_autonomous == false) is_injection_autonomous = true;
                    else is_injection_autonomous = false;
                }
            }

            //gは緊急。is_emergencyを使って、トグルになるようにしてる。
            if(msg->g)
            {
                robotcontrol_flag = true;
                if(is_emergency == false) is_emergency = true;
                else is_emergency = false;
            }

            //sはリスタート。緊急と手自動のboolをfalseにしてリセットしている。
            if(msg->s)
            {
                robotcontrol_flag = true;
                flag_restart = true;
                is_wheel_autonomous = defalt_wheel_autonomous_flag;
                is_injection_autonomous = defalt_injection_autonomous_flag;
                is_emergency = defalt_emergency_flag;
            }

            //l2が左、r2が右の発射機構のトリガー。
            //それぞれ、発射されたら収束がfalseにするようにしている。
            if(msg->l2)
            {
                if(is_spline_convergence && is_injection0_convergence && is_injection_calculator0_convergence)
                {
                flag_injection0 = true;
                is_injection0_convergence = false;
                is_injection_calculator0_convergence = false;
                }
            }
            
            if(msg->r2)
            {
                if(is_spline_convergence && is_injection1_convergence && is_injection_calculator1_convergence)
                {
                flag_injection1 = true;
                is_injection1_convergence = false;
                is_injection_calculator1_convergence = false;
                }
            }

            is_reset = msg->s;

            //basecontrolへの代入
            auto msg_base_control = std::make_shared<controller_interface_msg::msg::BaseControl>();
            msg_base_control->is_restart = is_reset;
            msg_base_control->is_wheel_autonomous = is_wheel_autonomous;
            msg_base_control->is_injection_autonomous = is_injection_autonomous;
            msg_base_control->is_emergency = is_emergency;
            
            //mainへ緊急を送る代入
            _candata_btn[0] = is_emergency;
            for(int i=0; i<msg_emergency->candlc; i++) msg_emergency->candata[i] = _candata_btn[i];

            //mainへ射出司令を送る代入
            _candata_btn[0] = flag_injection0;
            _candata_btn[1] = flag_injection1;
            for(int i=0; i<msg_injection->candlc; i++) msg_injection->candata[i] = _candata_btn[i];

            if(msg->g)_pub_canusb->publish(*msg_emergency);
            if(msg->s)_pub_canusb->publish(*msg_restart);
            if(flag_injection0 || flag_injection1)_pub_canusb->publish(*msg_injection);
            if(robotcontrol_flag)_pub_base_control->publish(*msg_base_control);
            if(flag_restart)
            {
                msg_base_control->is_restart = false;
                _pub_base_control->publish(*msg_base_control);
            }
        }

        void SmartphoneGamepad::callback_udp()
        {
            auto msg_linear = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_linear->canid = 0x100;
            msg_linear->candlc = 8;

            auto msg_angular = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_angular->canid = 0x101;
            msg_angular->candlc = 4;

            auto msg_l_elevation_velocity = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_linear->canid = 0x130;
            msg_linear->candlc = 8;

            auto msg_l_yaw = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_angular->canid = 0x131;
            msg_angular->candlc = 4;

            auto msg_r_elevation_velocity = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_linear->canid = 0x132;
            msg_linear->candlc = 8;

            auto msg_r_yaw = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_angular->canid = 0x133;
            msg_angular->candlc = 4;

            uint8_t _candata_joy[8];

            bool flag_wheel_autonomous = false;
            bool flag_injection_autonomous = false;

            float analog_l_x = 0.0f;
            float analog_l_y = 0.0f;
            float analog_r_x = 0.0f;
            float analog_r_y = 0.0f;

            while(rclcpp::ok())
            {
                clilen = sizeof(cliaddr);
                    
                // bufferに受信したデータが格納されている
                n = recvfrom(sockfd, buffer, BUFSIZE, 0, (struct sockaddr *) &cliaddr, &clilen);
                    
                if (n < 0)
                {
                    perror("recvfrom");
                    exit(1);
                }

                std::memcpy(&analog_l_x, &buffer[0], sizeof(analog_l_x));
                std::memcpy(&analog_l_y, &buffer[4], sizeof(analog_l_y));
                std::memcpy(&analog_r_x, &buffer[8], sizeof(analog_r_x));
                std::memcpy(&analog_r_y, &buffer[12], sizeof(analog_r_y));

                if(is_wheel_autonomous == false && is_injection_autonomous == true)
                {
                    velPlanner_linear_x.vel(static_cast<double>(analog_l_y));//unityとロボットにおける。xとyが違うので逆にしている。
                    velPlanner_linear_y.vel(static_cast<double>(analog_l_x));
                    velPlanner_angular_z.vel(static_cast<double>(analog_r_x));

                    velPlanner_linear_x.cycle();
                    velPlanner_linear_y.cycle();
                    velPlanner_angular_z.cycle();

                    //RCLCPP_INFO(this->get_logger(), "vel_x:%f", analog_l_y);

                    float_to_bytes(_candata_joy, static_cast<float>(velPlanner_linear_x.vel()) * manual_linear_max_vel);
                    float_to_bytes(_candata_joy+4, static_cast<float>(velPlanner_linear_y.vel()) * manual_linear_max_vel);
                    for(int i=0; i<msg_linear->candlc; i++) msg_linear->candata[i] = _candata_joy[i];

                    float_to_bytes(_candata_joy, static_cast<float>(velPlanner_angular_z.vel()) * manual_angular_max_vel);
                    for(int i=0; i<msg_angular->candlc; i++) msg_angular->candata[i] = _candata_joy[i];

                    _pub_canusb->publish(*msg_linear);
                    _pub_canusb->publish(*msg_angular);

                    flag_wheel_autonomous = true;
                }
                else if(is_injection_autonomous == false && is_wheel_autonomous == true)
                {
                    velPlanner_injection_v.vel(static_cast<double>(analog_l_x));

                    velPlanner_injection_v.cycle();

                    float_to_bytes(_candata_joy+4, static_cast<float>(velPlanner_injection_v.vel()) * manual_injection_max_vel);
                    float_to_bytes(_candata_joy, static_cast<float>(atan2(-analog_r_x, analog_r_y)));

                    if(is_injection_0)
                    {
                    for(int i=0; i<msg_l_elevation_velocity->candlc; i++) msg_l_elevation_velocity->candata[i] = _candata_joy[i];
                    for(int i=0; i<msg_l_yaw->candlc; i++) msg_l_yaw->candata[i] = _candata_joy[i];

                    _pub_canusb->publish(*msg_l_elevation_velocity);
                    _pub_canusb->publish(*msg_l_yaw);
                    }
                    else
                    {
                        for(int i=0; i<msg_r_elevation_velocity->candlc; i++) msg_r_elevation_velocity->candata[i] = _candata_joy[i];
                        for(int i=0; i<msg_r_yaw->candlc; i++) msg_r_yaw->candata[i] = _candata_joy[i];

                        _pub_canusb->publish(*msg_r_elevation_velocity);
                        _pub_canusb->publish(*msg_r_yaw);
                    }

                    flag_injection_autonomous = true;
                }
                else 
                {
                    //手動から自動になったときに、一回だけ速度指令値に0を代入してpubする。
                    if(flag_wheel_autonomous == true || flag_injection_autonomous == true)
                    {
                        RCLCPP_INFO(this->get_logger(), "vel_x:%f", analog_l_y);

                        float_to_bytes(_candata_joy, 0);
                        for(int i=0; i<msg_linear->candlc; i++) msg_linear->candata[i] = _candata_joy[i];
                        for(int i=0; i<msg_angular->candlc; i++) msg_angular->candata[i] = _candata_joy[i];
                        for(int i=0; i<msg_l_elevation_velocity->candlc; i++) msg_l_elevation_velocity->candata[i] = _candata_joy[i];
                        for(int i=0; i<msg_l_yaw->candlc; i++) msg_l_yaw->candata[i] = _candata_joy[i];
                        for(int i=0; i<msg_r_elevation_velocity->candlc; i++) msg_r_elevation_velocity->candata[i] = _candata_joy[i];
                        for(int i=0; i<msg_r_yaw->candlc; i++) msg_r_yaw->candata[i] = _candata_joy[i];

                        _pub_canusb->publish(*msg_linear);
                        _pub_canusb->publish(*msg_angular);

                        flag_wheel_autonomous = false;
                        flag_injection_autonomous = false;
                    }
                }
            }
        }

        void SmartphoneGamepad::callback_main(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg)
        {
            //mainから射出可能司令のsub。それぞれをconvergenceの適当なところに入れてpub。上物の収束状況。
            auto msg_injectioncommnd = std::make_shared<controller_interface_msg::msg::Convergence>();
            uint8_t _candata[2];
            for(int i=0; i<msg->candlc; i++) _candata[i] = msg->candata[i];
            msg_injectioncommnd->injection0 = static_cast<bool>(_candata[0]);
            msg_injectioncommnd->injection1 = static_cast<bool>(_candata[1]);
            _pub_convergence->publish(*msg_injectioncommnd);
        }

        void SmartphoneGamepad::callback_spline(const std_msgs::msg::Bool::SharedPtr msg)
        {
            //spline_pidから足回り収束のsub。onvergenceの適当なところに入れてpub。足回りの収束状況。
            auto msg_spline_convergence = std::make_shared<controller_interface_msg::msg::Convergence>();
            is_spline_convergence = msg->data;
            msg_spline_convergence->spline_convergence = is_spline_convergence;
            _pub_convergence->publish(*msg_spline_convergence);
        }

        void SmartphoneGamepad::callback_injection_calculator_er_left(const std_msgs::msg::Bool::SharedPtr msg)
        {
            //injection_calculatorから上モノ指令値計算収束のsub。onvergenceの適当なところに入れてpub。上物の指令値の収束情報。
            auto msg_injection_calculator0_convergence = std::make_shared<controller_interface_msg::msg::Convergence>();
            is_injection_calculator0_convergence = msg->data;
            msg_injection_calculator0_convergence->injection_calculator0 = is_injection_calculator0_convergence;
            _pub_convergence->publish(*msg_injection_calculator0_convergence);
        }

        void SmartphoneGamepad::callback_injection_calculator_er_right(const std_msgs::msg::Bool::SharedPtr msg)
        {
            //injection_calculatorから上モノ指令値計算収束のsub。onvergenceの適当なところに入れてpub。上物の指令値の収束情報。
            auto msg_injection_calculator1_convergence = std::make_shared<controller_interface_msg::msg::Convergence>();
            is_injection_calculator1_convergence = msg->data;
            msg_injection_calculator1_convergence->injection_calculator1 = is_injection_calculator1_convergence;
            _pub_convergence->publish(*msg_injection_calculator1_convergence);
        }

         void SmartphoneGamepad::callback_injection_calculator_rr(const std_msgs::msg::Bool::SharedPtr msg)
        {
            //injection_calculatorから上モノ指令値計算収束のsub。onvergenceの適当なところに入れてpub。上物の指令値の収束情報。
            // auto msg_injection_calculator1_convergence = std::make_shared<controller_interface_msg::msg::Convergence>();
            // is_injection_calculator1_convergence = msg->data;
            // msg_injection_calculator1_convergence->injection_calculator1 = is_injection_calculator1_convergence;
            // _pub_convergence->publish(*msg_injection_calculator1_convergence);
        }
    
    DualSense::DualSense(const rclcpp::NodeOptions &options) : DualSense("", options) {}
    DualSense::DualSense(const std::string &name_space, const rclcpp::NodeOptions &options)
        : rclcpp::Node("controller_interface_node", name_space, options),
        limit_linear(DBL_MAX,
        get_parameter("linear_max_vel").as_double(),
        get_parameter("linear_max_acc").as_double(),
        get_parameter("linear_max_dec").as_double() ),
        limit_angular(DBL_MAX,
        dtor(get_parameter("angular_max_vel").as_double()),
        dtor(get_parameter("angular_max_acc").as_double()),
        dtor(get_parameter("angular_max_dec").as_double()) )
        {
            _sub_SPpad = this->create_subscription<sensor_msgs::msg::Joy>(
                "/joy",
                _qos,
                std::bind(&DualSense::callback_SPpad, this, std::placeholders::_1)
            );
            RCLCPP_INFO(this->get_logger(), "Hello World");
        }

        void DualSense::callback_SPpad(const sensor_msgs::msg::Joy::SharedPtr msg)
        {
            RCLCPP_INFO(this->get_logger(), "Hello World");
        }
}
