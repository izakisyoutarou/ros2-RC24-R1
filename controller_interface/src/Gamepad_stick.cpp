#include "controller_interface/Gamepad_stick.hpp"

Gamepadstick::Gamepadstick(){}

msg Gamepadstick::_recv_joy_main(const unsigned char data[16],const int16_t can_linear_id,const int16_t can_angular_id,bool is_move_autonomous,bool is_slow_speed,
                            const float high_manual_linear_max_vel,const float slow_manual_linear_max_vel,const float manual_angular_max_vel){
        float values[4];
        //memcpy関数で指定したバイト数分のメモリをコピー
        memcpy(values, data, sizeof(float)*4);
        //ジョイスティックのベクトル
        //スティックに入力されたXとYをcanidとcandlcのパラメータを格納
        auto msg_linear = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_linear->canid = can_linear_id;
        msg_linear->candlc = 8;
        //ジョイスティックの回転
        //回転の値をcanidとcandlcのパラメータを格納
        auto msg_angular = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_angular->canid = can_angular_id;
        msg_angular->candlc = 4;
        //twistの型を変数に格納
        auto msg_gazebo = std::make_shared<geometry_msgs::msg::Twist>();

        msg joy_msg;

        
        
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

                return joy_msg;
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

                return joy_msg;
            }

        }
}

