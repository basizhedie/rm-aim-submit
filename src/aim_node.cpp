#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include "rm_aim_sim_bridge/msg/armor_corners_array.hpp"
#include "rm_aim_sim_bridge/msg/gimbal_state.hpp"
#include "rm_aim_sim_bridge/msg/telemetry.hpp"
#include "rm_aim_sim_bridge/msg/aim_command.hpp"

using namespace std::placeholders;

class AimNode : public rclcpp::Node {
public:
    AimNode() : Node("aim_node") { 
        // 1. 订阅三个话题
        sub_armors_ = this->create_subscription<rm_aim_sim_bridge::msg::ArmorCornersArray>(
            "/aim_sim/armors", 10, std::bind(&AimNode::armor_callback, this, _1));
        sub_gimbal_ = this->create_subscription<rm_aim_sim_bridge::msg::GimbalState>(
            "/aim_sim/gimbal", 10, std::bind(&AimNode::gimbal_callback, this, _1));
        sub_telemetry_ = this->create_subscription<rm_aim_sim_bridge::msg::Telemetry>(
            "/aim_sim/telemetry", 10, std::bind(&AimNode::telemetry_callback, this, _1));

        // 2. 发布指令话题
        pub_cmd_ = this->create_publisher<rm_aim_sim_bridge::msg::AimCommand>("/aim_sim/command", 10);
        
        RCLCPP_INFO(this->get_logger(), "C++ 自瞄节点已启动！等待数据...");
    }

private:
    // 声明订阅者和发布者
    rclcpp::Subscription<rm_aim_sim_bridge::msg::ArmorCornersArray>::SharedPtr sub_armors_;
    rclcpp::Subscription<rm_aim_sim_bridge::msg::GimbalState>::SharedPtr sub_gimbal_;
    rclcpp::Subscription<rm_aim_sim_bridge::msg::Telemetry>::SharedPtr sub_telemetry_;
    rclcpp::Publisher<rm_aim_sim_bridge::msg::AimCommand>::SharedPtr pub_cmd_;

    // 声明状态变量（先随便给个初值）
    double current_yaw_ = 0.0;
    double current_pitch_ = 0.0;
    double bullet_speed_ = 23.0;
    double gravity_ = 9.78;
    rclcpp::Time last_fire_time_{0,0,RCL_ROS_TIME};

    // --- 以下是三个回调函数 ---
    
    void telemetry_callback(const rm_aim_sim_bridge::msg::Telemetry::SharedPtr msg) {
        bullet_speed_ = msg->speed;
        gravity_ = msg->gravity;
    }

    void gimbal_callback(const rm_aim_sim_bridge::msg::GimbalState::SharedPtr msg) {
        current_yaw_ = msg->yaw;
        current_pitch_ = msg->pitch;
    }

    void armor_callback(const rm_aim_sim_bridge::msg::ArmorCornersArray::SharedPtr msg) {
        if (msg->armors.empty()) return; // 没目标直接退出

        auto corners = msg->armors[0].corners;

        double cx = 0.0,cy = 0.0,cz = 0.0;

        for (int i = 0; i < 4; i++)
        {
            cx += corners[i].x;
            cy += corners[i].y;
            cz += corners[i].z;
        }
        cx /= 4.0; cy /= 4.0; cz /= 4.0;


        
        
       // 1. 算目标 Yaw
        double target_yaw = std::atan2(cy, cx);

        // 2. 枪管长度补偿 (默认0.22m)
        double barrel_length = 0.22; 
        
        // 算出枪口在三维空间中的实际位置 (子弹是从这里出发的)
        double gun_x = barrel_length * std::cos(target_yaw);
        double gun_y = barrel_length * std::sin(target_yaw);

        // 目标相对于枪口的实际水平距离
        double dx = cx - gun_x;
        double dy = cy - gun_y;
        double distance = std::sqrt(dx * dx + dy * dy);

        // 3. 重力补偿 (用新的距离算)
        double flight_time = distance / bullet_speed_; 
        double drop = 0.5 * gravity_ * flight_time * flight_time;
        double target_pitch = std::atan2(cz + drop, distance);

        

        auto cmd = rm_aim_sim_bridge::msg::AimCommand();

        cmd.header.stamp = msg->header.stamp;
        cmd.header.frame_id = msg->header.frame_id;

        cmd.session_id = msg->session_id;

        cmd.yaw = target_yaw;
        cmd.pitch = target_pitch;
        cmd.enable = true;

        double yaw_error = std::abs(target_yaw - current_yaw_);
        double pitch_error = std::abs(target_pitch - current_pitch_);

        if (yaw_error < 0.1 && pitch_error < 0.1){

            cmd.fire = true;
        }
        else{
            cmd.fire = false;
        }

        rclcpp::Time now = this->now();

        double time_since_last = (now - last_fire_time_).seconds();

        if (last_fire_time_.nanoseconds() == 0 || time_since_last >0.1)
        {
            pub_cmd_->publish(cmd);
            last_fire_time_ = now;
        }
        

        static int count = 0;
        if (count++ % 30 ==0){
            RCLCPP_INFO(this->get_logger(), "发送指令： Yaw=%.2f,Pitch=%.2f, Fire=%d", cmd.yaw, cmd.pitch, cmd.fire);
        }

        
        
        
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AimNode>();
    
    // 【添加这行】强制将 use_sim_time 设为 true，不听命令行的
    node->set_parameter(rclcpp::Parameter("use_sim_time", true));

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}