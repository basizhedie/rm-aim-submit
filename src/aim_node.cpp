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
        
        sub_armors_ = this->create_subscription<rm_aim_sim_bridge::msg::ArmorCornersArray>(
            "/aim_sim/armors", 10, std::bind(&AimNode::armor_callback, this, _1));
        sub_gimbal_ = this->create_subscription<rm_aim_sim_bridge::msg::GimbalState>(
            "/aim_sim/gimbal", 10, std::bind(&AimNode::gimbal_callback, this, _1));
        sub_telemetry_ = this->create_subscription<rm_aim_sim_bridge::msg::Telemetry>(
            "/aim_sim/telemetry", 10, std::bind(&AimNode::telemetry_callback, this, _1));

        pub_cmd_ = this->create_publisher<rm_aim_sim_bridge::msg::AimCommand>("/aim_sim/command", 10);
        
        RCLCPP_INFO(this->get_logger(), "C++ 自瞄节点已启动！等待数据...");
    }

private:
    rclcpp::Subscription<rm_aim_sim_bridge::msg::ArmorCornersArray>::SharedPtr sub_armors_;
    rclcpp::Subscription<rm_aim_sim_bridge::msg::GimbalState>::SharedPtr sub_gimbal_;
    rclcpp::Subscription<rm_aim_sim_bridge::msg::Telemetry>::SharedPtr sub_telemetry_;
    rclcpp::Publisher<rm_aim_sim_bridge::msg::AimCommand>::SharedPtr pub_cmd_;

    double current_yaw_ = 0.0;
    double current_pitch_ = 0.0;
    double bullet_speed_ = 23.0;
    double gravity_ = 9.78;
    double last_fire_time_sec_ = 0.0;
    double last_cx_ = 0.0;
    double last_cy_ = 0.0;
    bool is_first_frame_ = true;
    double last_armor_time_sec =0.0;

    double smoothed_vx_ = 0.0;
    double smoothed_vy_ = 0.0;

    
    void telemetry_callback(const rm_aim_sim_bridge::msg::Telemetry::SharedPtr msg) {
        bullet_speed_ = msg->speed;
        gravity_ = msg->gravity;
    }

    void gimbal_callback(const rm_aim_sim_bridge::msg::GimbalState::SharedPtr msg) {
        current_yaw_ = msg->yaw;
        current_pitch_ = msg->pitch;
    }

    void armor_callback(const rm_aim_sim_bridge::msg::ArmorCornersArray::SharedPtr msg) {
        if (msg->armors.empty()) {
            is_first_frame_ = true; 
            return;
        }


        double cx = 0.0, cy = 0.0, cz = 0.0;
        double max_score = -9999.0; // 用打分制
        bool found = false;

        for (const auto& armor : msg->armors) {
            // 1. 算中心点
            double temp_cx = 0, temp_cy = 0, temp_cz = 0;
            for (int i = 0; i < 4; i++) {
                temp_cx += armor.corners[i].x;
                temp_cy += armor.corners[i].y;
                temp_cz += armor.corners[i].z;
            }
            temp_cx /= 4.0; temp_cy /= 4.0; temp_cz /= 4.0;

            auto dist_sq = [](auto p1, auto p2) {
                return (p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z);
            };
            double d01 = dist_sq(armor.corners[0], armor.corners[1]);
            double d12 = dist_sq(armor.corners[1], armor.corners[2]);
            double d23 = dist_sq(armor.corners[2], armor.corners[3]);
            double d30 = dist_sq(armor.corners[3], armor.corners[0]);
            double width = std::sqrt(std::max({d01, d12, d23, d30}));

            double score = width * 10.0; 
            

            if (!is_first_frame_) {
                double dist_to_last = std::sqrt(std::pow(temp_cx - last_cx_, 2) + std::pow(temp_cy - last_cy_, 2));
                if (dist_to_last < 0.3) {
                    score += 0.5; 
                }
            }

            if (score > max_score) {
                max_score = score;
                cx = temp_cx; cy = temp_cy; cz = temp_cz;
                found = true;
            }
        }

        if (!found) return; 
        is_first_frame_ = false;   

        // ================== 2. 测速与滤波 ==================
       double current_time_sec = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        
        if (!is_first_frame_) {
            double dt = current_time_sec - last_armor_time_sec;
            if (dt > 0.01 && dt < 0.04) {
                double raw_vx = (cx - last_cx_) / dt;
                double raw_vy = (cy - last_cy_) / dt;

            

                // 正确的限幅
                if (raw_vx > 2.5) raw_vx = 2.5;
                if (raw_vx < -2.5) raw_vx = -2.5;
                if (raw_vy > 2.5) raw_vy = 2.5;
                if (raw_vy < -2.5) raw_vy = -2.5;

                // 变向反冲清零
                if (raw_vx * smoothed_vx_ < 0) smoothed_vx_ = 0;
                if (raw_vy * smoothed_vy_ < 0) smoothed_vy_ = 0;

                // 平滑滤波 (0.3 历史 + 0.7 新数据)
                smoothed_vx_ = smoothed_vx_ * 0.3 + raw_vx * 0.7;
                smoothed_vy_ = smoothed_vy_ * 0.3 + raw_vy * 0.7;
            }
        } else {
            
        }

        // 更新历史数据
        last_cx_ = cx; last_cy_ = cy; last_armor_time_sec = current_time_sec;

        // ================== 3. 预判计算 ==================
        double initial_distance = std::sqrt(cx * cx + cy * cy);
        // 文档里的延迟 0.015s
        double flight_time = (initial_distance / bullet_speed_) + 0.015;
        
        // 限制预测量在 1.2 米以内
        double max_predict_dist = 1.2;
        double offset_x = smoothed_vx_ * flight_time * 1.1; // 微调系数 1.1
        double offset_y = smoothed_vy_ * flight_time * 1.1;

        if (offset_x > max_predict_dist) offset_x = max_predict_dist;
        if (offset_x < -max_predict_dist) offset_x = -max_predict_dist;
        if (offset_y > max_predict_dist) offset_y = max_predict_dist;
        if (offset_y < -max_predict_dist) offset_y = -max_predict_dist;

        double predicted_cx = cx + offset_x;
        double predicted_cy = cy + offset_y;

        // ================== 4. 算 Yaw 和 Pitch ==================
        double target_yaw = std::atan2(predicted_cy, predicted_cx);
        
        // 枪管补偿
        double barrel_length = 0.22; 
        double gun_x = barrel_length * std::cos(target_yaw);
        double gun_y = barrel_length * std::sin(target_yaw);
        double dx = predicted_cx - gun_x;
        double dy = predicted_cy - gun_y;
        double distance = std::sqrt(dx * dx + dy * dy);

        // 重力补偿
        double flight_time_final = (distance / bullet_speed_) + 0.015; 
        double drop = 0.5 * gravity_ * flight_time_final * flight_time_final * 0.95; // 0.9防打高
        double target_pitch = std::atan2(cz + drop, distance);

        auto cmd = rm_aim_sim_bridge::msg::AimCommand();
        cmd.header = msg->header;
        cmd.session_id = msg->session_id;
        cmd.yaw = target_yaw;
        cmd.pitch = target_pitch;
        cmd.enable = true;

        double yaw_error = std::abs(target_yaw- current_yaw_);
        double pitch_error = std::abs(target_pitch - current_pitch_);

        if (yaw_error < 0.06 && pitch_error < 0.06) {
            cmd.fire = true;
        } else {
            cmd.fire = false;
        }

        // 10Hz 频率控制发送
         
        
        if (current_time_sec - last_fire_time_sec_ > 0.1) {
            pub_cmd_->publish(cmd);
            last_fire_time_sec_ = current_time_sec; 
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

