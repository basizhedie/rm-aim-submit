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
        double min_yaw_diff = 9999.0; 
        bool found = false;

        for (const auto& armor : msg->armors) {
            double temp_cx = 0, temp_cy = 0, temp_cz = 0;
            for (int i = 0; i < 4; i++) {
                temp_cx += armor.corners[i].x;
                temp_cy += armor.corners[i].y;
                temp_cz += armor.corners[i].z;
            }
            temp_cx /= 4.0; temp_cy /= 4.0; temp_cz /= 4.0;


            double temp_yaw = std::atan2(temp_cy, temp_cx);
            double yaw_diff = std::abs(temp_yaw - current_yaw_);

    
            if (yaw_diff < min_yaw_diff) {
                min_yaw_diff = yaw_diff;
                cx = temp_cx; cy = temp_cy; cz = temp_cz;
                found = true;
            }
        }
        if (!found) return;
        is_first_frame_ = false; 


        double current_time_sec = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        if (!is_first_frame_) {
            double dt = current_time_sec - last_armor_time_sec;
            if (dt > 0.01 && dt < 0.1) {
                double raw_vx = (cx - last_cx_) / dt;
                double raw_vy = (cy - last_cy_) / dt;


                if (raw_vx > 4.0) raw_vx = 4.0;
                if (raw_vx < -4.0) raw_vx = -4.0;
                if (raw_vy > 4.0) raw_vy = 4.0;
                if (raw_vy < -4.0) raw_vy = -4.0;

                // 滤波
                smoothed_vx_ = smoothed_vx_ * 0.3 + raw_vx * 0.7;
                smoothed_vy_ = smoothed_vy_ * 0.3 + raw_vy * 0.7;
            }
        } else {
            is_first_frame_ = false;
        }
        last_cx_ = cx; last_cy_ = cy; last_armor_time_sec = current_time_sec;


        double initial_distance = std::sqrt(cx * cx + cy * cy);
        double flight_time = (initial_distance / bullet_speed_) + 0.015; 


        double offset_x = smoothed_vx_ * flight_time * 1.0;
        double offset_y = smoothed_vy_ * flight_time * 1.0;


        double max_predict = 0.8; 
        if (offset_x > max_predict) offset_x = max_predict;
        if (offset_x < -max_predict) offset_x = -max_predict;
        if (offset_y > max_predict) offset_y = max_predict;
        if (offset_y < -max_predict) offset_y = -max_predict;

        double predicted_cx = cx + offset_x;
        double predicted_cy = cy + offset_y;

        double target_yaw = std::atan2(predicted_cy, predicted_cx);
        double barrel_length = 0.22; 
        double gun_x = barrel_length * std::cos(target_yaw);
        double gun_y = barrel_length * std::sin(target_yaw);
        double dx = predicted_cx - gun_x;
        double dy = predicted_cy - gun_y;
        double distance = std::sqrt(dx * dx + dy * dy);

        double flight_time_final = (distance / bullet_speed_) + 0.015; 
        double drop = 0.5 * gravity_ * flight_time_final * flight_time_final * 0.95; 
        double target_pitch = std::atan2(cz + drop, distance);


        auto cmd = rm_aim_sim_bridge::msg::AimCommand();
        cmd.header = msg->header;
        cmd.session_id = msg->session_id;
        
         double yaw_diff = target_yaw - current_yaw_;
        double pitch_diff = target_pitch - current_pitch_;

        double max_turn = 0.2; 
        if (yaw_diff > max_turn) yaw_diff = max_turn;
        if (yaw_diff < -max_turn) yaw_diff = -max_turn;
        if (pitch_diff > max_turn) pitch_diff = max_turn;
        if (pitch_diff < -max_turn) pitch_diff = -max_turn;

        cmd.yaw = current_yaw_ + yaw_diff;
        cmd.pitch = current_pitch_ + pitch_diff;
        cmd.enable = true;
        cmd.pitch = target_pitch;
        cmd.enable = true;

        double yaw_error = std::abs(target_yaw - current_yaw_);
        double pitch_error = std::abs(target_pitch - current_pitch_);

        // 开火阈值 0.1
        if (yaw_error < 0.1 && pitch_error < 0.1) {
            cmd.fire = true;
        } else {
            cmd.fire = false;
        }

        if (current_time_sec - last_fire_time_sec_ > 0.1) {
            pub_cmd_->publish(cmd);
            last_fire_time_sec_ = current_time_sec; 
        }
    }
};
        
        
    


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AimNode>();
    

    node->set_parameter(rclcpp::Parameter("use_sim_time", true));

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

