#include "source_manager/source.hpp"

State Source::get_state()const {
        return current_state_;
    }

State Source::get_prestate() const {
        return previous_state_;
    }

bool Source::is_state_changed() const {
        if(current_state_ != previous_state_) {
            state_changed_ = true;
        } else {
            state_changed_ = false;
        }
        return state_changed_;
    }

    

void Source::set_state() {
        if(gps_available) {
            current_state_ = State::GPS;
        } else if(slam_available) {
            current_state_ = State::SLAM;
        }
    }

void Source::publish() {
        nav_msgs::msg::Odometry position_msg;
        if(current_state_ == State::GPS) {
            last_pose_x = d_lat + offset_x;
            last_pose_y = d_lon + offset_y;
            last_pose_z = d_alt + offset_z;
            last_yaw = yaw_enu;
        } else if(current_state_ == State::SLAM) {
            last_pose_x = current_odometry_.pose.pose.position.x + offset_x;
            last_pose_y = current_odometry_.pose.pose.position.y + offset_y;
            last_pose_z = current_odometry_.pose.pose.position.z + offset_z;
            last_yaw = calculate_yaw(current_odometry_);
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Unknown state: %d", static_cast<int>(current_state_));
            return; // Exit if the state is unknown
        }
        position_msg.header.stamp = node_->now();
        position_msg.header.frame_id = "world";
        
        position_msg.pose.pose.position.x = last_pose_x;
        position_msg.pose.pose.position.y = last_pose_y;
        position_msg.pose.pose.position.z = last_pose_z;
        
        position_msg.pose.pose.orientation.x = 0.0; // Placeholder, should be set based on actual orientation
        position_msg.pose.pose.orientation.y = 0.0; // Placeholder, should be set based on actual orientation
        position_msg.pose.pose.orientation.z = sin(last_yaw / 2.0);
        position_msg.pose.pose.orientation.w = cos(last_yaw / 2.0);
        
        position_pub_->publish(position_msg);
    }