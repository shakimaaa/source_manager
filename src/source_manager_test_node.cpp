# include "source_manager/source_manager_test_node.hpp"

SourceManagerNode::SourceManagerNode()
  : Node("source_manager_test_node")
{
    // Initialize the source, GPS, and SLAM objects
    source_ = std::make_shared<Source>(shared_from_this());
    gps_ = std::make_shared<GPS>(shared_from_this());
    slam_ = std::make_shared<SLAM>(shared_from_this()); 
    position_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/position", 10);
    RCLCPP_INFO(this->get_logger(), "SourceManagerNode initialized successfully!");

     timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / 15), 
            std::bind(&SourceManagerNode::timer_callback, this)
        );
}

void SourceManagerNode::timer_callback()
{
    source_->set_state();

    if(source_->is_state_changed()) {
        source_->state_changed_ = false; // Reset the state changed flag
        handle_state_change();
        RCLCPP_INFO(this->get_logger(), "State changed to: %d", static_cast<int>(source_->get_state()));
    }

    publish_odom();
}

void SourceManagerNode::handle_state_change()
{
    if(source_->get_state() == Source::State::GPS) {
        gps_->set_offset(source_->last_pose_x, source_->last_pose_y, source_->last_pose_z, source_->last_yaw);
    } else if(source_->get_state() == Source::State::SLAM) {
        slam_->set_offset(source_->last_pose_x, source_->last_pose_y, source_->last_pose_z, source_->last_yaw);
    } else if(source_->get_state() == Source::State::UNINIT) {
        RCLCPP_WARN(this->get_logger(), "Source is uninitialized, cannot handle state change.");
    }
}

void SourceManagerNode::publish_odom()
{
    nav_msgs::msg::Odometry position_msg;

    if(source_->get_state() == Source::State::GPS) {
        position_msg.pose.pose.position.x = gps_->d_lon + gps_->offset_x;
        position_msg.pose.pose.position.y = gps_->d_lat + gps_->offset_y;
        position_msg.pose.pose.position.z = gps_->d_alt + gps_->offset_z;
        position_msg.pose.pose.orientation.z = sin(gps_->yaw_enu / 2.0);
        position_msg.pose.pose.orientation.w = cos(gps_->yaw_enu / 2.0);
    } else if(source_->get_state() == Source::State::SLAM) {
        position_msg.pose.pose.position.x = slam_->current_odometry_.pose.pose.position.x + slam_->offset_x;
        position_msg.pose.pose.position.y = slam_->current_odometry_.pose.pose.position.y + slam_->offset_y;
        position_msg.pose.pose.position.z = slam_->current_odometry_.pose.pose.position.z + slam_->offset_z;
        position_msg.pose.pose.orientation.x = slam_->current_odometry_.pose.pose.orientation.x;
        position_msg.pose.pose.orientation.y = slam_->current_odometry_.pose.pose.orientation.y;    
        position_msg.pose.pose.orientation.z = slam_->current_odometry_.pose.pose.orientation.z;
        position_msg.pose.pose.orientation.w = slam_->current_odometry_.pose.pose.orientation.w;
        position_msg.twist.twist.linear.x = slam_->current_odometry_.twist.twist.linear.x;
        position_msg.twist.twist.linear.y = slam_->current_odometry_.twist.twist.linear.y;    
        position_msg.twist.twist.linear.z = slam_->current_odometry_.twist.twist.linear.z;
    }else if(source_->get_state() == Source::State::UNINIT) {
        RCLCPP_WARN(this->get_logger(), "Source is uninitialized, cannot publish position."); 
        return; // Exit if the source is uninitialized
    }
    position_pub_->publish(position_msg);

    source_->last_pose_x = position_msg.pose.pose.position.x;
    source_->last_pose_y = position_msg.pose.pose.position.y;
    source_->last_pose_z = position_msg.pose.pose.position.z;
    source_->last_yaw = std::atan2(position_msg.pose.pose.orientation.z, position_msg.pose.pose.orientation.w);
    source->last_pub_time = this->now().seconds();
}