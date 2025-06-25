#include "source_manager/slam.hpp"


void SLAM::slam_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_odometry_ = *msg;

    if(!origin_set_) {
        origin_ = *msg;
        origin_set_ = true;
        return;
    }

    check_state_health();
    
}


void SLAM::check_state_health()
{
    // Check if the SLAM data is valid
    if (current_odometry_.pose.pose.position.x == 0.0 &&
        current_odometry_.pose.pose.position.y == 0.0 &&
        current_odometry_.pose.pose.position.z == 0.0)
    {
        slam_available = false;
    } else {
        slam_available = true;
    }
}

void SLAM::set_offset(double lastX, double lastY, double lastZ, double lastYAW)
{
    offset_x = lastX - current_odometry_.pose.pose.position.x;
    offset_y = lastY - current_odometry_.pose.pose.position.y;
    offset_z = lastZ - current_odometry_.pose.pose.position.z;
    offset_yaw = lastYAW - calculate_yaw(current_odometry_);
}