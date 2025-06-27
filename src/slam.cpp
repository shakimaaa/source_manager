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
    offset_x = lacurrent_odometry_.pose.pose.position.xstX - lastX;
    offset_y = current_odometry_.pose.pose.position.y - lastY;
    offset_z = current_odometry_.pose.pose.position.z - lastZ;
    offset_yaw = calculate_yaw(current_odometry_) - lastYAW;
    
}

void SLAM::calculate_yaw(const nav_msgs::msg::Odometry &odometry) const
{
    // Calculate yaw from the orientation quaternion in the ENU frame.
    double siny_cosp = 2 * (odometry.pose.pose.orientation.w * odometry.pose.pose.orientation.z +
                            odometry.pose.pose.orientation.x * odometry.pose.pose.orientation.y);
    double cosy_cosp = 1 - 2 * (odometry.pose.pose.orientation.y * odometry.pose.pose.orientation.y +
                                odometry.pose.pose.orientation.z * odometry.pose.pose.orientation.z);
    return std::atan2(siny_cosp, cosy_cosp);
}