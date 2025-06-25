#ifndef SLAM_HPP
#define SLAM_HPP

#include "Source.hpp"
#include "nav_msgs/msg/odometry.hpp"

class SLAM : public Source
{
public:
    SLAM(rclcpp::Node::SharedPtr node) : Source(node)
    {
        slam_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry", 10, std::bind(&SLAM::slam_callback, this, std::placeholders::_1));
    }

private:
    void slam_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    void check_state_health() override;

    void set_offset(double lastX, double lastY, double lastZ, double lastYAW) override;

    double calculate_yaw(const nav_msgs::msg::Odometry &odometry) const
    {
        // Calculate yaw from the orientation quaternion in the ENU frame.
        double siny_cosp = 2 * (odometry.pose.pose.orientation.w * odometry.pose.pose.orientation.z +
                                odometry.pose.pose.orientation.x * odometry.pose.pose.orientation.y);
        double cosy_cosp = 1 - 2 * (odometry.pose.pose.orientation.y * odometry.pose.pose.orientation.y +
                                    odometry.pose.pose.orientation.z * odometry.pose.pose.orientation.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    nav_msgs::msg::Odometry current_odometry_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr slam_sub_;

    bool origin_set_ = false;
    nav_msgs::msg::Odometry origin_;

    double offset_x = 0.0;
    double offset_y = 0.0;
    double offset_z = 0.0;
    double offset_yaw = 0.0;
};

#endif  // SLAM_HPP