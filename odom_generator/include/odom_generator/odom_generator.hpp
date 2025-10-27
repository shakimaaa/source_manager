# pragma once

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "xion_msg/msg/global_position_int.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>

class OdomGenerator : public rclcpp::Node {
public:
    OdomGenerator();

private:
    // void timerCallback();
    void gpsCallback(const xion_msg::msg::GlobalPositionInt::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

    rclcpp::Subscription<xion_msg::msg::GlobalPositionInt>::SharedPtr gps_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Odometry odom_msg;
    xion_msg::msg::GlobalPositionInt origin_;
    bool origin_set_ = false;

    Eigen::Quaterniond imu_orientation_;
    Eigen::Vector3d imu_angular_velocity_;

    int fix_type_ = 0;
    bool not_pub = false;
};