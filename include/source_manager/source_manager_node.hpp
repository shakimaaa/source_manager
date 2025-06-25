#ifndef SOURCE_MANAGER_NODE_HPP_
#define SOURCE_MANAGER_NODE_HPP_
#pragma once
#include <rclcpp/rclcpp.hpp> // ROS 2 core library
#include "nav_msgs/msg/odometry.hpp"
#include "navx_msgs/msg/global_position_int.hpp"
#include <std_msgs/msg/string.hpp>
#include <cmath> // For mathematical operations

class SourceManagerNode : public rclcpp::Node
{
public:
    enum class State
    {
        UNINIT,
        SLAM,
        GPS
    };

    SourceManagerNode();

private:
    void timer_callback();
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void mavlink_callback(const navx_msgs::msg::GlobalPositionInt::SharedPtr msg);

    void publish_position(State state);
    double calculate_yaw(const nav_msgs::msg::Odometry &position);


    State get_state(int fix_type);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<navx_msgs::msg::GlobalPositionInt>::SharedPtr mavlink_sub_; 
    rclcpp::TimerBase::SharedPtr timer_;
   
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr position_pub_;

    nav_msgs::msg::Odometry current_odometry_;
    nav_msgs::msg::Odometry last_odometry_;
    nav_msgs::msg::Odometry origin_odometry_;

    navx_msgs::msg::GlobalPositionInt current_mavlink_position_;
    // navx_msgs::msg::GlobalPositionInt last_mavlink_position_;
    navx_msgs::msg::GlobalPositionInt origin_;

    rclcpp::Time last_slam_time_;  
    rclcpp::Time last_gps_time_;   

    bool origin_set_ = false;
    bool mode_changed_ = false;
    bool odometry_origin_set_ = false;

    int fix_type_;

    double d_lat;
    double d_lon;
    double d_alt;
    double yaw_enu;
    double last_slam_yaw_;
    double last_gps_yaw_;
    double last_lat_;
    double last_lon_;
    double last_alt_;

    State current_state_;
    State previous_state_ = State::UNINIT;
};

#endif // SOURCE_MANAGER_NODE_HPP_