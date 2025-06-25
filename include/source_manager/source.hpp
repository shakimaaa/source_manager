#ifndef SOURCE_HPP
#define SOURCE_HPP

#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "navx_msgs/msg/global_position_int.hpp"

class Source
{
public:
    enum class State
    {
        UNINIT = 0,
        SLAM = 1,
        GPS = 2
    };

    Source(rclcpp::Node::SharedPtr node) : node_(node), current_state_(State::UNINIT) {}

    virtual ~Source() = default;

    // 获取当前状态
    State get_state() const;
    State get_prestate() const;

    bool is_state_changed()const;

    void set_state();

 

    // check if the localization source is available
    virtual void check_state_health() = 0;

    // 状态切换后的回调函数
    virtual void set_offset(double lastX, double lastY, double lastZ, double lastYAW) = 0;

protected:
    rclcpp::Node::SharedPtr node_;  // ROS 2 node



    State current_state_;           // current state
    State previous_state_;          // previous state

    bool state_changed_ = false; // state changed flag
    bool gps_available = false;  // GPS data availability
    bool slam_available = false; // SLAM data availability

    double last_pose_x = 0.0; // last pose x coordinate
    double last_pose_y = 0.0; // last pose y coordinate
    double last_pose_z = 0.0; // last pose z coordinate
    double last_yaw = 0.0;    // last yaw angle
    
};

#endif  // SOURCE_HPP