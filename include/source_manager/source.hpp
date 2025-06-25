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

    // 设置状态
    void set_state(State new_state) {
        current_state_ = new_state;
    }

    // 切换状态
    virtual void switch_state(State new_state) = 0;

    // 状态切换后的回调函数
    virtual void handle_state_change() = 0;

protected:
    rclcpp::Node::SharedPtr node_;  // ROS 2 node
    State current_state_;           // current state
    State previous_state_;          // previous state

    bool state_changed_ = false; // state changed flag
    
};

#endif  // SOURCE_HPP