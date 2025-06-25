#ifndef GPS_HPP
#define GPS_HPP

#include "Source.hpp"
#include "navx_msgs/msg/global_position_int.hpp"

class GPS : public Source
{
public:
    GPS(rclcpp::Node::SharedPtr node) : Source(node)
    {
        gps_sub_ = node_->create_subscription<navx_msgs::msg::GlobalPositionInt>(
            "/mav/global_position_int", 10, std::bind(&GPS::gps_callback, this, std::placeholders::_1));
    }

private:
    void gps_callback(const navx_msgs::msg::GlobalPositionInt::SharedPtr msg)
    {
        if (!msg) {
            RCLCPP_ERROR(node_->get_logger(), "Received null GPS message");
            return;
        }

        // 处理GPS消息
        current_gps_position_ = *msg;
        evaluate_gps_health();
    }

    void evaluate_gps_health()
    {
        // 基于GPS数据评估健康状态
        if (current_gps_position_.lat == 0 || current_gps_position_.lon == 0) {
            RCLCPP_ERROR(node_->get_logger(), "GPS data is invalid.");
        } else {
            RCLCPP_INFO(node_->get_logger(), "GPS data is healthy.");
        }
    }

    navx_msgs::msg::GlobalPositionInt current_gps_position_;

    rclcpp::Subscription<navx_msgs::msg::GlobalPositionInt>::SharedPtr gps_sub_;

    bool origin_set_ = false;
    navx_msgs::msg::GlobalPositionInt origin_;
};

#endif  // GPS_HPP