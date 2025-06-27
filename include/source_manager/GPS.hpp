#ifndef GPS_HPP
#define GPS_HPP

#include "Source.hpp"
#include "navx_msgs/msg/global_position_int.hpp"
#include <cmath>

class GPS : public Source
{
public:
    GPS(rclcpp::Node::SharedPtr node) : Source(node)
    {
        gps_sub_ = node_->create_subscription<navx_msgs::msg::GlobalPositionInt>(
            "/mav/global_position_int", 10, std::bind(&GPS::gps_callback, this, std::placeholders::_1));
    }

private:
    void gps_callback(const navx_msgs::msg::GlobalPositionInt::SharedPtr msg);

    void check_state_health() override;

    void set_offset(double lastX, double lastY, double lastZ, double lastYAW) override;


    navx_msgs::msg::GlobalPositionInt current_mavlink_position_;

    rclcpp::Subscription<navx_msgs::msg::GlobalPositionInt>::SharedPtr gps_sub_;

    bool origin_set_ = false;
    navx_msgs::msg::GlobalPositionInt origin_;

    int fix_type_;

    double d_lat;
    double d_lon;
    double d_alt;
    double yaw_enu;

    double received_time_ = 0.0; // Time when the GPS data was received

    double offset_x = 0.0;
    double offset_y = 0.0;
    double offset_z = 0.0;  
    double offset_yaw = 0.0;
    
};

#endif  // GPS_HPP