#pragma once

#include <rclcpp/rclcpp.hpp>
#include "source.hpp"
#include "slam.hpp"
#include "gps.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <thread>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include <nav_msgs/msg/path.hpp>
#include "xion_msg/srv/switch_source_type.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <rmw/qos_profiles.h>

class SourceManager : public rclcpp::Node
{
public:
    SourceManager();
    void init();

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr curr_odometry_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<GPS> gps_source_;
    std::unique_ptr<SLAM> slam_source_;
    std::shared_ptr<BaseData> shared_data_;

    SourceBase::State active_source_ = SourceBase::State::UNINIT;
    SourceBase::State previous_source_ = SourceBase::State::UNINIT;

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void timerCallback();
    void publishPropagateOdometry();
    void checkSourceHealth();
};