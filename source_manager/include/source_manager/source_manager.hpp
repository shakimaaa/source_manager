#pragma once

#include <rclcpp/rclcpp.hpp>
#include "source.hpp"
#include "slam.hpp"
#include "gps.hpp"
#include "util.hpp"
#include "util.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <thread>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include <std_srvs/srv/set_bool.hpp>
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
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr propagate_odometry_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    std::vector<geometry_msgs::msg::PoseStamped> path_history_;

    // =========================
    // Switch source service
    // =========================
    rclcpp::Service<xion_msg::srv::SwitchSourceType>::SharedPtr switch_source_srv_;
    std::optional<SourceBase::State> manual_request_;
    bool manual_pin_active_ = false;
    SourceBase::State manual_pin_target_ = SourceBase::State::UNINIT;

    //===========================
    // Lifecycle Management
    //===========================
    // rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr lifecycle_client_;
    // rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr get_state_client_;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_restart_req_;
    bool restart_srv = false;

    // =========================
    // Smooth Transition Support
    // =========================
    rclcpp::Time transition_start_time_;
    bool is_transitioning_ = false;
    // Duration of transition in seconds (1.0 means 1 second)
    const double transition_duration_sec_ = 0.5;
    Eigen::Vector3d position_start_;   // From
    Eigen::Vector3d position_target_;  // To
    Eigen::Quaterniond orientation_start_;  // From
    Eigen::Quaterniond orientation_target_; // To


    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<GPS> gps_source_;
    std::shared_ptr<SLAM> slam_source_;

    SourceBase::State active_source_ = SourceBase::State::UNINIT;
    SourceBase::State previous_source_ = SourceBase::State::UNINIT;

    std::vector<int64_t> pri_raw;
    std::vector<SourceBase::State> priority_source_{SourceBase::State::GPS,  SourceBase::State::SLAM};  // 0: UNINIT, 1:SLAM, 2:GPS

    NavState current_propagate_state_;
    NavState current_odometry_state_;
    Eigen::Vector3d current_position_;
    Eigen::Vector3d current_velocity_;
    Eigen::Vector3d current_angular_acceleration_;
    Eigen::Quaterniond current_orientation_;
    double current_yaw_ = 0.0;

    bool is_state_changed_ = false;

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void timerCallback();
    void publishPropagateOdometry();
    void publishOdometry();
    void update();
    void checkSourceHealth();
    void changeSourceType();

    void raisePriority(SourceBase::State target);
    bool canRaisePriority(SourceBase::State target, std::string &reason) const;
    void onSwitchSource(
        const std::shared_ptr<xion_msg::srv::SwitchSourceType::Request> req,
        std::shared_ptr<xion_msg::srv::SwitchSourceType::Response> res);
    // void changeSourcefromsrv();
    void onSetRestartReq(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                         std::shared_ptr<std_srvs::srv::SetBool::Response> resp);
    void restartCheck();

    void requestLifecycleRestart();
    void onSlamRestartRequested();

    void interpolationFilter();

    std::shared_ptr<SourceBase> getSource(SourceBase::State state);

};

static inline const char* stateToString(SourceBase::State s) {
    switch (s) {
        case SourceBase::State::GPS: return "GPS";
        case SourceBase::State::SLAM: return "SLAM";
        case SourceBase::State::UNINIT: return "UNINIT";
        default: return "UNKNOWN";
    }
}