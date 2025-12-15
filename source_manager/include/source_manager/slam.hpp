#pragma once


#include "source_manager/source.hpp"
#include "source_manager/util.hpp"
#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "xion_msg/msg/extended_odometry.hpp"
#include <Eigen/Core>
#include <atomic>
#include <Eigen/Geometry>  
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/msg/state.hpp"

// struct SLAMData {
//     Eigen::Vector3d p{Eigen::Vector3d::Zero()};
//     Eigen::Vector3d v{Eigen::Vector3d::Zero()};
//     Eigen::Vector3d a{Eigen::Vector3d::Zero()};
//     Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};
//     double yaw{0.0};

//     Eigen::Vector3d Ba{Eigen::Vector3d::Zero()};
//     Eigen::Vector3d Bg{Eigen::Vector3d::Zero()};
// };

class SLAM : public SourceBase
{
public:
    explicit SLAM(rclcpp::Node::SharedPtr node);
    void setImudata(const Eigen::Vector3d& linearAcceleration,
                    const Eigen::Vector3d& angularVelocity,
                    const Eigen::Quaterniond& orientation,
                    const rclcpp::Time imu_time_stamp) override;
    void setHealthy(bool healthy) override;
    void setCurrPose(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, const Eigen::Quaterniond& q) override;
    bool isHealthy() override;
    void setOffset(const Eigen::Vector3d& p, const Eigen::Quaterniond& q, double yaw) override;
    NavState getOdometry()const override;
    NavState getPropagateOdometry() const override;
    bool canRestart() override;
    void restartSource() override;

    // bool extractImuInterval_(double t0, double t1, std::vector<ImuLite>& out) override;
    // ImuLite interpImu_(const ImuLite& a, const ImuLite& b, double t) override;
    // void integrateIntervalMidpoint_(const std::vector<ImuLite>& seg, Eigen::Vector3d& dvel_imu) override;

    
    

    void setSlamdata();
    void setOdometry();
    void setRestartOffset(NavState latest_slam_data);
    // void setOffset(const Eigen::Vector3d& p, const Eigen::Quaterniond& q, double yaw);
    
    bool restart();
    void setRestartquest(bool restart);

private:
    rclcpp::CallbackGroup::SharedPtr slam_callback_group_;
    rclcpp::Subscription<xion_msg::msg::ExtendedOdometry>::SharedPtr slam_sub_;
    rclcpp::TimerBase::SharedPtr slam_timer_;


    NavState odom_data_;
    NavState propageted_data_;

    // Eigen::Vector3d imu_acc_ = Eigen::Vector3d::Zero();
    // Eigen::Vector3d imu_gyro_ = Eigen::Vector3d::Zero();
    // Eigen::Quaterniond imu_orientation_ = Eigen::Quaterniond::Identity();

    // Just received slam data
    Eigen::Vector3d r_slam_p_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d r_slam_v_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d r_slam_a_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond r_slam_q_ = Eigen::Quaterniond::Identity();
    double r_slam_yaw_ = 0.0;
    rclcpp::Time timestamp;

    // latest slam data
    Eigen::Vector3d latest_slam_p_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d latest_slam_v_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d latest_slam_a_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond latest_slam_q_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d latest_Ba_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d latest_Bg_ = Eigen::Vector3d::Zero();
    double latest_slam_yaw_ = 0.0;

    // offset
    Eigen::Vector3d slam_offset_p_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond slam_offset_q_ = Eigen::Quaterniond::Identity();
    double slam_offset_yaw_ = 0.0;
    Eigen::Vector3d restart_offset_p_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond restart_offset_q_ = Eigen::Quaterniond::Identity();
    double restart_offset_yaw_ = 0.0;

    // imu propagate
    Eigen::Vector3d latest_slam_acc_0 = Eigen::Vector3d::Zero();
    Eigen::Vector3d latest_slam_gyr_0 = Eigen::Vector3d::Zero();
    // Eigen::Vector3d g_ = Eigen::Vector3d(0, 0, 9.81); // Gravity constant

    // 积分
    // Eigen::Vector3d integrated_v_imu_ = Eigen::Vector3d::Zero();
    double maxSpeeddiff_ = 3.0;
    double maxAnglediff_ = 100.0;
    Eigen::Vector3d last_integrated_v_imu_ = Eigen::Vector3d::Zero();

    // restart
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr lifecycle_client_;
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr get_state_client_;
    bool can_restart_ = true;
    std::atomic_bool restarting_{false};
    double restart_time_threshold_ = 0.0;
    
    
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_set_restart_req_;
    bool restart_srv = false;
    bool restart_requested_ = false;
    bool runing = false;
    bool is_received_message_ = false;
    rclcpp::Time unhealthy_start_time_;

    bool SLAM_healthy_ = false;
    bool receiving_slam_ = false;

    

    rclcpp::Time last_get_slam_time_;
    rclcpp::Time last_slam_time_;
    rclcpp::Time last_propagate_time_;
    rclcpp::Time propagate_time_;


    void slamCallback(const xion_msg::msg::ExtendedOdometry::SharedPtr msg);
    void timerCallback();

    void onSetRestartReq(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                         std::shared_ptr<std_srvs::srv::SetBool::Response> resp);
    void restartRequested();
    
};              