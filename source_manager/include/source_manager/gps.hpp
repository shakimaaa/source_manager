#pragma once 

#include "source.hpp"
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>                  
#include <Eigen/Geometry> 
#include <nav_msgs/msg/odometry.hpp>
#include "xion_msg/msg/global_position_int.hpp"

class GPS : public SourceBase 
{
public: 
    explicit GPS(rclcpp::Node::SharedPtr node,
                 std::shared_ptr<BaseData> data);

    void setImudata(const Eigen::Vector3d& linearAcceleration,
                    const Eigen::Vector3d& angularVelocity,
                    const Eigen::Quaterniond& orientation) override;
    void setHealthy(bool healthy) override;
    void setCurrPose(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, const Eigen::Quaterniond& q) override;
    bool isHealthy() override;

    void setGpsdata();

private:
    rclcpp::CallbackGroup::SharedPtr gps_callback_group_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_sub_;
    rclcpp::TimerBase::SharedPtr gps_timer_;
    
    Eigen::Vector3d imu_acc_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d imu_gyro_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond imu_orientation_ = Eigen::Quaterniond::Identity();

    // Just received  gps data
    Eigen::Vector3d r_gps_p_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d r_gps_v_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d r_gps_a_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond r_gps_q_ = Eigen::Quaterniond::Identity();
    double r_gps_yaw_ = 0.0;

    // latest gps data
    Eigen::Vector3d latest_gps_p_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d latest_gps_v_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d latest_gps_a_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond latest_gps_q_ = Eigen::Quaterniond::Identity();
    double latest_gps_yaw_ = 0.0;
    // offset
    Eigen::Vector3d gps_offset_p_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond gps_offset_q_ = Eigen::Quaterniond::Identity();
    double gps_offset_yaw_ = 0.0;

    // imu propagate
    Eigen::Vector3d latest_gps_acc_0 = Eigen::Vector3d::Zero();
    Eigen::Vector3d latest_gps_gyr_0 = Eigen::Vector3d::Zero();
    Eigen::Vector3d g_ = Eigen::Vector3d(0, 0, 9.81); // Gravity constant

    // 积分
    Eigen::Vector3d integrated_v_imu_ = Eigen::Vector3d::Zero();

    bool gps_healthy_ = false;
    bool receiving_gps_ = false;

    rclcpp::Time last_get_gps_time_;
    rclcpp::Time last_gps_time_;
    rclcpp::Time last_propagate_time_;



    void gpsCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();
    bool gpsOdomIsValid(const nav_msgs::msg::Odometry& o);
};