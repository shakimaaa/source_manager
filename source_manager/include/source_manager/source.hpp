#pragma once

#include <rclcpp/rclcpp.hpp>
//#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <deque>
#include "source_manager/util.hpp"
#include "source_manager/ekf.hpp"


// enum  class State { UNINIT = 0, SLAM = 1, GPS = 2 };

class SourceBase
{
public:
    SourceBase(std::shared_ptr<rclcpp::Node> node);

    enum class State { UNINIT = 0, SLAM = 1, GPS = 2 };

    virtual void setImudata(const Eigen::Vector3d& linearAcceleration,
                            const Eigen::Vector3d& angularVelocity,
                            const Eigen::Quaterniond& orientation,
                            const rclcpp::Time imu_time_stamp);
    virtual void setHealthy(bool healthy); 
    virtual void setCurrPose(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, const Eigen::Quaterniond& q);
    virtual bool isHealthy();
    virtual void setOffset(const Eigen::Vector3d& p, const Eigen::Quaterniond& q, double yaw);
    virtual NavState getPropagateOdometry() const = 0;
    virtual NavState getOdometry() const = 0;
    virtual bool canRestart() = 0;
    virtual void restartSource() = 0;
    // virtual bool extractImuInterval_(double t0, double t1, std::vector<ImuLite>& out) = 0;
    // virtual ImuLite interpImu_(const ImuLite& a, const ImuLite& b, double t) = 0;
    // virtual void integrateIntervalMidpoint_(const std::vector<ImuLite>& seg, Eigen::Vector3d& dvel_imu) = 0;
    bool extractImuInterval_(double t0, double t1, std::vector<ImuLite>& out);
    ImuLite interpImu_(const ImuLite& a, const ImuLite& b, double t);
    void integrateIntervalMidpoint_(const std::vector<ImuLite>& seg, Eigen::Vector3d& dvel_imu);

    Eigen::Vector3d imu_acc_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d imu_gyro_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond imu_orientation_ = Eigen::Quaterniond::Identity();

    Eigen::Vector3d g_ = Eigen::Vector3d(0, 0, 9.81); // Gravity constant

    // ImuLite
    std::deque<ImuLite> imu_buf_;
    std::mutex imu_mtx_;
    size_t imu_buf_max_ = 2000;
    rclcpp::Time last_odom_stamp_;
    rclcpp::Time curr_odom_stamp_;
    NavState     last_odom_state_;
    NavState     curr_odom_state_;
    std::atomic_bool odom_pending_compare_{false};
    Eigen::Vector3d integrated_v_imu_ = Eigen::Vector3d::Zero();


    
protected:
    std::shared_ptr<rclcpp::Node> node_;
    // 每个源都有自己的EKF实例
    std::unique_ptr<SourceEKF> ekf_;

    // Add a new state machine to manage initial checks
     enum class InitialCheckState {
        PENDING,    // Waiting for first data
        CHECKING,   // Stilling check in progress
        COMPLETE    // Check completed
    };


    InitialCheckState initial_check_state_;
    rclcpp::Time stillness_check_start_time_;
    std::vector<Eigen::Vector3d> stillness_check_velocities_;
    double stillness_velocity_threshold_;
    double stillness_check_duration_;
    bool init_check_false_restart;

    // 新增的成员函数
    void performInitialStillnessCheck(const Eigen::Vector3d& current_velocity);

};
