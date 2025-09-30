#pragma once

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>


// enum  class State { UNINIT = 0, SLAM = 1, GPS = 2 };
struct BaseData
{
    Eigen::Vector3d curr_pos{Eigen::Vector3d::Zero()};
    Eigen::Vector3d curr_vel{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond curr_q{Eigen::Quaterniond::Identity()};

    Eigen::Vector3d prev_pos{Eigen::Vector3d::Zero()};
    Eigen::Vector3d prev_vel{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond prev_q{Eigen::Quaterniond::Identity()};

    Eigen::Vector3d gps_curr_pos{Eigen::Vector3d::Zero()};
    Eigen::Vector3d gps_curr_vel{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond gps_curr_q{Eigen::Quaterniond::Identity()};

    Eigen::Vector3d slam_curr_pos{Eigen::Vector3d::Zero()};
    Eigen::Vector3d slam_curr_vel{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond slam_curr_q{Eigen::Quaterniond::Identity()};
    
    bool gps_healthy = false;
    bool slam_healthy = false;

};

class SourceBase
{
public:
    SourceBase(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<BaseData> base_data);

    enum class State { UNINIT = 0, SLAM = 1, GPS = 2 };

    virtual void setImudata(const Eigen::Vector3d& linearAcceleration,
                            const Eigen::Vector3d& angularVelocity,
                            const Eigen::Quaterniond& orientation);
    virtual void setHealthy(bool healthy); 
    virtual void setCurrPose(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, const Eigen::Quaterniond& q);
    virtual bool isHealthy();
    

    static Eigen::Quaterniond deltaQ(const Eigen::Vector3d& theta);

    
protected:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<BaseData> base_data_;

};
