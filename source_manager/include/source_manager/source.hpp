#pragma once

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

struct BaseData
{
    Eigen::Vector3d curr_pos{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond curr_q{Eigen::Quaterniond::Identity()};

    bool gps_healthy = false;
    bool slam_healthy = false;

};

class SourceBase
{
public:
    enmu class State { UNINIT = 0, SLAM = 1, GPS = 2 };

    virtual void setImudata(Eigen::Vector3d &linearAccleration,
                            Eigen::Vector3d &angularVelocity,
                            Eigen::Quaterniond &orientation);
    
    virtual void setHealthy(bool healthy); 
protected:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<BaseData> base_data_;
};
