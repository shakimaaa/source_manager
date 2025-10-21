#pragma once

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include "source_manager/util.hpp"


// enum  class State { UNINIT = 0, SLAM = 1, GPS = 2 };

class SourceBase
{
public:
    SourceBase(std::shared_ptr<rclcpp::Node> node);

    enum class State { UNINIT = 0, SLAM = 1, GPS = 2 };

    virtual void setImudata(const Eigen::Vector3d& linearAcceleration,
                            const Eigen::Vector3d& angularVelocity,
                            const Eigen::Quaterniond& orientation);
    virtual void setHealthy(bool healthy); 
    virtual void setCurrPose(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, const Eigen::Quaterniond& q);
    virtual bool isHealthy();
    virtual void setOffset(const Eigen::Vector3d& p, const Eigen::Quaterniond& q, double yaw);
    virtual NavState getPropagateOdometry() const = 0;
    virtual NavState getOdometry() const = 0;
    virtual bool canRestart() = 0;
    virtual void restartSource() = 0;


    
protected:
    std::shared_ptr<rclcpp::Node> node_;

};
