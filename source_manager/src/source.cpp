#include "source_manager/source.hpp"

SourceBase::SourceBase(std::shared_ptr<rclcpp::Node> node)
: node_(std::move(node))
{
    // if (!node_) {
    //     RCLCPP_WARN(rclcpp::get_logger("SourceBase"), "SourceBase constructed with null node");
    // }
}

void SourceBase::setImudata(const Eigen::Vector3d& linearAcceleration,
                            const Eigen::Vector3d& angularVelocity,
                            const Eigen::Quaterniond& orientation) {

    (void) linearAcceleration;
    (void) angularVelocity;
    (void) orientation;
}

void SourceBase::setHealthy(bool healthy) {
    (void) healthy;
}

void SourceBase::setCurrPose(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, const Eigen::Quaterniond& q) {
    (void) pos;
    (void) vel;
    (void) q;
}

bool SourceBase::isHealthy() {
    return true;
}

void SourceBase::setOffset(const Eigen::Vector3d& p, const Eigen::Quaterniond& q, double yaw) {
    (void)p;
    (void)q;
    (void)yaw;
}