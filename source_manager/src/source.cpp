#include "source_manager/source.hpp"

SourceBase::SourceBase(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<BaseData> base_data)
: node_(std::move(node)), base_data_(std::move(base_data))
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

Eigen::Quaterniond SourceBase::deltaQ(const Eigen::Vector3d& theta) {
    if (theta.squaredNorm() < Eigen::NumTraits<double>::epsilon()) {
        return Eigen::Quaterniond::Identity();
    }

    Eigen::Quaterniond dq;
    const Eigen::Vector3d half_theta = theta / 2.0;
    const double theta_norm = half_theta.norm();

    if (theta_norm > 1e-5) {
        dq.w() = std::cos(theta_norm);
        dq.vec() = std::sin(theta_norm) * half_theta.normalized();
    } else {
        dq.w() = 1.0;
        dq.vec() = half_theta;
    }
    return dq.normalized();
}

void SourceBase::setCurrPose(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, const Eigen::Quaterniond& q) {
    base_data_->curr_pos = pos;
    base_data_->curr_vel = vel;
    base_data_->curr_q = q;
}

bool SourceBase::isHealthy() {
    return true;
}

void SourceBase::updateData() {}