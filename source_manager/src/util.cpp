#include "source_manager/util.hpp"
#include <Eigen/Geometry>
#include <cmath>

Eigen::Quaterniond deltaQ(const Eigen::Vector3d& theta) {
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

double normalizeAngle(double angle) {
    while (angle > M_PI)  angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

Eigen::Matrix3d Rz(double yaw) {
    return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}