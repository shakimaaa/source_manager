#include "source_manager/source.hpp"

void SourceBase::setImudata(Eigen::Vector3d &linearAccleration,
                            Eigen::Vector3d &angularVelocity,
                            Eigen::Quaterniond &orientation) {

    (void) linearAccleration;
    (void) angularVelocity;
    (void) orientation;
}

void SourceBase::setHealthy(bool healthy) {
    (void) healthy;
}