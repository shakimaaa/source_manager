#include "source_manager/source_manager.hpp"

SourceManager::SourceManager() : Node("SourceManager")
{
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/mav/imu/data_raw", rclcpp::QoS(100).best_effort(),
        std::bind(&SourceManager::imuCallback, this, std::placeholders::_1));
    
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(5),
        std::bind(&SourceManager::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "SourceManager node statrted.");
}

void SourceManager::init() {
    auto self = shared_from_this();
    shared_data_ = std::make_shared<BaseData>();
    gps_source_ = std::make_unique<GPS>(self, shared_data_);
    slam_source_ = std::make_unique<SLAM>(self, shared_data_);
}

void SourceManager::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "IMU data received");

    Eigen::Vector3d linearAcceleration(
        msg->linear_acceleration.x,
        msg->linear_acceleration.y,
        msg->linear_acceleration.z);
    Eigen::Vector3d angularVelocity(
        msg->angular_velocity.x,
        msg->angular_velocity.y,
        msg->angular_velocity.z);
    Eigen::Quaterniond orientation(
        msg->orientation.w,
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z);
    
    if (gps_source_) gps_source_->setImudata(linearAcceleration, angularVelocity, orientation);
    if (slam_source_) slam_source_->setImudata(linearAcceleration, angularVelocity, orientation);
    
}

void SourceManager::checkSourceHealth() {
    if (gps_source_ ->isHealthy() && slam_source_->isHealthy()) {
        active_source_ = SourceBase::State::GPS;
    } else if(gps_source_ ->isHealthy()) {
        active_source_ = SourceBase::State::GPS;
    } else if(slam_source_->isHealthy()) {
        active_source_ = SourceBase::State::SLAM;
    } else {
        active_source_ = SourceBase::State::UNINIT;
    }

    RCLCPP_INFO(this->get_logger(), "healthy GPS: %s, healthy SLAM: %s",
                gps_source_->isHealthy() ? "true" : "false",
                slam_source_->isHealthy() ? "true" : "false");
}

void publishPropagateOdometry() {
        
}


void SourceManager::timerCallback() {
    
}

