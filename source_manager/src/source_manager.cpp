#include "source_manager/source_manager.hpp"

SourceManager::SourceManager() : Node("SourceManager")
{
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/mav/imu/data_raw", rclcpp::QoS(100).best_effort(),
        std::bind(&SourceManager::imuCallback, this, std::placeholders::_1));

    propagate_odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/imu_propagate_", rclcpp::QoS(10).best_effort().durability_volatile());

    odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/odometry_",1000);
    
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

void SourceManager::publishPropagateOdometry() {

    nav_msgs::msg::Odometry propagated_odometry;
    propagated_odometry.header.stamp = this->now();
    propagated_odometry.header.frame_id = "world";

    if (active_source_ == SourceBase::State::GPS) {
        propagated_odometry.pose.pose.position.x = shared_data_->gps_curr_pos.x();
        propagated_odometry.pose.pose.position.y = shared_data_->gps_curr_pos.y();
        propagated_odometry.pose.pose.position.z = shared_data_->gps_curr_pos.z();
        propagated_odometry.pose.pose.orientation.w = shared_data_->gps_curr_q.w();
        propagated_odometry.pose.pose.orientation.x = shared_data_->gps_curr_q.x();
        propagated_odometry.pose.pose.orientation.y = shared_data_->gps_curr_q.y();
        propagated_odometry.pose.pose.orientation.z = shared_data_->gps_curr_q.z();
        propagated_odometry.twist.twist.linear.x = shared_data_->gps_curr_vel.x();
        propagated_odometry.twist.twist.linear.y = shared_data_->gps_curr_vel.y();
        propagated_odometry.twist.twist.linear.z = shared_data_->gps_curr_vel.z();
        // propagated_odometry.twist.twist.angular.x = shared_data_->gps_curr_angvel.x();
        // propagated_odometry.twist.twist.angular.y = shared_data_->gps_curr_angvel.y();
        // propagated_odometry.twist.twist.angular.z = shared_data_->gps_curr_angvel.z();
        // propagate_odometry_pub_->publish(propagated_odometry);
    } else if (active_source_ == SourceBase::State::SLAM) {
        propagated_odometry.pose.pose.position.x = shared_data_->slam_curr_pos.x();
        propagated_odometry.pose.pose.position.y = shared_data_->slam_curr_pos.y();
        propagated_odometry.pose.pose.position.z = shared_data_->slam_curr_pos.z();
        propagated_odometry.pose.pose.orientation.w = shared_data_->slam_curr_q.w();
        propagated_odometry.pose.pose.orientation.x = shared_data_->slam_curr_q.x();
        propagated_odometry.pose.pose.orientation.y = shared_data_->slam_curr_q.y();
        propagated_odometry.pose.pose.orientation.z = shared_data_->slam_curr_q.z();
        propagated_odometry.twist.twist.linear.x = shared_data_->slam_curr_vel.x();
        propagated_odometry.twist.twist.linear.y = shared_data_->slam_curr_vel.y();
        propagated_odometry.twist.twist.linear.z = shared_data_->slam_curr_vel.z();
        // propagated_odometry.twist.twist.angular.x = shared_data_->slam_curr_angvel.x();
        // propagated_odometry.twist.twist.angular.y = shared_data_->slam_curr_angvel.y();
        // propagated_odometry.twist.twist.angular.z = shared_data_->slam_curr_angvel.z();
        // propagate_odometry_pub_->publish(propagated_odometry);
    }
    
    propagated_odometry_pub_->publish(propagated_odometry);
}

void SLAM::publishOdometry() {
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = this->now();
    odometry.header.frame_id = "world";
   if (active_source_ == SourceBase::State::GPS) {
        odometry.pose.pose.position.x = shared_data_->gps_odom_p.x();
        odometry.pose.pose.position.y = shared_data_->gps_odom_p.y();
        odometry.pose.pose.position.z = shared_data_->gps_odom_p.z();
        odometry.pose.pose.orientation.w = shared_data_->gps_odom_q.w();
        odometry.pose.pose.orientation.x = shared_data_->gps_odom_q.x();
        odometry.pose.pose.orientation.y = shared_data_->gps_odom_q.y();
        odometry.pose.pose.orientation.z = shared_data_->gps_odom_q.z();
        odometry.twist.twist.linear.x = shared_data_->gps_odom_v.x();
        odometry.twist.twist.linear.y = shared_data_->gps_odom_v.y();
        odometry.twist.twist.linear.z = shared_data_->gps_odom_v.z();
        // odometry.twist.twist.angular.x = shared_data_->gps_odom_a.x();
        // odometry.twist.twist.angular.y = shared_data_->gps_odom_a.y();
        // odometry.twist.twist.angular.z = shared_data_->gps_odom_a.z();

    } else if (active_source_ == SourceBase::State::SLAM) {
       odometry.pose.pose.position.x = shared_data_->slam_odom_p.x();
       odometry.pose.pose.position.y = shared_data_->slam_odom_p.y();
       odometry.pose.pose.position.z = shared_data_->slam_odom_p.z();
       odometry.pose.pose.orientation.w = shared_data_->slam_odom_q.w();
       odometry.pose.pose.orientation.x = shared_data_->slam_odom_q.x();
       odometry.pose.pose.orientation.y = shared_data_->slam_odom_q.y();
       odometry.pose.pose.orientation.z = shared_data_->slam_odom_q.z();
       odometry.twist.twist.linear.x = shared_data_->slam_odom_v.x();
       odometry.twist.twist.linear.y = shared_data_->slam_odom_v.y();
       odometry.twist.twist.linear.z = shared_data_->slam_odom_v.z();
       // odometry.twist.twist.angular.x = shared_data_->slam_odom_a.x();
       // odometry.twist.twist.angular.y = shared_data_->slom_odom_a.y();
       // odometry.twist.twist.angular.z = shared_data_->slam_odom_a.z();
   }
    odometry_pub_->publish(odometry);
}


void SourceManager::timerCallback() {
    checkSourceHealth();
    publishOdometry();
    publishPropagateOdometry();
}

