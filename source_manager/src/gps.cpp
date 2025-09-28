#include "source_manager/gps.hpp"
#include <functional>

GPS::GPS(rclcpp::Node::SharedPtr node, std::shared_ptr<BaseData> data) 
    : SourceBase(std::move(node), std::move(data)) {

    gps_callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    
    rclcpp::SubscriptionOptions gps_sub_opt;
    gps_sub_opt.callback_group = gps_callback_group_;

    gps_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/odom2", rclcpp::QoS(100).best_effort(),
        std::bind(&GPS::gpsCallback, this, std::placeholders::_1),
        gps_sub_opt);
    
    gps_timer_ = node_->create_timer(
        std::chrono::milliseconds(10),
        std::bind(&GPS::timerCallback, this));
}

void GPS::gpsCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    RCLCPP_INFO(node_->get_logger(), "GPS data received");

    // if (!gpsOdomIsValid(*msg)) {
    //     RCLCPP_WARN(node_->get_logger(), "fix_type <3, Invalid GPS data");
    //     setHealthy(false);
    //     return;
    // }

    receiving_gps_ = true;
    r_gps_p_ << msg->pose.pose.position.x,
                msg->pose.pose.position.y,
                msg->pose.pose.position.z;
    r_gps_v_ << msg->twist.twist.linear.x,
                msg->twist.twist.linear.y,
                msg->twist.twist.linear.z;
    r_gps_a_ << msg->twist.twist.angular.x,
                msg->twist.twist.angular.y,
                msg->twist.twist.angular.z;
    r_gps_q_ = Eigen::Quaterniond(msg->pose.pose.orientation.w,
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z).normalized();
    RCLCPP_INFO(node_->get_logger(), "r_gps_p_ : %f, %f, %f\n r_gps_v_ : %f, %f, %f\n r_gps_a_ : %f, %f, %f\n r_gps_q_ : %f, %f, %f, %f",
                r_gps_p_(0), r_gps_p_(1), r_gps_p_(2),
                r_gps_v_(0), r_gps_v_(1), r_gps_v_(2),
                r_gps_a_(0), r_gps_a_(1), r_gps_a_(2),
                r_gps_q_(0), r_gps_q_(1), r_gps_q_(2), r_gps_q_(3));
    receiving_gps_ = false;

}

void GPS::timerCallback() {
    if (!receiving_gps_) {
        rclcpp::Time now_ = node_->now();
        if (last_gps_time_.seconds() == 0) {
            last_gps_time_ = now_;
            return;
        }

        double dt = (now_ - last_gps_time_).seconds();
        last_gps_time_ = now_;

        imu_acc.z() -= 9.81;

        integrated_v_imu_ += imu_acc_ * dt;
        // Calculate the difference in magnitude
        double diff = (r_gps_v_ - integrated_v_imu_).norm();
        double angle = 0.0;
        if (r_gps_v_.norm() > 1e-3 && integrated_v_imu_.norm() > 1e-3) {
            double cos_angle = r_gps_v_.normalized().dot(integrated_v_imu_.normalized());
            cos_angle = std::clamp(cos_angle, -1.0, 1.0);
            angle = std::acos(cos_angle) * 180.0 / M_PI;
        }

        if (diff > 3.0 && angle > 30.0)
        {
            setHealthy(false);
            RCLCPP_WARN(node_->get_logger(), "speed difference = %f, angle = %f", diff, angle);
        }else {
            setHealthy(true);
        }
    }

    if (last_propagate_time_.seconds() == 0) {
        last_propagate_time_ = node_->now();
        latest_gps_acc_0 = imu_acc_;
        latest_gps_gyr_0 = imu_gyro_;
        return;
    }

    double dt_gps = (node_->now() - last_propagate_time_).seconds();
    last_propagate_time_ = node_->now();
    if (dt_gps <= 0 || dt_gps > 1.0) {
        RCLCPP_WARN(node_->get_logger(), "Invalid gps dt = %f", dt_gps);
        return;
    }
}
     


void GPS::setImudata(const Eigen::Vector3d& linearAcceleration,
                     const Eigen::Vector3d& angularVelocity,
                     const Eigen::Quaterniond& orientation) {

    imu_acc_ = linearAcceleration;
    imu_gyro_ = angularVelocity;
    imu_orientation_ = orientation;
}

void GPS::setHealthy(bool healthy) {
    base_data_->gps_healthy = healthy;
}

bool GPS::gpsOdomIsValid(const nav_msgs::msg::Odometry& o) {
   const auto& p = o.pose.covariance;
   return (p[0] < 1e5 && p[7] < 1e5 && p[14] < 1e5);
}