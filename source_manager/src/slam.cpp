#include "source_manager/slam.hpp"

SLAM::SLAM(rclcpp::Node::SharedPtr node, std::shared_ptr<BaseData> data)
    : SourceBase(std::move(node), std::move(data)) {

    slam_callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    
    rclcpp::SubscriptionOptions slam_sub_opt;
    slam_sub_opt.callback_group = slam_callback_group_;

    slam_sub_ = node_->create_subscription<xion_msg::msg::ExtendedOdometry>(
        "/odometry_source",10,
        std::bind(&SLAM::slamCallback, this, std::placeholders::_1),
        slam_sub_opt);
    
    slam_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&SLAM::timerCallback, this));

    RCLCPP_INFO(node_->get_logger(), "SLAM source started.");
}

void SLAM::setHealthy(bool healthy) {
    base_data_->slam_healthy = healthy;
}

void SLAM::setSlamdata() {
    latest_slam_p_ = r_slam_p_ + slam_offset_p_;
    latest_slam_q_ = r_slam_q_ * slam_offset_q_;
    latest_slam_v_ = r_slam_v_;
    latest_slam_a_ = r_slam_a_;
    latest_slam_yaw_ = r_slam_yaw_ + slam_offset_yaw_;
}

void SLAM::setCurrPose(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, const Eigen::Quaterniond& q) {
    base_data_->slam_curr_pos = pos;
    base_data_->slam_curr_vel = vel;
    base_data_->slam_curr_q = q;
}

void SLAM::slamCallback(const xion_msg::msg::ExtendedOdometry::SharedPtr msg) 
{
    RCLCPP_INFO(node_->get_logger(), "SLAM data received");
    if (!msg){
        RCLCPP_WARN(node_->get_logger(), "SLAM data is null");
        setHealthy(false);
        return;
    }

    last_get_slam_time_ = node_->now();
    receiving_slam_ = true;

    r_slam_p_ << msg->pose.position.x,
                msg->pose.position.y,
                msg->pose.position.z;
    r_slam_q_ = Eigen::Quaterniond(msg->pose.orientation.w,
                msg->pose.orientation.x,
                msg->pose.orientation.y,
                msg->pose.orientation.z).normalized();
    r_slam_v_ << msg->twist.linear.x,
                msg->twist.linear.y,
                msg->twist.linear.z;
    r_slam_a_ << msg->twist.angular.x,
                msg->twist.angular.y,
                msg->twist.angular.z;
    r_slam_yaw_ = std::atan2(2.0*(r_slam_q_.w()*r_slam_q_.z() + r_slam_q_.x()*r_slam_q_.y()),
                        1.0 - 2.0*(r_slam_q_.y()*r_slam_q_.y() + r_slam_q_.z()*r_slam_q_.z()));
    latest_Ba_ << msg->bas.x,
                msg->bas.y,
                msg->bas.z;
    latest_Bg_ << msg->bgs.x,
                msg->bgs.y,
                msg->bgs.z;
    
    RCLCPP_INFO(node_->get_logger(),
        "r_slam_p_: %.3f, %.3f, %.3f\n"
        "r_slam_q_: %.3f, %.3f, %.3f, %.3f\n"
        "r_slam_v_: %.3f, %.3f, %.3f\n"
        "r_slam_a_: %.3f, %.3f, %.3f\n"
        "r_slam_yaw_: %.3f\n"
        "latest_Ba_: %.3f, %.3f, %.3f\n"
        "latest_Bg_: %.3f, %.3f, %.3f\n",
        r_slam_p_(0), r_slam_p_(1), r_slam_p_(2),
        r_slam_q_.w(), r_slam_q_.x(), r_slam_q_.y(), r_slam_q_.z(), 
        r_slam_v_(0), r_slam_v_(1), r_slam_v_(2),
        r_slam_a_(0), r_slam_a_(1), r_slam_a_(2),
        r_slam_yaw_,
        latest_Ba_(0), latest_Ba_(1), latest_Ba_(2),
        latest_Bg_(0), latest_Bg_(1), latest_Bg_(2));
    
    setSlamdata();
    receiving_slam_ = false;
}

void SLAM::timerCallback() {
    if (!receiving_slam_) {
        rclcpp::Time now_ = node_->now();
        if (last_slam_time_.seconds() == 0){
            last_slam_time_ = now_;
            return;
        }

        double dt = (now_ - last_slam_time_).seconds();
        last_slam_time_ = now_;

        imu_acc_.z() -= 9.81;

        integrated_v_imu_ += imu_acc_ * dt;
        // Calculate the difference in magnitude
        double diff = (latest_slam_v_ - integrated_v_imu_).norm();
        double angle = 0.0;
        if (latest_slam_v_.norm() > 1e-3 && integrated_v_imu_.norm() > 1e-3) {
            double cos_angle = latest_slam_v_.normalized().dot(integrated_v_imu_.normalized());
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

    if (last_propagate_time_.seconds() == 0){
        last_propagate_time_ = node_->now();
        latest_slam_acc_0 = imu_acc_;
        latest_slam_gyr_0 = imu_gyro_;
        return;
    }

    double dt_slam = (node_->now() - last_propagate_time_).seconds();
    last_propagate_time_ = node_->now();

    if (dt_slam <= 0.0 || dt_slam >1.0) 
    {
        RCLCPP_WARN(node_->get_logger(), "Invalid slam dt = %f", dt_slam);
        return;
    }
    Eigen::Vector3d un_slam_acc_0 = latest_slam_q_ *(latest_slam_acc_0 - latest_Ba_) - g_;
    Eigen::Vector3d un_slam_gyr = 0.5 * (latest_slam_gyr_0 + imu_gyro_) - latest_Bg_;
    latest_slam_q_ = latest_slam_q_ * deltaQ(un_slam_gyr * dt_slam);
    Eigen::Vector3d un_slam_acc_1 = latest_slam_q_ *(imu_acc_ - latest_Ba_) - g_;
    Eigen::Vector3d un_slam_acc = 0.5 * (un_slam_acc_0 + un_slam_acc_1);
    latest_slam_p_ = latest_slam_p_ + latest_slam_v_ * dt_slam + 0.5 * un_slam_acc * dt_slam * dt_slam;
    latest_slam_v_ = latest_slam_v_ + un_slam_acc * dt_slam;
    latest_slam_acc_0 = imu_acc_;
    latest_slam_gyr_0 = imu_gyro_;

    setCurrPose(latest_slam_p_, latest_slam_v_, latest_slam_q_); // update current pose
}

void SLAM::setImudata(const Eigen::Vector3d& linearAcceleration,
                      const Eigen::Vector3d& angularVelocity,
                      const Eigen::Quaterniond& orientation) {
    imu_acc_ = linearAcceleration;
    imu_gyro_ = angularVelocity;
    imu_orientation_ = orientation;
}

bool SLAM::isHealthy() {
    rclcpp::Time now_ = node_->now();
    double dt = (now_ - last_get_slam_time_).seconds();
    RCLCPP_INFO(node_->get_logger(), "time diff = %f", dt);
    if (dt > 1.0) {
        RCLCPP_WARN(node_->get_logger(), "SLAM data empty.");
        return false;
    }

    return base_data_->slam_healthy;
}
