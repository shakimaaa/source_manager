#include "source_manager/gps.hpp"
#include <functional>

GPS::GPS(rclcpp::Node::SharedPtr node) 
    : SourceBase(std::move(node)) {


    node_->declare_parameter("gps.maxSpeeddiff", 4.0);
    node_->declare_parameter("gps.maxAnglediff", 100.0);
    node_->declare_parameter("gps.canrestart", false);

    node_->get_parameter("gps.maxSpeeddiff", maxSpeeddiff_);
    node_->get_parameter("gps.maxAnglediff", maxAnglediff_);
    node_->get_parameter("gps.canrestart", can_restart_);

    gps_callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    
    rclcpp::SubscriptionOptions gps_sub_opt;
    gps_sub_opt.callback_group = gps_callback_group_;

    gps_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/position/gps_odom", rclcpp::QoS(400).best_effort(),
        std::bind(&GPS::gpsCallback, this, std::placeholders::_1),
        gps_sub_opt);
    
    gps_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&GPS::timerCallback, this));

    auto clock = node_->get_clock();
    auto clock_type = clock->get_clock_type();
    last_get_gps_time_ = rclcpp::Time(0, 0, clock_type);
    last_gps_time_ = rclcpp::Time(0, 0, clock_type);
    last_propagate_time_ = rclcpp::Time(0, 0, clock_type);

    RCLCPP_INFO(node_->get_logger(), "[GPS source]GPS source started.");
}

bool GPS::canRestart() {
    return can_restart_;
}

void GPS::restartSource() {
    bool expected = false;
    if (!restarting_.compare_exchange_strong(expected, true)) {
        return; 
    }
    struct RestartGuard {
        std::atomic_bool& flag;
        explicit RestartGuard(std::atomic_bool& f) : flag(f) {}
        ~RestartGuard() { flag.store(false); }
    } guard{restarting_};

    return;
}

void GPS::setOffset(const Eigen::Vector3d& p, const Eigen::Quaterniond& q, double yaw) {
    gps_offset_p_ = p;
    gps_offset_q_ = q;
    gps_offset_yaw_ = yaw;
    latest_gps_q_ = (gps_offset_q_ * r_gps_q_).normalized();
    latest_gps_p_ = (latest_gps_q_ * r_gps_p_) + gps_offset_p_;
    latest_gps_yaw_ = r_gps_yaw_ + gps_offset_yaw_;
    RCLCPP_INFO(node_->get_logger(), "[gps source] set offset");
}

void GPS::setOdometry() {
    odom_data_.p = latest_gps_p_;
    odom_data_.q = latest_gps_q_;
    odom_data_.v = latest_gps_v_;
    odom_data_.a = latest_gps_a_;
    odom_data_.yaw = latest_gps_yaw_;
    
}
void GPS::setGpsdata() {

    
    latest_gps_q_ = (gps_offset_q_ * r_gps_q_).normalized();
    latest_gps_p_ = (latest_gps_q_ * r_gps_p_) + gps_offset_p_;
    latest_gps_v_ = r_gps_v_ ;
    latest_gps_a_ = r_gps_a_ ;
    latest_gps_yaw_ = r_gps_yaw_ + gps_offset_yaw_;
    setOdometry();
}

void GPS::setCurrPose(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, const Eigen::Quaterniond& q) {
    propageted_data_.p = pos;
    propageted_data_.v = vel;
    propageted_data_.q = q;
    propageted_data_.yaw = latest_gps_yaw_;
}

NavState GPS::getPropagateOdometry() const {
    return propageted_data_;
}

NavState GPS::getOdometry() const {
    return odom_data_;
}


void GPS::gpsCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // RCLCPP_INFO(node_->get_logger(), "[GPS source] data received");

     
    // if (!gpsOdomIsValid(*msg)) {
    //     RCLCPP_WARN(node_->get_logger(), "fix_type <3, Invalid GPS data");
    //     setHealthy(false);
    //     return;
    // }

    last_get_gps_time_ = node_->get_clock()->now();
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
    
    r_gps_yaw_ = std::atan2(2.0*(r_gps_q_.w()*r_gps_q_.z() + r_gps_q_.x()*r_gps_q_.y()),
                        1.0 - 2.0*(r_gps_q_.y()*r_gps_q_.y() + r_gps_q_.z()*r_gps_q_.z()));
    
    // RCLCPP_INFO(node_->get_logger(), "[GPS source] received yaw: %f", r_gps_yaw_);
    // RCLCPP_INFO(node_->get_logger(), "[GPS source] received pos: [%f, %f, %f], vel: [%f, %f, %f], q: [%f, %f, %f, %f]", 
    //     r_gps_p_(0), r_gps_p_(1), r_gps_p_(2),
    //     r_gps_v_(0), r_gps_v_(1), r_gps_v_(2),
    //     r_gps_q_.w(), r_gps_q_.x(), r_gps_q_.y(), r_gps_q_.z());
    // RCLCPP_INFO(node_->get_logger(),
    //     "r_gps_p_: %.3f, %.3f, %.3f\n"
    //     "r_gps_v_: %.3f, %.3f, %.3f\n"
    //     "r_gps_a_: %.3f, %.3f, %.3f\n"
    //     "r_gps_q_: %.3f, %.3f, %.3f, %.3f",
    //     r_gps_p_(0), r_gps_p_(1), r_gps_p_(2),
    //     r_gps_v_(0), r_gps_v_(1), r_gps_v_(2),
    //     r_gps_a_(0), r_gps_a_(1), r_gps_a_(2),
    //     r_gps_q_.w(), r_gps_q_.x(), r_gps_q_.y(), r_gps_q_.z());

    setGpsdata();

    rclcpp::Time this_stamp = node_->get_clock()->now();
    if (curr_odom_stamp_.nanoseconds() != 0) {
        last_odom_stamp_ = curr_odom_stamp_;
        last_odom_state_ = curr_odom_state_;
    } else {
        // first time，last=curr
        last_odom_stamp_ = this_stamp;
        last_odom_state_.p = latest_gps_p_;
        last_odom_state_.v = latest_gps_v_;
        last_odom_state_.q = latest_gps_q_;
        last_odom_state_.yaw = latest_gps_yaw_;
    }
    curr_odom_stamp_ = this_stamp;
    curr_odom_state_.p = latest_gps_p_;
    curr_odom_state_.v = latest_gps_v_;
    curr_odom_state_.q = latest_gps_q_;
    curr_odom_state_.yaw = latest_gps_yaw_;
    if (last_odom_stamp_.nanoseconds() != 0) {
        odom_pending_compare_.store(true);
    }

    receiving_gps_ = false;

}

void GPS::timerCallback() {
    if (!odom_pending_compare_.load()) return;

    const double t0 = last_odom_stamp_.seconds();
    const double t1 = curr_odom_stamp_.seconds();

    if (t1 <= t0) {
        RCLCPP_WARN(node_->get_logger(),"t1 <= t0");
        odom_pending_compare_.store(false);
        return;
    }

    std::vector<ImuLite> seg;
    if (!extractImuInterval_(t0, t1, seg)) {
        // Insufficient IMU samples, skip this time and wait for next time
        RCLCPP_WARN(node_->get_logger(),"do not extractImuInterval_");
        odom_pending_compare_.store(false);
        return;
    }
    Eigen::Vector3d dvel_imu;
    integrateIntervalMidpoint_(seg, dvel_imu);
    integrated_v_imu_ = last_odom_state_.v + dvel_imu;
    const Eigen::Vector3d v_slam = curr_odom_state_.v;

    double diff = (v_slam - integrated_v_imu_).norm();

    double angle = 0.0;
    if (v_slam.norm() > 1e-3 && integrated_v_imu_.norm() > 1e-3) {
        double c = v_slam.normalized().dot(integrated_v_imu_.normalized());
        c = std::clamp(c, -1.0, 1.0);
        angle = std::acos(c) * 180.0 / M_PI;
    }
    RCLCPP_WARN(node_->get_logger(), "speed difference = %f, angle = %f", diff, angle);
    if (diff > maxSpeeddiff_ && angle > maxAnglediff_) {
        setHealthy(false);
        RCLCPP_WARN(node_->get_logger(), "speed difference = %f, angle = %f", diff, angle);
    } else {
        //std::cout << "speed difference = " << diff << ", angle = " << angle << std::endl;
        setHealthy(true);
    }
    // if (!receiving_gps_) {
    //     rclcpp::Time now_ = node_->now();
    //     if (last_gps_time_.seconds() == 0) {
    //         last_gps_time_ = now_;
    //         return;
    //     }

    //     double dt = (now_ - last_gps_time_).seconds();
    //     last_gps_time_ = now_;

    //     imu_acc_.z() -= 9.81;

    //     integrated_v_imu_ += imu_acc_ * dt;
    //     // Calculate the difference in magnitude
    //     double diff = (latest_gps_v_ - integrated_v_imu_).norm();
    //     double angle = 0.0;
    //     if (latest_gps_v_.norm() > 1e-3 && integrated_v_imu_.norm() > 1e-3) {
    //         double cos_angle = latest_gps_v_.normalized().dot(integrated_v_imu_.normalized());
    //         cos_angle = std::clamp(cos_angle, -1.0, 1.0);
    //         angle = std::acos(cos_angle) * 180.0 / M_PI;
    //     }

    //     // if (diff > maxSpeeddiff_ && angle > maxAnglediff_)
    //     // {
    //     //     setHealthy(false);
    //     //     RCLCPP_WARN(node_->get_logger(), "speed difference = %f, angle = %f", diff, angle);
    //     // }else {
    //         //std::cout << "speed difference = " << diff << ", angle = " << angle << std::endl;
    //         setHealthy(true);
    //     // }
    // }

    // if (last_propagate_time_.seconds() == 0) {
    //     last_propagate_time_ = node_->now();
    //     latest_gps_acc_0 = imu_acc_;
    //     latest_gps_gyr_0 = imu_gyro_;
    //     return;
    // }

    // double dt_gps = (node_->now() - last_propagate_time_).seconds();
    // last_propagate_time_ = node_->now();
    // if (dt_gps <= 0 || dt_gps > 1.0) {
    //     RCLCPP_WARN(node_->get_logger(), "q gps dt = %f", dt_gps);
    //     return;
    // }
    // Eigen::Vector3d un_gps_acc_0 = latest_gps_q_ * latest_gps_acc_0 - g_;
    // Eigen::Vector3d un_gps_gyr = 0.5 * (latest_gps_gyr_0 + imu_gyro_);
    // latest_gps_q_ = latest_gps_q_ * deltaQ(un_gps_gyr * dt_gps);
    // Eigen::Vector3d un_gps_acc_1 = latest_gps_q_ * imu_acc_ - g_;
    // Eigen::Vector3d un_gps_acc = 0.5 * (un_gps_acc_0 + un_gps_acc_1);
    // latest_gps_p_ = latest_gps_p_ + latest_gps_v_ * dt_gps + 0.5 * un_gps_acc * dt_gps * dt_gps;
    // latest_gps_v_ = latest_gps_v_ + un_gps_acc * dt_gps;
    // latest_gps_a_ = un_gps_acc;
    // latest_gps_gyr_0 = imu_gyro_;
    // latest_gps_acc_0 = imu_acc_;
    // latest_gps_yaw_ = std::atan2(2.0*(latest_gps_q_.w()*latest_gps_q_.z() + latest_gps_q_.x()*latest_gps_q_.y()),
    //                     1.0 - 2.0*(latest_gps_q_.y()*latest_gps_q_.y() + latest_gps_q_.z()*latest_gps_q_.z()));

    // setCurrPose(latest_gps_p_, latest_gps_v_, latest_gps_q_);
}
     


void GPS::setImudata(const Eigen::Vector3d& linearAcceleration,
                     const Eigen::Vector3d& angularVelocity,
                     const Eigen::Quaterniond& orientation) {

    // imu_acc_ = linearAcceleration;
    // imu_gyro_ = angularVelocity;
    // imu_orientation_ = orientation;

    const double t = node_->now().seconds();
    std::lock_guard<std::mutex> lk(imu_mtx_);
    if (!imu_buf_.empty() && t < imu_buf_.back().t) {
        return; 
    }
    imu_buf_.push_back(ImuLite{t, linearAcceleration, orientation});
    if (imu_buf_.size() > imu_buf_max_) imu_buf_.pop_front();

    if (last_propagate_time_.seconds() == 0) {
        last_propagate_time_ = node_->now();
        latest_gps_acc_0 = linearAcceleration;
        latest_gps_gyr_0 = angularVelocity;
        return;
    }

    double dt_gps = (node_->now() - last_propagate_time_).seconds();
    last_propagate_time_ = node_->now();
    if (dt_gps <= 0 || dt_gps > 1.0) {
        RCLCPP_WARN(node_->get_logger(), "[GPS source] q gps dt = %f", dt_gps);
        return;
    }

    Eigen::Vector3d un_gps_acc_0 = latest_gps_q_ * latest_gps_acc_0 - g_;
    Eigen::Vector3d un_gps_gyr = 0.5 * (latest_gps_gyr_0 + angularVelocity);
    latest_gps_q_ = latest_gps_q_ * deltaQ(un_gps_gyr * dt_gps);
    Eigen::Vector3d un_gps_acc_1 = latest_gps_q_ * linearAcceleration - g_;
    Eigen::Vector3d un_gps_acc = 0.5 * (un_gps_acc_0 + un_gps_acc_1);
    latest_gps_p_ = latest_gps_p_ + latest_gps_v_ * dt_gps + 0.5 * un_gps_acc * dt_gps * dt_gps;
    latest_gps_v_ = latest_gps_v_ + un_gps_acc * dt_gps;
    latest_gps_a_ = un_gps_acc;
    latest_gps_gyr_0 = angularVelocity;
    latest_gps_acc_0 = linearAcceleration;
    latest_gps_yaw_ = std::atan2(2.0*(latest_gps_q_.w()*latest_gps_q_.z() + latest_gps_q_.x()*latest_gps_q_.y()),
                        1.0 - 2.0*(latest_gps_q_.y()*latest_gps_q_.y() + latest_gps_q_.z()*latest_gps_q_.z()));

    setCurrPose(latest_gps_p_, latest_gps_v_, latest_gps_q_);
}

void GPS::setHealthy(bool healthy) {
    gps_healthy_ = healthy;
}

bool GPS::gpsOdomIsValid(const nav_msgs::msg::Odometry& o) {
   const auto& p = o.pose.covariance;
   return (p[0] < 1e5 && p[7] < 1e5 && p[14] < 1e5);
}

bool GPS::isHealthy() {
    rclcpp::Time now_ = node_->now();
    double dt = (now_ - last_get_gps_time_).seconds();
    // RCLCPP_INFO(node_->get_logger(), "time diff = %f", dt);
    if (dt > 1.0) {
        // RCLCPP_WARN(node_->get_logger(), "GPS data empty.");
        return false;
    }

    return gps_healthy_;
}