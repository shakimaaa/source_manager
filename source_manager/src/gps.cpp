#include "source_manager/gps.hpp"
#include <functional>

GPS::GPS(rclcpp::Node::SharedPtr node) 
    : SourceBase(std::move(node)) {


    // Declare GPS related parameters
    node_->declare_parameter("gps.maxSpeeddiff", 4.0);
    node_->declare_parameter("gps.maxAnglediff", 100.0);
    node_->declare_parameter("gps.canrestart", false);

    node_->get_parameter("gps.maxSpeeddiff", maxSpeeddiff_);
    node_->get_parameter("gps.maxAnglediff", maxAnglediff_);
    node_->get_parameter("gps.canrestart", can_restart_);

    // Create a mutually exclusive callback group
    gps_callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    
    rclcpp::SubscriptionOptions gps_sub_opt;
    gps_sub_opt.callback_group = gps_callback_group_;

    // Subscribe to GPS odometer data
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
    // Set the offset of the GPS coordinate system relative to the output coordinate system
    gps_offset_p_ = p;
    gps_offset_q_ = q;
    gps_offset_yaw_ = yaw;

    propageted_data_.q = (gps_offset_q_ * propageted_data_.q).normalized();
    propageted_data_.p = (gps_offset_q_ * propageted_data_.p) + gps_offset_p_;
    propageted_data_.v = gps_offset_q_ * propageted_data_.v;
    propageted_data_.yaw = gps_offset_yaw_ + propageted_data_.yaw;
    
    RCLCPP_INFO(node_->get_logger(), "[GPS source] set offset");
}

void GPS::setOdometry() {
    // Update internal odometer status
    odom_data_.q = (gps_offset_q_ * latest_gps_q_).normalized();
    odom_data_.p = (gps_offset_q_ * latest_gps_p_) + gps_offset_p_;
    odom_data_.v = ps_offset_q_ * latest_gps_v_;
    odom_data_.a = latest_gps_a_;
    odom_data_.yaw = gps_offset_yaw_ + latest_gps_yaw_;
    
}
void GPS::setGpsdata() {

    // Apply offset to transform GPS data
    latest_gps_q_ = r_gps_q_;
    latest_gps_p_ = r_gps_p_;
    // latest_gps_p_ =  r_gps_p_ + gps_offset_p_;
    latest_gps_v_ = r_gps_v_ ;
    latest_gps_a_ = r_gps_a_ ;
    latest_gps_yaw_ = r_gps_yaw_;

    // latest_gps_q_ = (gps_offset_q_ * r_gps_q_).normalized();
    // latest_gps_p_ = (gps_offset_q_ * r_gps_p_) + gps_offset_p_;
    // // latest_gps_p_ =  r_gps_p_ + gps_offset_p_;
    // latest_gps_v_ = gps_offset_q_ * r_gps_v_ ;
    // latest_gps_a_ = r_gps_a_ ;
    // latest_gps_yaw_ = r_gps_yaw_ + gps_offset_yaw_;
    setOdometry();
}

void GPS::setCurrPose(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, const Eigen::Quaterniond& q) {
    // Update propagation status (for high frequency output)
    propageted_data_.q = (gps_offset_q_ * q).normalized();;
    propageted_data_.p = (gps_offset_q_ * pos) + gps_offset_p_;;
    propageted_data_.v = gps_offset_q_ * vel;
    propageted_data_.yaw = latest_gps_yaw_ + gps_offset_yaw_;
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

    // Update last received time
    last_get_gps_time_ = node_->get_clock()->now();
    receiving_gps_ = true;
    // Extract position, speed, attitude
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
    
    // Calculate Yaw angle
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

    // Apply offset
    setGpsdata();

    // If the initial check is not completed, perform the check
    if (initial_check_state_ != InitialCheckState::COMPLETE) {
        performInitialStillnessCheck(r_gps_v_);
        return; // During the inspection period, no subsequent health checks such as speed comparison will be performed.
    } else {
        // Using EKF for health checks
        //1. Construct observation vector z
        Matrix<double, M_OBS_POS + M_OBS_VEL + M_OBS_ORI, 1> z;
        z.block<3, 1>(0, 0) = r_gps_p_;
        z.block<3, 1>(3, 0) = r_gps_v_;
        z.block<4, 1>(6, 0) << r_gps_q_.x(), r_gps_q_.y(), r_gps_q_.z(), r_gps_q_.w();

        // 2. Call EKF update and set the health status based on the return value
        bool is_healthy = ekf_->update(z);
        setHealthy(is_healthy);
        if (!is_healthy) {
            RCLCPP_WARN(node_->get_logger(), "[GPS source] EKF innovation check failed. Marked as unhealthy.");
        }
    }

    // Update odometer timestamp and status for possible subsequent comparison checks
    rclcpp::Time this_stamp = node_->get_clock()->now();
    if (curr_odom_stamp_.nanoseconds() != 0) {
        last_odom_stamp_ = curr_odom_stamp_;
        last_odom_state_ = curr_odom_state_;
    } else {
        // first time，last=curr
        // integrated_v_imu_ = latest_gps_v_;
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
    //This function can now be cleared or removed

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
                     const Eigen::Quaterniond& orientation,
                     const rclcpp::Time imu_time_stamp) {

    // imu_acc_ = linearAcceleration;
    // imu_gyro_ = angularVelocity;
    // imu_orientation_ = orientation;

    const double t = node_->now().seconds();
    std::lock_guard<std::mutex> lk(imu_mtx_);

    // 使用 IMU 数据执行 EKF 预测步骤
    if (last_propagate_time_.seconds() != 0) {
        double dt = (imu_time_stamp - last_propagate_time_).seconds();
        if (dt > 0 && dt < 1.0) {
            ekf_->predict(linearAcceleration, angularVelocity, dt);
        }
    }
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

    // Kinematic integral prediction (for high frequency output)
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
    // Check if data reception times out
    rclcpp::Time now_ = node_->now();
    double dt = (now_ - last_get_gps_time_).seconds();
    // 
    if (dt > 1.0) {
        RCLCPP_WARN(node_->get_logger(), "[GPS source] time diff = %f", dt);
        // RCLCPP_WARN(node_->get_logger(), "GPS data empty.");
        return false;
    }

    return gps_healthy_;
}