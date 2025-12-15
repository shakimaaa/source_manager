#include "source_manager/slam.hpp"

SLAM::SLAM(rclcpp::Node::SharedPtr node)
    : SourceBase(std::move(node)) {

    // Declaration parameters: maximum speed difference, maximum angle difference, restart time threshold, whether restart is allowed
    node_->declare_parameter("slam.maxSpeeddiff", 0.25);
    node_->declare_parameter("slam.maxAnglediff", 60.0);
    node_->declare_parameter("slam.restart_time_threshold", 3.0);
    node_->declare_parameter("slam.canrestart", true);

    // Get parameter value
    node_->get_parameter("slam.maxSpeeddiff", maxSpeeddiff_);
    node_->get_parameter("slam.maxAnglediff", maxAnglediff_);
    node_->get_parameter("slam.restart_time_threshold", restart_time_threshold_);
    node_->get_parameter("slam.canrestart", can_restart_);

    slam_callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    
    //Configure subscription options and use mutually exclusive callback groups
    rclcpp::SubscriptionOptions slam_sub_opt;
    slam_sub_opt.callback_group = slam_callback_group_;

    // Subscribe to SLAM odometer data
    slam_sub_ = node_->create_subscription<xion_msg::msg::ExtendedOdometry>(
        "/position/slam_odom",10,
        std::bind(&SLAM::slamCallback, this, std::placeholders::_1),
        slam_sub_opt);
    
    // Create a timer (currently mainly used to trigger callbacks, the specific logic is in timerCallback)
    slam_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(400),
        std::bind(&SLAM::timerCallback, this));
    
    // Create a lifecycle management client to control the restart of external SLAM nodes
    lifecycle_client_ = node_->create_client<lifecycle_msgs::srv::ChangeState>("/xvins_lifecycle_node/change_state");
    get_state_client_ = node_->create_client<lifecycle_msgs::srv::GetState>("/xvins_lifecycle_node/get_state");
    // srv_set_restart_req_ = node_->create_service<std_srvs::srv::SetBool>(
    //     "slam/set_restart_requested",
    //     std::bind(&SLAM::onSetRestartReq, this, std::placeholders::_1, std::placeholders::_2));

    auto clock = node_->get_clock();
    auto clock_type = clock->get_clock_type();
    last_get_slam_time_ = rclcpp::Time(0, 0, clock_type);
    last_slam_time_ = rclcpp::Time(0, 0, clock_type);
    last_propagate_time_ = rclcpp::Time(0, 0, clock_type);


    RCLCPP_INFO(node_->get_logger(), "[SLAM source] SLAM source started.");
}

bool SLAM::canRestart() {
    // If configured to be non-restartable, return false directly.
    if (!can_restart_) {
        return false;
    }
    // Do not allow restart if initial quiescence check is not completed
    if (initial_check_state_ != InitialCheckState::COMPLETE) return false;

    // If the restart request is caused by an initial check failure
    if (init_check_false_restart)
    {
        init_check_false_restart = false;
        return true;
    }

    // If the SLAM status is unhealthy and messages have been received
    if (SLAM_healthy_ == false && is_received_message_) {
        if (unhealthy_start_time_.nanoseconds() == 0) {
            unhealthy_start_time_ = node_->now();
        }
        rclcpp::Time un_health_time = node_->now();
        double duration = (un_health_time - unhealthy_start_time_).seconds();
        if (duration > restart_time_threshold_){
            // If the duration of the unhealthy state exceeds the threshold, request a restart
            RCLCPP_ERROR(node_->get_logger(), "[SLAM source] SLAM unhealthy for 2 seconds. Requesting restart...");
            unhealthy_start_time_ = rclcpp::Time(0, 0);
            return true;
            
        }
    } else {
        unhealthy_start_time_ = rclcpp::Time(0, 0);
    }
    return false;
}

void SLAM::restartSource() {
    // Use atomic variables to ensure that only one restart process is running at the same time
    bool expected = false;
    if (!restarting_.compare_exchange_strong(expected, true)) {
        return; 
    }
    struct RestartGuard {
        std::atomic_bool& flag;
        explicit RestartGuard(std::atomic_bool& f) : flag(f) {}
        ~RestartGuard() { flag.store(false); }
    } guard{restarting_};

    RCLCPP_WARN(node_->get_logger(), "[SLAM source] Starting robust lifecycle restart sequence...");

    // Auxiliary Lambda: Send life cycle state transition request
    auto send_transition = [this](uint8_t id, const std::string& desc) -> bool {
        auto client = this->lifecycle_client_;
        if (!client || !client->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(node_->get_logger(), "[SLAM source] Service for %s not available", desc.c_str());
            return false;
        }

        auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        req->transition.id = id;
        auto future = client->async_send_request(req);

        if (future.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
            RCLCPP_INFO(node_->get_logger(), "[SLAM source] %s transition succeeded", desc.c_str());
            return true;
        } else {
            RCLCPP_ERROR(node_->get_logger(), "[SLAM source] %s transition failed", desc.c_str());
            return false;
        }
    };

    // Auxiliary Lambda: Wait for the node to reach the desired state
    auto wait_for_state = [this](uint8_t expected_state, const std::string& state_name) -> bool {
        (void)state_name;
        auto client = this->get_state_client_;
        if (!client || !client->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(node_->get_logger(), "[SLAM source] GetState service not available");
            return false;
        }

        auto req = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
        auto future = client->async_send_request(req);

        if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
            RCLCPP_ERROR(node_->get_logger(), "[SLAM source] GetState request failed");
            return false;
        }

        return future.get()->current_state.id == expected_state;
    };

    // Execute the life cycle restart sequence: Deactivate -> Cleanup -> Configure -> Activate
    if (!send_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, "DEACTIVATE")) return;
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    if (!wait_for_state(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "INACTIVE")) return;

    if (!send_transition(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP, "CLEANUP")) return;
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    if (!wait_for_state(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "UNCONFIGURED")) return;

    if (!send_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, "CONFIGURE")) return;
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    if (!wait_for_state(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "INACTIVE")) return;

    send_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, "ACTIVATE");
    RCLCPP_INFO(node_->get_logger(), "[SLAM source] Lifecycle restart sequence completed.");
}

void SLAM::setHealthy(bool healthy) {
    SLAM_healthy_ = healthy;
}

void SLAM::setOffset(const Eigen::Vector3d& p, const Eigen::Quaterniond& q, double yaw) {
    // Set the offset of the SLAM coordinate system relative to the output coordinate system
    slam_offset_p_ = p;
    slam_offset_q_ = q;
    slam_offset_yaw_ = yaw;

    // Update the coordinate system of the current propagation (IMU prediction) data
    propageted_data_.q = (slam_offset_q_ * propageted_data_.q).normalized();
    propageted_data_.p = (slam_offset_q_ * propageted_data_.p) + slam_offset_p_;
    propageted_data_.yaw = slam_offset_yaw_ + propageted_data_.yaw;
    propageted_data_.v = slam_offset_q_ * propageted_data_.v;
    
    // RCLCPP_INFO(node_->get_logger(), "[slam source] set offset");
    // std::cout << slam_offset_p_ <<std::endl;
    // std::cout <<"offset" << latest_slam_p_ << std::endl;
} 

void SLAM::setOdometry() {
    // Apply the offset to the latest SLAM data and store it in odom_data_
    odom_data_.q = (slam_offset_q_ * latest_slam_q_).normalized();
    odom_data_.p = (slam_offset_q_ * latest_slam_p_) + slam_offset_p_;
    odom_data_.v = slam_offset_q_ * latest_slam_v_;
    odom_data_.a = latest_slam_a_;
    odom_data_.yaw = latest_slam_yaw_ + slam_offset_yaw_ ;
}

void SLAM::setSlamdata() {
    // Process the original SLAM data, here it is mainly used to save the original data and call setOdometry
    
    // latest_slam_q_ = (restart_offset_q_ * slam_offset_q_ * r_slam_q_).normalized();
    // latest_slam_p_ = (restart_offset_q_ * slam_offset_q_* r_slam_p_) + slam_offset_p_ + restart_offset_p_;
    // latest_slam_v_ = restart_offset_q_ * slam_offset_q_* r_slam_v_;
    // latest_slam_a_ = r_slam_a_;
    // latest_slam_yaw_ = r_slam_yaw_ + slam_offset_yaw_ + restart_offset_yaw_;
    latest_slam_q_ = r_slam_q_;
    latest_slam_p_ = r_slam_p_;
    latest_slam_v_ = r_slam_v_;
    latest_slam_a_ = r_slam_a_;
    latest_slam_yaw_ = r_slam_yaw_;
    setOdometry();
}

void SLAM::setCurrPose(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, const Eigen::Quaterniond& q) {
    // Update current propagation status (for high frequency output)
    propageted_data_.q = (slam_offset_q_ * q).normalized();
    propageted_data_.p = (slam_offset_q_ * pos) + slam_offset_p_;
    propageted_data_.v = slam_offset_q_ * vel;
    propageted_data_.yaw = latest_slam_yaw_ + slam_offset_yaw_;
} 

void SLAM::setRestartOffset(NavState latest_slam_data) {
    // Set the offset when restarting (used to maintain trajectory continuity after restarting, part of the logic in the code is currently commented)
    restart_offset_p_ = latest_slam_data.p;
    restart_offset_q_ = latest_slam_data.q;
    restart_offset_yaw_ = latest_slam_data.yaw;
}

NavState SLAM::getOdometry() const {
    return odom_data_;
}

NavState SLAM::getPropagateOdometry() const {
    return propageted_data_;
}

void SLAM::slamCallback(const xion_msg::msg::ExtendedOdometry::SharedPtr msg) 
{
    // RCLCPP_INFO(node_->get_logger(), "[SLAM source] data received");
    if (!msg){
        RCLCPP_WARN(node_->get_logger(), "[SLAM source] data is null");
        setHealthy(false);
        return;
    }

    // Update last received time for timeout checking
    last_get_slam_time_ = node_->now();
    receiving_slam_ = true;

    timestamp = msg->header.stamp;


    // Extract position, attitude, speed and other information from messages
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
    // Calculate Yaw angle
    r_slam_yaw_ = std::atan2(2.0*(r_slam_q_.w()*r_slam_q_.z() + r_slam_q_.x()*r_slam_q_.y()),
                        1.0 - 2.0*(r_slam_q_.y()*r_slam_q_.y() + r_slam_q_.z()*r_slam_q_.z()));
    latest_Ba_ << msg->bas.x,
                msg->bas.y,
                msg->bas.z;
    latest_Bg_ << msg->bgs.x,
                msg->bgs.y,
                msg->bgs.z;
    RCLCPP_INFO(node_->get_logger(), "[SLAM source] received yaw: %f", r_slam_yaw_);
    RCLCPP_INFO(node_->get_logger(), "[SLAM source] received pos: [%f, %f, %f], vel: [%f, %f, %f], q: [%f, %f, %f, %f]", 
        r_slam_p_(0), r_slam_p_(1), r_slam_p_(2),
        r_slam_v_(0), r_slam_v_(1), r_slam_v_(2),
        r_slam_q_.w(), r_slam_q_.x(), r_slam_q_.y(), r_slam_q_.z());
    
    // Apply offsets and update internal state
    setSlamdata();

    // If the initial check is not completed, perform the check
    if (initial_check_state_ != InitialCheckState::COMPLETE) {
        // Check using original, unoffset and unrotated velocities
        // performInitialStillnessCheck(r_slam_v_);
        initial_check_state_ = InitialCheckState::COMPLETE;
        return; // 在检查期间，不进行后续的健康检查
    } else {
        // Using EKF for health checks
        //1. Construct observation vector z
        Matrix<double, M_OBS_POS + M_OBS_VEL + M_OBS_ORI, 1> z;
        z.block<3, 1>(0, 0) = r_slam_p_; // Use raw data as observations
        z.block<3, 1>(3, 0) = r_slam_v_;
        z.block<4, 1>(6, 0) << r_slam_q_.x(), r_slam_q_.y(), r_slam_q_.z(), r_slam_q_.w();

        // 2. Call EKF update and set the health status based on the return value
        bool is_healthy = ekf_->update(z);
        setHealthy(is_healthy);
        if (!is_healthy) {
            RCLCPP_WARN(node_->get_logger(), "[SLAM source] EKF innovation check failed. Marked as unhealthy.");
        }
    }

    receiving_slam_ = false;
    // restartRequested();
}

void SLAM::timerCallback() {

    // This function can now be cleared or removed since the health check has been moved to slamCallback

    // if (!odom_pending_compare_.load()) return;

    // // If the initial check has not been completed, the regular health check is not performed
    // if (initial_check_state_ != InitialCheckState::COMPLETE) {
    //     odom_pending_compare_.store(false);
    //     return;
    // }

    // // std::cout << " 2" <<std::endl;
    // const double t0 = last_odom_stamp_.seconds();
    // const double t1 = curr_odom_stamp_.seconds();

    // if (t1 <= t0) {
    //     odom_pending_compare_.store(false);
    //     RCLCPP_WARN(node_->get_logger(),"[SLAM source] t1 <= t0");
    //     return;
    // }

    // std::vector<ImuLite> seg;
    // if (!extractImuInterval_(t0, t1, seg)) {
    //     RCLCPP_WARN(node_->get_logger(),"[SLAM source] not extractImuInterval_");
    //     // Insufficient IMU samples, skip this time and wait for next time
    //     odom_pending_compare_.store(false);
    //     return;
    // }
    // Eigen::Vector3d dvel_imu;
    // integrateIntervalMidpoint_(seg, dvel_imu);
    // integrated_v_imu_ = curr_odom_state_.v+  dvel_imu;
    // const Eigen::Vector3d v_slam = curr_odom_state_.v;

    // double diff = (v_slam - integrated_v_imu_).norm();

    // double angle = 0.0;
    // if (v_slam.norm() > 1e-3 && integrated_v_imu_.norm() > 1e-3) {
    //     double c = v_slam.normalized().dot(integrated_v_imu_.normalized());
    //     c = std::clamp(c, -1.0, 1.0);
    //     angle = std::acos(c) * 180.0 / M_PI;
    // }
    // RCLCPP_WARN(node_->get_logger(), "[SLAM source] speed difference = %f, angle = %f", diff, angle);
    // if (diff > maxSpeeddiff_ && angle > maxAnglediff_) {
    //     setHealthy(false);
    //     RCLCPP_WARN(node_->get_logger(), "[SLAM source] big difference :speed difference = %f, angle = %f", diff, angle);
    // } else {
    //     //std::cout << "speed difference = " << diff << ", angle = " << angle << std::endl;
    //     setHealthy(true);
    // }


    // if (!receiving_slam_) {
    //     rclcpp::Time now_ = node_->now();
    //     if (last_slam_time_.seconds() == 0){
    //         last_slam_time_ = now_;
    //         return;
    //     }

    //     double dt = (now_ - last_slam_time_).seconds();
    //     last_slam_time_ = now_;

    //     imu_acc_.z() -= 9.81;

    //     integrated_v_imu_ += imu_acc_ * dt;
    //     // Calculate the difference in magnitude
    //     double diff = (latest_slam_v_ - integrated_v_imu_).norm();
    //     double angle = 0.0;
    //     if (latest_slam_v_.norm() > 1e-3 && integrated_v_imu_.norm() > 1e-3) {
    //         double cos_angle = latest_slam_v_.normalized().dot(integrated_v_imu_.normalized());
    //         cos_angle = std::clamp(cos_angle, -1.0, 1.0);
    //         angle = std::acos(cos_angle) * 180.0 / M_PI;
    //     }

    //     if (diff > maxSpeeddiff_ && angle > maxAnglediff_)
    //     {
    //         setHealthy(false);
    //         RCLCPP_WARN(node_->get_logger(), "[SLAM source] speed difference = %f, angle = %f", diff, angle);
    //     }else {
    //         setHealthy(true);
    //     }
    // }

    
}

// void SLAM::onSetRestartReq(
//     const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
//     std::shared_ptr<std_srvs::srv::SetBool::Response> resp) {
    
//     restart_srv = req->data; 
//     resp->success = true;
//     resp->message = req->data ? "restart_requested_ = true"
//                               : "restart_requested_ = false";
// }

// void SLAM::restartRequested() {
//    if (SLAM_healthy_ == false && is_received_message_) {
//         if (unhealthy_start_time_.nanoseconds() == 0) {
//             unhealthy_start_time_ = node_->now();
//         }
//         rclcpp::Time un_health_time = node_->now();
//         double duration = (un_health_time - unhealthy_start_time_).seconds();
//         if (duration > 2.0){
//             RCLCPP_ERROR(node_->get_logger(), "SLAM unhealthy for 2 seconds. Requesting restart...");

//             restart_requested_ = true;
//             unhealthy_start_time_ = rclcpp::Time(0, 0);
//         }
//     }else {
//         unhealthy_start_time_ = rclcpp::Time(0, 0);
//     }
    
// }

bool SLAM::restart() {
    return restart_requested_;
}

void SLAM::setRestartquest(bool restart) {
    restart_requested_ = restart;
}

void SLAM::setImudata(const Eigen::Vector3d& linearAcceleration,
                      const Eigen::Vector3d& angularVelocity,
                      const Eigen::Quaterniond& orientation,
                      const rclcpp::Time imu_time_stamp) {
    // buffer
    // imu_acc_ = linearAcceleration;
    // imu_gyro_ = angularVelocity;
    // imu_orientation_ = orientation;

    // Check the time difference between IMU and SLAM odometry
    if(timestamp.seconds() != 0){
        auto time_diff = (imu_time_stamp - timestamp).seconds();
        RCLCPP_INFO(node_->get_logger(), "IMU and SLAM Odometry time diff: %f seconds", time_diff);
        timestamp = rclcpp::Time(0);
    }
    
    if(restarting_) return;
    const double t = imu_time_stamp.seconds();
    std::lock_guard<std::mutex> lk(imu_mtx_);

    // Perform EKF prediction step (update state covariance) using IMU data
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

    propagate_time_ = imu_time_stamp;

    // initial propagation time
    if (last_propagate_time_.seconds() == 0){
        last_propagate_time_ = propagate_time_;
        latest_slam_acc_0 = linearAcceleration;
        latest_slam_gyr_0 = angularVelocity;
        return;
    }

    // Calculate time intervals for kinematic integral prediction (for high frequency output)
    double dt_slam = (propagate_time_ - last_propagate_time_).seconds();
    RCLCPP_INFO(node_->get_logger(),"dt_slam= %f", dt_slam);
    last_propagate_time_ = propagate_time_;

    if (dt_slam <= 0.0 || dt_slam >1.0) 
    {
        RCLCPP_WARN(node_->get_logger(), "[SLAM source] Invalid slam dt = %f", dt_slam);
        return;
    }
    // Median integration method updates pose and velocity
    Eigen::Vector3d un_slam_acc_0 = latest_slam_q_ *(latest_slam_acc_0 - latest_Ba_) - g_;
    Eigen::Vector3d un_slam_gyr = 0.5 * (latest_slam_gyr_0 + angularVelocity) - latest_Bg_;
    latest_slam_q_ = latest_slam_q_ * deltaQ(un_slam_gyr * dt_slam);
    Eigen::Vector3d un_slam_acc_1 = latest_slam_q_ *(linearAcceleration - latest_Ba_) - g_;
    Eigen::Vector3d un_slam_acc = 0.5 * (un_slam_acc_0 + un_slam_acc_1);
    latest_slam_p_ = latest_slam_p_ + latest_slam_v_ * dt_slam + 0.5 * un_slam_acc * dt_slam * dt_slam;
    latest_slam_v_ = latest_slam_v_ + un_slam_acc * dt_slam;
    latest_slam_acc_0 = linearAcceleration;
    latest_slam_gyr_0 = angularVelocity;

    setCurrPose(latest_slam_p_, latest_slam_v_, latest_slam_q_); // Update the current pose cache
}

bool SLAM::isHealthy() {
    // Check if data reception times out
    rclcpp::Time now_ = node_->now();
    double dt = (now_ - last_get_slam_time_).seconds();
    // RCLCPP_INFO(node_->get_logger(), "time diff = %f", dt);
    if (dt > 1.0) {
        RCLCPP_WARN(node_->get_logger(), "[SLAM source] time diff = %f", dt);
        // RCLCPP_WARN(node_->get_logger(), "[SLAM source] SLAM data empty.");
        is_received_message_ = false;
        return false;
    }
    // As long as there is data and the EKF check passes, it is considered healthy
    is_received_message_ = true;
    return SLAM_healthy_;
}
