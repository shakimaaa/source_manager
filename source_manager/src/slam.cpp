#include "source_manager/slam.hpp"

SLAM::SLAM(rclcpp::Node::SharedPtr node)
    : SourceBase(std::move(node)) {

    node_->declare_parameter("slam.maxSpeeddiff", 4.0);
    node_->declare_parameter("slam.maxAnglediff", 100.0);
    node_->declare_parameter("slam.canrestart", true);

    node_->get_parameter("slam.maxSpeeddiff", maxSpeeddiff_);
    node_->get_parameter("slam.maxAnglediff", maxAnglediff_);
    node_->get_parameter("slam.canrestart", can_restart_);

    slam_callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    
    rclcpp::SubscriptionOptions slam_sub_opt;
    slam_sub_opt.callback_group = slam_callback_group_;

    slam_sub_ = node_->create_subscription<xion_msg::msg::ExtendedOdometry>(
        "/position/slam_odom",10,
        std::bind(&SLAM::slamCallback, this, std::placeholders::_1),
        slam_sub_opt);
    
    slam_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(400),
        std::bind(&SLAM::timerCallback, this));
    
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
    if (!can_restart_) {
        return false;
    }

    if (SLAM_healthy_ == false && is_received_message_) {
        if (unhealthy_start_time_.nanoseconds() == 0) {
            unhealthy_start_time_ = node_->now();
        }
        rclcpp::Time un_health_time = node_->now();
        double duration = (un_health_time - unhealthy_start_time_).seconds();
        if (duration > 2.0){
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
    slam_offset_p_ = p;
    slam_offset_q_ = q;
    slam_offset_yaw_ = yaw;
}

void SLAM::setOdometry() {
    odom_data_.p = latest_slam_p_;
    odom_data_.q = latest_slam_q_;
    odom_data_.v = latest_slam_v_;
    odom_data_.a = latest_slam_a_;
    odom_data_.yaw = latest_slam_yaw_;
}

void SLAM::setSlamdata() {
    latest_slam_p_ = r_slam_p_ + slam_offset_p_ + restart_offset_p_;
    latest_slam_q_ = r_slam_q_ * slam_offset_q_ * restart_offset_q_;
    latest_slam_v_ = r_slam_v_;
    latest_slam_a_ = r_slam_a_;
    latest_slam_yaw_ = r_slam_yaw_ + slam_offset_yaw_ + restart_offset_yaw_;
    setOdometry();
}

void SLAM::setCurrPose(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, const Eigen::Quaterniond& q) {
    propageted_data_.p = pos;
    propageted_data_.v = vel;
    propageted_data_.q = q;
    propageted_data_.yaw = latest_slam_yaw_;
}

void SLAM::setRestartOffset(NavState latest_slam_data) {
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
    RCLCPP_INFO(node_->get_logger(), "[SLAM source] data received");
    if (!msg){
        RCLCPP_WARN(node_->get_logger(), "[SLAM source] data is null");
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
    RCLCPP_INFO(node_->get_logger(), "[SLAM source] received yaw: %f", r_slam_yaw_);
    RCLCPP_INFO(node_->get_logger(), "[SLAM source] received pos: [%f, %f, %f], vel: [%f, %f, %f], q: [%f, %f, %f, %f]", 
        r_slam_p_(0), r_slam_p_(1), r_slam_p_(2),
        r_slam_v_(0), r_slam_v_(1), r_slam_v_(2),
        r_slam_q_.w(), r_slam_q_.x(), r_slam_q_.y(), r_slam_q_.z());
    
    setSlamdata();
    receiving_slam_ = false;
    // restartRequested();
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

        if (diff > maxSpeeddiff_ && angle > maxAnglediff_)
        {
            setHealthy(false);
            RCLCPP_WARN(node_->get_logger(), "[SLAM source] speed difference = %f, angle = %f", diff, angle);
        }else {
            setHealthy(true);
        }
    }

    
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
                      const Eigen::Quaterniond& orientation) {
    imu_acc_ = linearAcceleration;
    imu_gyro_ = angularVelocity;
    imu_orientation_ = orientation;

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

bool SLAM::isHealthy() {
    rclcpp::Time now_ = node_->now();
    double dt = (now_ - last_get_slam_time_).seconds();
    // RCLCPP_INFO(node_->get_logger(), "time diff = %f", dt);
    if (dt > 1.0) {
        // RCLCPP_WARN(node_->get_logger(), "SLAM data empty.");
        return false;
    }
    is_received_message_ = true;
    return SLAM_healthy_;
}
