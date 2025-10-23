#include "source_manager/source_manager.hpp"

SourceManager::SourceManager() : Node("SourceManager")
{

    this->declare_parameter<std::vector<int64_t>>("manager.priority_source", std::vector<int64_t>{2, 1});

    this->get_parameter("manager.priority_source", pri_raw);
    priority_source_.clear();
    priority_source_.reserve(pri_raw.size());
    for (auto v : pri_raw) {
        if (v == 0)      priority_source_.push_back(SourceBase::State::UNINIT);
        else if (v == 1) priority_source_.push_back(SourceBase::State::SLAM);
        else if (v == 2) priority_source_.push_back(SourceBase::State::GPS);
    }

    std::string order;
    order.reserve(64);
    for (size_t i = 0; i < priority_source_.size(); ++i) {
        if (i) order += " > ";
        order += stateToString(priority_source_[i]);
    }
    RCLCPP_INFO(this->get_logger(), "priority order: %s", order.c_str());

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/mav/imu/data_raw", rclcpp::QoS(100).best_effort(),
        std::bind(&SourceManager::imuCallback, this, std::placeholders::_1));

    propagate_odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/imu_propagate_", rclcpp::QoS(10).best_effort().durability_volatile());

    odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/odometry_",1000);
    
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "/path_",1000);
    
    switch_source_srv_ = create_service<xion_msg::srv::SwitchSourceType>(
        "switch_source",
        std::bind(&SourceManager::onSwitchSource, this,
        std::placeholders::_1, std::placeholders::_2));
    
    set_restart_req_ = this->create_service<std_srvs::srv::SetBool>(
        "set_restart_requested",
        std::bind(&SourceManager::onSetRestartReq, this, std::placeholders::_1, std::placeholders::_2));
    
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&SourceManager::timerCallback, this));
    
    // lifecycle_client_ = this->create_client<lifecycle_msgs::srv::ChangeState>("/xvins_lifecycle_node/change_state");
    // get_state_client_ = this->create_client<lifecycle_msgs::srv::GetState>("/xvins_lifecycle_node/get_state");

    RCLCPP_INFO(this->get_logger(), "SourceManager node statrted.");
}

void SourceManager::init() {
    auto self = shared_from_this();
    gps_source_ = std::make_unique<GPS>(self);
    slam_source_ = std::make_unique<SLAM>(self);
}

void SourceManager::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "IMU data received");

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
    
    // RCLCPP_INFO(this->get_logger(), "IMU data raw: [%f %f %f %f] ", orientation.w(), orientation.x(), orientation.y(), orientation.z());
    
    if (gps_source_) gps_source_->setImudata(linearAcceleration, angularVelocity, orientation);
    if (slam_source_) slam_source_->setImudata(linearAcceleration, angularVelocity, orientation);

    update();
    publishPropagateOdometry();
    
}

std::shared_ptr<SourceBase> SourceManager::getSource(SourceBase::State state) {
    switch (state) {
        case SourceBase::State::GPS: return gps_source_;
        case SourceBase::State::SLAM: return slam_source_;
        default: return nullptr;
    }

}

void SourceManager::onSetRestartReq(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                         std::shared_ptr<std_srvs::srv::SetBool::Response> resp) {

    restart_srv = req->data; 
    resp->success = true;
    resp->message = req->data ? "restart_requested_ = true"
                              : "restart_requested_ = false";
}

void SourceManager::restartCheck() {
    // xie cheng yige
    if (restart_srv || slam_source_->canRestart()) {
        slam_source_->setRestartOffset(current_propagate_state_);
        std::thread([this]() {
           slam_source_->restartSource();
        }).detach();
        
    }

    if (restart_srv || gps_source_->canRestart()) {
        std::thread([this]() {
           gps_source_->restartSource();
        }).detach();
    }
    restart_srv = false;
}

void SourceManager::raisePriority(SourceBase::State target) {
    // mutx
    auto it = std::find(priority_source_.begin(), priority_source_.end(), target);
    if (it != priority_source_.end()) {
        priority_source_.erase(it);
    }
    
    priority_source_.insert(priority_source_.begin(), target);
}

bool SourceManager::canRaisePriority(SourceBase::State target, std::string &reason) const{
    if (target == SourceBase::State::GPS) {
        if (!gps_source_ || !gps_source_->isHealthy()) {
            reason = "GPS is not healthy or not ready.";
            return false;
        }
    } else if (target == SourceBase::State::SLAM) {
        if (!slam_source_ || !slam_source_->isHealthy()) {
            reason = "SLAM is not healthy or not ready.";
            return false;
        }
    } else {
        reason = "Invalid target source.";
        return false;
    }
    return true;

    // if (active_source_ == target) {
    //     reason = std::string("Already on target: ") + stateToString(target);
    //     return false;
    // }
    // if (target == SourceBase::State::GPS) {
    //     if (!gps_source_ || !gps_source_->isHealthy()) {
    //         reason = "GPS is not healthy or not ready.";
    //         return false;
    //     }
    // } else if (target == SourceBase::State::SLAM) {
    //     if (!slam_source_ || !slam_source_->isHealthy()) {
    //         reason = "SLAM is not healthy or not ready.";
    //         return false;
    //     }
    // } else {
    //     reason = "Invalid target source.";
    //     return false;
    // }
    // return true;
}

void SourceManager::onSwitchSource(const std::shared_ptr<xion_msg::srv::SwitchSourceType::Request> req,
                                    std::shared_ptr<xion_msg::srv::SwitchSourceType::Response> res) {
    SourceBase::State target;
    if (req->target == 0)      target = SourceBase::State::GPS;
    else if (req->target == 1) target = SourceBase::State::SLAM;
    else {
        res->success = false;
        res->message = "Bad target value. Use 0=GPS, 1=SLAM (set priority).";
        RCLCPP_WARN(this->get_logger(), "Priority change denied: %s", res->message.c_str());
        return;
    }

    std::string reason;
    if (!canRaisePriority(target, reason)) {
        res->success = false;
        res->message = reason;
        RCLCPP_WARN(this->get_logger(), "Priority change denied: %s", reason.c_str());
        return;
    }

    raisePriority(target);

    // manual_pin_active_ = true;
    // manual_pin_target_ = target;

    // manual_request_ = target;
    res->success = true;
    res->message = std::string("Priority updated. Top preference: ") + stateToString(target);
    RCLCPP_WARN(this->get_logger(), "Priority -> [%s ...]", stateToString(target));
}

// void SourceManager::changeSourcefromsrv(){
//     if (manual_request_.has_value()) {
//         std::string reason;
//         if (canRaisePriority(*manual_request_, reason)) {
//             active_source_ = *manual_request_;
//             RCLCPP_WARN(this->get_logger(), "Manual override applied -> %s",
//                         stateToString(active_source_));
//         } else {
//             RCLCPP_WARN(this->get_logger(), "Manual override canceled: %s", reason.c_str());
//         }
//         manual_request_.reset();
//     }
//     if (manual_pin_active_) {
//         bool target_healthy = (manual_pin_target_ == SourceBase::State::GPS) 
//                                 ? (gps_source_ && gps_source_->isHealthy())
//                                 : (manual_pin_target_ == SourceBase::State::SLAM
//                                     ? (slam_source_ && slam_source_->isHealthy())
//                                     : false);
//         if (target_healthy) {
//             if (active_source_ != manual_pin_target_) {
//                 RCLCPP_INFO(this->get_logger(), "Pin override: active -> %s",
//                             stateToString(manual_pin_target_));
//             }
//         } else {
//             // When the pinned source becomes unhealthy, unpin and fall back to AUTO mode
//             RCLCPP_WARN(this->get_logger(),
//                         "Pinned source %s became unhealthy. Unpinning and falling back to AUTO.",
//                         stateToString(manual_pin_target_));
//             manual_pin_active_ = false;
//             manual_pin_target_ = SourceBase::State::UNINIT;
//         }
//     }
// }

void SourceManager::checkSourceHealth() {
    const bool gps_healthy = gps_source_ ? gps_source_->isHealthy() : false;
    const bool slam_healthy = slam_source_ ? slam_source_->isHealthy() : false;

    SourceBase::State state = SourceBase::State::UNINIT;

    auto healthy = [&](SourceBase::State s)->bool {
        switch (s) {
            case SourceBase::State::GPS: return gps_healthy;
            case SourceBase::State::SLAM: return slam_healthy;
            default: return false;
        }
    };

    for (auto s : priority_source_) {
        if (healthy(s)) {
            state = s;
            break;
        }
    }

    // for (auto s : priority_source_) {
    //     auto src = getSource(s);
    //     if (src = nullptr) continue;
    //     if (src && src->isHealthy()) {
            // std::cout<< "1" <<std::endl;    
    //         state = s;
    //         break;
    //     }
    // }
    // active_source_ = state;

    if (state != active_source_) {
        previous_source_ = active_source_;
        active_source_   = state;
        RCLCPP_WARN(this->get_logger(), "AUTO select: %s", stateToString(active_source_));
    }

    RCLCPP_INFO(this->get_logger(), "Current source: %s healthy GPS: %s, healthy SLAM: %s",
                stateToString(active_source_),
                gps_healthy ? "true" : "false",
                slam_healthy ? "true" : "false");
}

void SourceManager::update() {
    auto current_source  = getSource(active_source_);
    if (!current_source) return;
    current_propagate_state_ = current_source->getPropagateOdometry();

    // previous_source_ = active_source_; // Update previous source
}

void SourceManager::changeSourceType() {
    if (active_source_ != previous_source_ && previous_source_ != SourceBase::State::UNINIT) {
        RCLCPP_INFO(this->get_logger(), "State changed to: %d", static_cast<int>(active_source_));
        is_state_changed_ = true;
    }

    if (is_state_changed_) {
        RCLCPP_INFO(this->get_logger(), "State transition started");

        auto previous_source = getSource(previous_source_);
        auto current_source = getSource(active_source_);
        if (previous_source == nullptr || current_source == nullptr) {
            previous_source_ = active_source_;
            is_state_changed_ = false;
            return;
        }
        auto odom = previous_source->getOdometry();

        const Eigen::Vector3d pos = odom.p;
        const double yaw = odom.yaw;
        //const Eigen::Quaterniond q = odom.q;
        
        double yaw_offset = normalizeAngle(current_propagate_state_.yaw - yaw);
        Eigen::Quaterniond q_offset (Rz(yaw_offset));
        Eigen::Vector3d pos_offset = (current_propagate_state_.p - Rz(yaw_offset) * pos);
        current_source->setOffset(pos_offset, q_offset, yaw_offset);

        transition_start_time_ = this->now();
        is_transitioning_ = true;
        is_state_changed_ = false;
        previous_source_ = active_source_;
    }
}

void SourceManager::interpolationFilter() {
    if (is_transitioning_) {
        rclcpp::Duration elapsed = this->now() - transition_start_time_;
        const auto trans = rclcpp::Duration::from_seconds(transition_duration_sec_);

        if (elapsed > trans) {
            current_propagate_state_.p = position_target_;
            current_propagate_state_.q = orientation_target_;
            is_transitioning_ = false;

            RCLCPP_INFO(this->get_logger(), "Smooth transition completed.");
        } else {
            double alpha = elapsed.seconds() / transition_duration_sec_;
            Eigen::Vector3d interp_position = position_start_ + alpha * (position_target_ - position_start_);
            Eigen::Quaterniond interp_orientation = orientation_start_.slerp(alpha, orientation_target_);
            current_propagate_state_.p = interp_position;
            current_propagate_state_.q = interp_orientation;
        }
    }
}

void SourceManager::publishPropagateOdometry() {

    if (active_source_ == SourceBase::State::UNINIT) {
        return;
    }


    nav_msgs::msg::Odometry propagated_odometry;
    propagated_odometry = current_propagate_state_.toRosMsg();
    propagated_odometry.header.stamp = this->now();
    propagated_odometry.header.frame_id = "world";

    
    
    // RCLCPP_INFO(this->get_logger(), "imu_propagate pose: [%f %f %f] vel: [%f %f %f], q: [%f %f %f %f] ", 
    //    propagated_odometry.pose.pose.position.x, propagated_odometry.pose.pose.position.y, propagated_odometry.pose.pose.position.z,
    //    propagated_odometry.twist.twist.linear.x, propagated_odometry.twist.twist.linear.y, propagated_odometry.twist.twist.linear.z,
     //   propagated_odometry.pose.pose.orientation.w, propagated_odometry.pose.pose.orientation.x, propagated_odometry.pose.pose.orientation.y, propagated_odometry.pose.pose.orientation.z);
    propagate_odometry_pub_->publish(propagated_odometry);
}

void SourceManager::publishOdometry() {
    if (active_source_ == SourceBase::State::UNINIT) {
        return;
    }
    auto odom = getSource(active_source_)->getOdometry();
    nav_msgs::msg::Odometry odometry;
    odometry = odom.toRosMsg();
    
    odometry.header.stamp = this->now();
    odometry.header.frame_id = "world";
    odometry_pub_->publish(odometry);

    

    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp = this->now();
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.position.x = odom.p.x();
    pose_stamped.pose.position.y = odom.p.y();
    pose_stamped.pose.position.z = odom.p.z();
    
    path_history_.push_back(pose_stamped);
    nav_msgs::msg::Path path_;
    path_.header.stamp = this->now();
    path_.header.frame_id = "world";
    path_.poses = path_history_;
    path_pub_->publish(path_);
}

// void SourceManager::requestLifecycleRestart() {
//     RCLCPP_WARN(this->get_logger(), "Starting robust lifecycle restart sequence...");

//     auto send_transition = [this](uint8_t id, const std::string& desc) -> bool {
//         auto client = this->lifecycle_client_;
//         if (!client || !client->wait_for_service(std::chrono::seconds(2))) {
//             RCLCPP_ERROR(this->get_logger(), "Service for %s not available", desc.c_str());
//             return false;
//         }

//         auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
//         req->transition.id = id;
//         auto future = client->async_send_request(req);

//         if (future.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
//             RCLCPP_INFO(this->get_logger(), "%s transition succeeded", desc.c_str());
//             return true;
//         } else {
//             RCLCPP_ERROR(this->get_logger(), "%s transition failed", desc.c_str());
//             return false;
//         }
//     };

//     auto wait_for_state = [this](uint8_t expected_state, const std::string& state_name) -> bool {
//         (void)state_name;
//         auto client = this->get_state_client_;
//         if (!client || !client->wait_for_service(std::chrono::seconds(2))) {
//             RCLCPP_ERROR(this->get_logger(), "GetState service not available");
//             return false;
//         }

//         auto req = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
//         auto future = client->async_send_request(req);

//         if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
//             RCLCPP_ERROR(this->get_logger(), "GetState request failed");
//             return false;
//         }

//         return future.get()->current_state.id == expected_state;
//     };

//     if (!send_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, "DEACTIVATE")) return;
//     rclcpp::sleep_for(std::chrono::milliseconds(100));
//     if (!wait_for_state(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "INACTIVE")) return;

//     if (!send_transition(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP, "CLEANUP")) return;
//     rclcpp::sleep_for(std::chrono::milliseconds(100));
//     if (!wait_for_state(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "UNCONFIGURED")) return;

//     if (!send_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, "CONFIGURE")) return;
//     rclcpp::sleep_for(std::chrono::milliseconds(100));
//     if (!wait_for_state(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "INACTIVE")) return;

//     send_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, "ACTIVATE");
//     RCLCPP_INFO(this->get_logger(), "Lifecycle restart sequence completed.");
// }

// void SourceManager::onSlamRestartRequested()
// {
//     std::thread([this]() {
//         this->requestLifecycleRestart();
//     }).detach();
    
// }

void SourceManager::timerCallback() {
    checkSourceHealth();
    restartCheck();
    // if (slam_source_->restart()) {
    //     RCLCPP_ERROR(this->get_logger(), "SLAM restart requested 1");
    //     onSlamRestartRequested();

    //     slam_source_->setRestartquest(false);
    // }
    // changeSourcefromsrv();
    
    changeSourceType();

    interpolationFilter();
    
    publishOdometry();
    // publishPropagateOdometry();
}

