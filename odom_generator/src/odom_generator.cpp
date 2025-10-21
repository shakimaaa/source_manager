#include "odom_generator/odom_generator.hpp"


OdomGenerator::OdomGenerator() : Node("odom_generator") {
    gps_sub_ = this->create_subscription<xion_msg::msg::GlobalPositionInt>(
        "/mav/global_position_int", rclcpp::QoS(100).best_effort(),
        std::bind(&OdomGenerator::gpsCallback, this, std::placeholders::_1));
    
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/mav/imu/data", rclcpp::QoS(100).best_effort(),
        std::bind(&OdomGenerator::imuCallback, this, std::placeholders::_1));
    
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/position/gps_odom", rclcpp::QoS(10).best_effort().durability_volatile());

    RCLCPP_INFO(this->get_logger(), "OdomGenerator node initialized.");
}

void OdomGenerator::gpsCallback(const xion_msg::msg::GlobalPositionInt::SharedPtr msg) {
    // Check if the GPS data is valid.
    if (msg->lat == 0) {
        RCLCPP_WARN(this->get_logger(), "GPS NOT READY");
        return;
    }

    if (msg->gps_status < 3) {
        RCLCPP_WARN(this->get_logger(), "fix_type <3, Invalid GPS data");
        if (origin_set_) {
            origin_set_ = false;
            RCLCPP_INFO(this->get_logger(), "Origin reset");
        }
        return;
    }
    // Set the origin if not already done.
    if (!origin_set_) {
        origin_ = *msg;
        origin_set_ = true;
        RCLCPP_INFO(this->get_logger(), "Origin set to lat=%f, lon=%f, alt=%f",
                    origin_.lat, origin_.lon, origin_.alt);
        return;
    }

    // -----------------------------------
    // Compute relative position in meters.
    // We assume:
    //   - Latitude difference -> North displacement
    //   - Longitude difference -> East displacement (scaled by cos(latitude))
    //   - Altitude difference -> Up displacement
    // -----------------------------------
    double d_lat = (msg->lat - origin_.lat) * 111139.0;  
    double d_lon = (msg->lon - origin_.lon) * 111139.0 * cos(origin_.lat * M_PI / 180.0);
    double d_alt = msg->alt - origin_.alt;

    // -----------------------------------
    // Create odometry message in the ENU frame.
    // In ENU:
    //   - x: East, y: North, z: Up.
    // -----------------------------------
    odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = "world";  // Global frame in ENU
    // odom_msg.child_frame_id = "base_link";   // Robot's body frame

    odom_msg.pose.pose.position.x = d_lon;  // East
    odom_msg.pose.pose.position.y = d_lat;  // North
    odom_msg.pose.pose.position.z = d_alt;  // Up
    // -----------------------------------
    // Orientation
    // -----------------------------------
    // Step 1: Convert GPS heading from degrees (from North) to radians.
    double gps_heading_rad = msg->hdg * M_PI / 180.0;

    // Step 2: Convert GPS heading (North-referenced) to ENU yaw.
    // In ENU, yaw=0 means facing East.
    // Thus, we define:
    double yaw_enu = M_PI/2 - gps_heading_rad;

    // Normalize yaw to the range [-pi, pi].
    yaw_enu = std::atan2(std::sin(yaw_enu), std::cos(yaw_enu));

    // Step 3: Get roll and pitch from the IMU.
    Eigen::Matrix3d R_enu = imu_orientation_.toRotationMatrix();
    double roll_enu = std::atan2(R_enu(2, 1), R_enu(2, 2));
    double pitch_enu = std::asin(-R_enu(2, 0));

    // Step 4: Combine the GPS yaw (converted to ENU) with the IMU roll and pitch.
    // Note: The multiplication order here is important.
    Eigen::Quaterniond q_des =
        Eigen::AngleAxisd(yaw_enu,       Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch_enu,     Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll_enu,      Eigen::Vector3d::UnitX());
    q_des.normalize();

    // For debugging: extract yaw from the constructed quaternion.
    double extracted_yaw = std::atan2(
        2.0 * (q_des.x() * q_des.y() + q_des.w() * q_des.z()),
        q_des.w()*q_des.w() + q_des.x()*q_des.x() - q_des.y()*q_des.y() - q_des.z()*q_des.z()
    );

    RCLCPP_INFO(this->get_logger(), "GPS heading (rad): %.6f, ENU yaw: %.6f, extracted_yaw: %.6f", 
                gps_heading_rad, yaw_enu, extracted_yaw);
    RCLCPP_INFO(this->get_logger(), "Final Quaternion: [w=%.6f, x=%.6f, y=%.6f, z=%.6f]",
                q_des.w(), q_des.x(), q_des.y(), q_des.z());

    // Set the final orientation (in ENU) in the odometry message.
    odom_msg.pose.pose.orientation.w = q_des.w();
    odom_msg.pose.pose.orientation.x = q_des.x();
    odom_msg.pose.pose.orientation.y = q_des.y();
    odom_msg.pose.pose.orientation.z = q_des.z();

    // -----------------------------------
    // Velocities
    // -----------------------------------
    // Assume GPS linear velocities (vx, vy, vz) are already in ENU.
    odom_msg.twist.twist.linear.x = msg->vy;  // East
    odom_msg.twist.twist.linear.y = msg->vx;  // North
    odom_msg.twist.twist.linear.z = - msg->vz;  // Up

    RCLCPP_INFO(this->get_logger(), "Odometry: [%.3f, %.3f, %.3f], velocities [%.3f, %.3f, %.3f], q[%.3f, %.3f, %.3f, %.3f]" ,
            odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z,
            odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y, odom_msg.twist.twist.linear.z,
            odom_msg.pose.pose.orientation.w, odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z);

    // Angular velocities from IMU (assumed to be already in ENU)
    odom_msg.twist.twist.angular.x = imu_angular_velocity_.x();
    odom_msg.twist.twist.angular.y = imu_angular_velocity_.y();
    odom_msg.twist.twist.angular.z = imu_angular_velocity_.z();

        // -----------------------------------
        // Covariance
        // -----------------------------------
        // If the fix_type is 3 (3D fix) or bigger than 3, then the covariance is valid which means covariance shuld be small.
        // if (msg->gps_status >= 3){
        //     for (double &c : odom_msg.pose.covariance) c = 0.0000001;
        //     for (double &c : odom_msg.twist.covariance) c = 0.0000001;
        // } else {
        //     for (double &c : odom_msg.pose.covariance) c = 1e6;
        //     for (double &c : odom_msg.twist.covariance) c = 1e6;
        // }
        
        // -----------------------------------
        // Publish the odometry message in the ENU frame.
        // -----------------------------------
        odom_pub_->publish(odom_msg);
    }

void OdomGenerator::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    // Update IMU orientation and angular velocity
    imu_orientation_ = Eigen::Quaterniond(
        msg->orientation.w,
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z);

    imu_angular_velocity_ = Eigen::Vector3d(
        msg->angular_velocity.x,
        msg->angular_velocity.y,
        msg->angular_velocity.z);
}

// class OdomGenerator : public rclcpp::Node
// {
// public:
//     OdomGenerator() : Node("odom_generator") {
//         // Subscribe to the GPS topic
//         gps_sub_ = this->create_subscription<xion_msg::msg::GlobalPositionInt>(
//             "/mav/global_position_int", rclcpp::QoS(100).best_effort(),
//             std::bind(&OdomGenerator::gpsCallback, this, std::placeholders::_1));
        
//         // Subscribe to the IMU topic
//         imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
//             "/mav/imu/data", rclcpp::QoS(100).best_effort(),
//             std::bind(&OdomGenerator::imuCallback, this, std::placeholders::_1));
        
//         // Publish odometry
//         odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom2_", rclcpp::QoS(10).best_effort().durability_volatile());

//         timer_ = this->create_wall_timer(
//             std::chrono::milliseconds(10),
//             std::bind(&OdomGenerator::timerCallback, this));

//         RCLCPP_INFO(this->get_logger(), "OdomGenerator node initialized.");
//     }

// private:
//     rclcpp::Subscription<xion_msg::msg::GlobalPositionInt>::SharedPtr gps_sub_;
//     rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
//     rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

//     rclcpp::TimerBase::SharedPtr timer_;
//     nav_msgs::msg::Odometry odom_msg;

//     xion_msg::msg::GlobalPositionInt origin_;
//     bool origin_set_ = false;

//     Eigen::Quaterniond imu_orientation_;
//     Eigen::Vector3d imu_angular_velocity_;

//     int fix_type_ = 0;

//     void timerCallback() {
        
//         odom_pub_->publish(odom_msg);
//     }

//     void gpsCallback(const xion_msg::msg::GlobalPositionInt::SharedPtr msg) {
//         // Check if the GPS data is valid.
//         if (msg->lat == 0) {
//             RCLCPP_WARN(this->get_logger(), "GPS NOT READY");
//             return;
//         }

//         if (msg->gps_status < 3) {
//             RCLCPP_WARN(this->get_logger(), "fix_type <3, Invalid GPS data");
//             if (origin_set_) {
//                 origin_set_ = false;
//                 RCLCPP_INFO(this->get_logger(), "Origin reset");
//             }
//             return;
//         }
//         // Set the origin if not already done.
//         if (!origin_set_) {
//             origin_ = *msg;
//             origin_set_ = true;
//             RCLCPP_INFO(this->get_logger(), "Origin set to lat=%f, lon=%f, alt=%f",
//                         origin_.lat, origin_.lon, origin_.alt);
//             return;
//         }

//         // -----------------------------------
//         // Compute relative position in meters.
//         // We assume:
//         //   - Latitude difference -> North displacement
//         //   - Longitude difference -> East displacement (scaled by cos(latitude))
//         //   - Altitude difference -> Up displacement
//         // -----------------------------------
//         double d_lat = (msg->lat - origin_.lat) * 111139.0;  
//         double d_lon = (msg->lon - origin_.lon) * 111139.0 * cos(origin_.lat * M_PI / 180.0);
//         double d_alt = msg->alt - origin_.alt;

//         // -----------------------------------
//         // Create odometry message in the ENU frame.
//         // In ENU:
//         //   - x: East, y: North, z: Up.
//         // -----------------------------------
//         odom_msg = nav_msgs::msg::Odometry();
//         odom_msg.header.stamp = this->now();
//         odom_msg.header.frame_id = "world";  // Global frame in ENU
//         // odom_msg.child_frame_id = "base_link";   // Robot's body frame

//         odom_msg.pose.pose.position.x = d_lon;  // East
//         odom_msg.pose.pose.position.y = d_lat;  // North
//         odom_msg.pose.pose.position.z = d_alt;  // Up
//         // -----------------------------------
//         // Orientation
//         // -----------------------------------
//         // Step 1: Convert GPS heading from degrees (from North) to radians.
//         double gps_heading_rad = msg->hdg * M_PI / 180.0;

//         // Step 2: Convert GPS heading (North-referenced) to ENU yaw.
//         // In ENU, yaw=0 means facing East.
//         // Thus, we define:
//         double yaw_enu = M_PI/2 - gps_heading_rad;

//         // Normalize yaw to the range [-pi, pi].
//         yaw_enu = std::atan2(std::sin(yaw_enu), std::cos(yaw_enu));

//         // Step 3: Get roll and pitch from the IMU.
//         Eigen::Matrix3d R_enu = imu_orientation_.toRotationMatrix();
//         double roll_enu = std::atan2(R_enu(2, 1), R_enu(2, 2));
//         double pitch_enu = std::asin(-R_enu(2, 0));

//         // Step 4: Combine the GPS yaw (converted to ENU) with the IMU roll and pitch.
//         // Note: The multiplication order here is important.
//         Eigen::Quaterniond q_des =
//             Eigen::AngleAxisd(yaw_enu,       Eigen::Vector3d::UnitZ()) *
//             Eigen::AngleAxisd(pitch_enu,     Eigen::Vector3d::UnitY()) *
//             Eigen::AngleAxisd(roll_enu,      Eigen::Vector3d::UnitX());
//         q_des.normalize();

//         // For debugging: extract yaw from the constructed quaternion.
//         double extracted_yaw = std::atan2(
//             2.0 * (q_des.x() * q_des.y() + q_des.w() * q_des.z()),
//             q_des.w()*q_des.w() + q_des.x()*q_des.x() - q_des.y()*q_des.y() - q_des.z()*q_des.z()
//         );

//         RCLCPP_INFO(this->get_logger(), "GPS heading (rad): %.6f, ENU yaw: %.6f, extracted_yaw: %.6f", 
//                     gps_heading_rad, yaw_enu, extracted_yaw);
//         RCLCPP_INFO(this->get_logger(), "Final Quaternion: [w=%.6f, x=%.6f, y=%.6f, z=%.6f]",
//                     q_des.w(), q_des.x(), q_des.y(), q_des.z());

//         // Set the final orientation (in ENU) in the odometry message.
//         odom_msg.pose.pose.orientation.w = q_des.w();
//         odom_msg.pose.pose.orientation.x = q_des.x();
//         odom_msg.pose.pose.orientation.y = q_des.y();
//         odom_msg.pose.pose.orientation.z = q_des.z();

//         // -----------------------------------
//         // Velocities
//         // -----------------------------------
//         // Assume GPS linear velocities (vx, vy, vz) are already in ENU.
//         odom_msg.twist.twist.linear.x = msg->vy;  // East
//         odom_msg.twist.twist.linear.y = msg->vx;  // North
//         odom_msg.twist.twist.linear.z = - msg->vz;  // Up

//         RCLCPP_INFO(this->get_logger(), "Odometry: [%.3f, %.3f, %.3f], velocities [%.3f, %.3f, %.3f], q[%.3f, %.3f, %.3f, %.3f]" ,
//                 odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z,
//                 odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y, odom_msg.twist.twist.linear.z,
//                 odom_msg.pose.pose.orientation.w, odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z);
    
//         // Angular velocities from IMU (assumed to be already in ENU)
//         odom_msg.twist.twist.angular.x = imu_angular_velocity_.x();
//         odom_msg.twist.twist.angular.y = imu_angular_velocity_.y();
//         odom_msg.twist.twist.angular.z = imu_angular_velocity_.z();

//         // -----------------------------------
//         // Covariance
//         // -----------------------------------
//         // If the fix_type is 3 (3D fix) or bigger than 3, then the covariance is valid which means covariance shuld be small.
//         // if (msg->gps_status >= 3){
//         //     for (double &c : odom_msg.pose.covariance) c = 0.0000001;
//         //     for (double &c : odom_msg.twist.covariance) c = 0.0000001;
//         // } else {
//         //     for (double &c : odom_msg.pose.covariance) c = 1e6;
//         //     for (double &c : odom_msg.twist.covariance) c = 1e6;
//         // }
        
//         // -----------------------------------
//         // Publish the odometry message in the ENU frame.
//         // -----------------------------------
//         // odom_pub_->publish(odom_msg);
//     }

//     void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
//         // Update IMU orientation and angular velocity
//         imu_orientation_ = Eigen::Quaterniond(
//             msg->orientation.w,
//             msg->orientation.x,
//             msg->orientation.y,
//             msg->orientation.z);

//         imu_angular_velocity_ = Eigen::Vector3d(
//             msg->angular_velocity.x,
//             msg->angular_velocity.y,
//             msg->angular_velocity.z);
//     }
// };

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomGenerator>());
    rclcpp::shutdown();
    return 0;
}