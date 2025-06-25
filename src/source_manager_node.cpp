#include "rclcpp/rclcpp.hpp"
#include "source_manager/source_manager_node.hpp"

SourceManagerNode::SourceManagerNode()
: Node("source_manager_node")
{
   odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
       "/odometry", 10, std::bind(&SourceManagerNode::odometry_callback, this, std::placeholders::_1));

   mavlink_sub_ = this->create_subscription<navx_msgs::msg::GlobalPositionInt>(
       "/mav/global_position_int", 10, std::bind(&SourceManagerNode::mavlink_callback, this, std::placeholders::_1));

   timer_ = this->create_wall_timer(
       std::chrono::seconds(1), std::bind(&SourceManagerNode::timer_callback, this));
    
   position_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/position", 10);


    this->declare_parameter("fix_type", 0);  // 默认值
  
}

void SourceManagerNode::timer_callback()
{
    RCLCPP_INFO(this->get_logger(), "Timer callback triggered");

    // double offset_x = 0.0;
    // double offset_y = 0.0;
    // double offset_z = 0.0;
    // double offset_yaw = 0.0;
    this->get_parameter("fix_type", fix_type_);
    current_state_ = get_state(fix_type_);
    RCLCPP_INFO(this->get_logger(), "Current state: %d", static_cast<int>(current_state_));
    if( current_state_ != previous_state_ && previous_state_ != State::UNINIT) 
    {
        RCLCPP_INFO(this->get_logger(), "State changed from %d to %d", static_cast<int>(previous_state_), static_cast<int>(current_state_));
        if( current_state_ == State::GPS)
        {
            previous_state_ = State::GPS;
            mode_changed_ = true;
        }else if( current_state_ == State::SLAM)
        {
            previous_state_ = State::SLAM;
            mode_changed_ = true;
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "Unknown previous state: %d", static_cast<int>(previous_state_));
            mode_changed_ = false;
        }
    }
    else {
        mode_changed_ = false;
    }
    if( current_state_ == State::SLAM) {
        if(mode_changed_){
            current_odometry_.pose.pose.position.x = current_odometry_.pose.pose.position.x + (current_odometry_.pose.pose.position.x - last_lat_);
            current_odometry_.pose.pose.position.y = current_odometry_.pose.pose.position.y + (current_odometry_.pose.pose.position.y - last_lon_);
            current_odometry_.pose.pose.position.z = current_odometry_.pose.pose.position.z + (current_odometry_.pose.pose.position.z - last_alt_);
            double yaw = calculate_yaw(current_odometry_);
            yaw = yaw + (yaw - last_slam_yaw_);
            current_odometry_.pose.pose.orientation.z = sin(yaw / 2.0);
            current_odometry_.pose.pose.orientation.w = cos(yaw / 2.0);
        }
        publish_position(State::SLAM);
        previous_state_ = State::SLAM;
        last_odometry_ = current_odometry_;
        last_slam_yaw_ = calculate_yaw(current_odometry_);
        last_slam_time_ = this->now();   
    }
    else if(current_state_ == State::GPS){
        if(mode_changed_) {
            d_lat = d_lat + (d_lat - last_odometry_.pose.pose.position.x);
            d_lon = d_lon + (d_lon - last_odometry_.pose.pose.position.y);
            d_alt = d_alt + (d_alt - last_odometry_.pose.pose.position.z);
            yaw_enu = yaw_enu + (yaw_enu - last_slam_yaw_);
        }
        publish_position(State::GPS);
        previous_state_ = State::GPS;
        last_lat_ = d_lat;
        last_lon_ = d_lon;
        last_alt_ = d_alt;
        last_gps_yaw_ = yaw_enu;
        last_gps_time_ = this->now();
    } 
}

void SourceManagerNode::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    if (!msg) {
        RCLCPP_ERROR(this->get_logger(), "Received null odometry message");
        return;
    } 
    if (!odometry_origin_set_) {
        origin_odometry_ = *msg;
        odometry_origin_set_ = true;
    }
    current_odometry_ = *msg;
    
}

void SourceManagerNode::mavlink_callback(const navx_msgs::msg::GlobalPositionInt::SharedPtr msg)
{
    if( !msg) {
        RCLCPP_ERROR(this->get_logger(), "Received null MAVLink message");
        return;
    }
    current_mavlink_position_ = *msg;
    if (!origin_set_) {
        origin_ = current_mavlink_position_;
        origin_set_ = true;
        RCLCPP_INFO(this->get_logger(), "Origin set to: lat=%f, lon=%f, alt=%f",
                    origin_.lat, origin_.lon, origin_.alt);
        return;
    }

    // fix_type_ = current_mavlink_position_.gps_status;

    // -----------------------------------
    // Compute relative position in meters.
    // We assume:
    //   - Latitude difference -> North displacement
    //   - Longitude difference -> East displacement (scaled by cos(latitude))
    //   - Altitude difference -> Up displacement
    // -----------------------------------
    d_lat = (current_mavlink_position_.lat - origin_.lat) * 111139.0;
    d_lon = (current_mavlink_position_.lon - origin_.lon) * 111139.0 * cos(origin_.lat * M_PI / 180.0);
    d_alt = current_mavlink_position_.alt - origin_.alt;

    double gps_heading_rad = current_mavlink_position_.hdg * M_PI / 180.0;
    yaw_enu = M_PI/2 - gps_heading_rad;
    // Normalize yaw to the range [-pi, pi].
    yaw_enu = std::atan2(std::sin(yaw_enu), std::cos(yaw_enu));

    
}

SourceManagerNode::State SourceManagerNode::get_state(int fix_type)
{
    switch (fix_type) {
        case 0:
        case 1:
        case 2:
            return State::SLAM;
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
        case 8:
            return State::GPS;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown fix type: %d", fix_type);
            return State::SLAM; // Default to SLAM if unknown
    }
}

double SourceManagerNode::calculate_yaw(const nav_msgs::msg::Odometry &position)
{
    // Calculate yaw from the orientation quaternion in the ENU frame.
    double siny_cosp = 2 * (position.pose.pose.orientation.w * position.pose.pose.orientation.z + 
                            position.pose.pose.orientation.x * position.pose.pose.orientation.y);
    double cosy_cosp = 1 - 2 * (position.pose.pose.orientation.y * position.pose.pose.orientation.y + 
                                position.pose.pose.orientation.z * position.pose.pose.orientation.z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    // Normalize yaw to the range [-pi, pi].
    return yaw;
}


void SourceManagerNode::publish_position(State state)
{
    nav_msgs::msg::Odometry position_msg;
    position_msg.header.stamp = this->now();
    position_msg.header.frame_id = "world";
    
    if (state == State::SLAM) {
        position_msg.pose.pose.position.x = current_odometry_.pose.pose.position.x;
        position_msg.pose.pose.position.y = current_odometry_.pose.pose.position.y;
        position_msg.pose.pose.position.z = current_odometry_.pose.pose.position.z;
        position_msg.pose.pose.orientation.x = current_odometry_.pose.pose.orientation.x;
        position_msg.pose.pose.orientation.y = current_odometry_.pose.pose.orientation.y;
        position_msg.pose.pose.orientation.z = current_odometry_.pose.pose.orientation.z;
        position_msg.pose.pose.orientation.w = current_odometry_.pose.pose.orientation.w;
        position_msg.twist.twist.linear.x = current_odometry_.twist.twist.linear.x;
        position_msg.twist.twist.linear.y = current_odometry_.twist.twist.linear.y;    
        position_msg.twist.twist.linear.z = current_odometry_.twist.twist.linear.z;
    } else if (state == State::GPS) {
        position_msg.pose.pose.position.x = d_lat;
        position_msg.pose.pose.position.y = d_lon;
        position_msg.pose.pose.position.z = d_alt;
        position_msg.pose.pose.orientation.x = 0.0; // Assuming no rotation for GPS
        position_msg.pose.pose.orientation.y = 0.0;
        position_msg.pose.pose.orientation.z = sin(yaw_enu / 2.0);
        position_msg.pose.pose.orientation.w = cos(yaw_enu / 2.0);
        position_msg.twist.twist.linear.x = 0.0; // Assuming no velocity for GPS
        position_msg.twist.twist.linear.y = 0.0;
        position_msg.twist.twist.linear.z = 0.0;
    }

    position_pub_->publish(position_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SourceManagerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}