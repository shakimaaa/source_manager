#include "source_manager/GPS.h"



void GPS::gps_callback(const navx_msgs::msg::GlobalPositionInt::SharedPtr msg)
{
    current_mavlink_position_ = *msg;

    if(!origin_set_) {
        origin_ = *msg;
        origin_set_ = true;

        return;
    }
    
    fix_type_ = current_mavlink_position_.fix_type;
    check_state_health();

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

void GPS::set_offset(double lastX, double lastY, double lastZ, double lastYAW)
{
    offset_x = lastX - d_lon;
    offset_y = lastY - d_lat;
    offset_z = lastZ - d_alt;
    offset_yaw = lastYAW - yaw_enu;

}

void GPS::check_state_health()
{
    switch (fix_type_)
    {
        case 0:
        case 1:
        case 2:
            gps_available = false;
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
        case 8:
            gps_available = true;
        default:
            gps_available = false;
            break;
    }
}