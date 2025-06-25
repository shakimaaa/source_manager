#include <rclcpp/rclcpp.hpp>
#include "source_manager/source.hpp"
#include "source_manager/GPS.hpp"
#include "source_manager/slam.hpp"

class SourceManagerNode : public rclcpp::Node
{
public:
    SourceManagerNode();
    ~SourceManagerNode() = default;
private:

    void publish_odom();

    void timer_callback();

    void handle_state_change();

    Source::SharedPtr source_;
    GPS::SharedPtr gps_;
    SLAM::SharedPtr slam_;  

    rclcpp::timer::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr position_pub_;

};    