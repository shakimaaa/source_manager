#include "source_manager/source.hpp"
#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "xion_msg/msg/extended_odometry.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>  
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>

class SLAM : public SourceBase
{
public:
    explicit SLAM(rclcpp::Node::SharedPtr node, std::shared_ptr<BaseData> data);
    void setImudata(const Eigen::Vector3d& linearAcceleration,
                    const Eigen::Vector3d& angularVelocity,
                    const Eigen::Quaterniond& orientation) override;
    void setHealthy(bool healthy) override;
    void setCurrPose(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, const Eigen::Quaterniond& q) override;
    bool isHealthy() override;
    void updateData() override;

    void setSlamdata();
    void setOdometry();

private:
    rclcpp::CallbackGroup::SharedPtr slam_callback_group_;
    rclcpp::Subscription<xion_msg::msg::ExtendedOdometry>::SharedPtr slam_sub_;
    rclcpp::TimerBase::SharedPtr slam_timer_;

    Eigen::Vector3d imu_acc_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d imu_gyro_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond imu_orientation_ = Eigen::Quaterniond::Identity();

    // Just received slam data
    Eigen::Vector3d r_slam_p_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d r_slam_v_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d r_slam_a_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond r_slam_q_ = Eigen::Quaterniond::Identity();
    double r_slam_yaw_ = 0.0;

    // latest slam data
    Eigen::Vector3d latest_slam_p_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d latest_slam_v_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d latest_slam_a_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond latest_slam_q_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d latest_Ba_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d latest_Bg_ = Eigen::Vector3d::Zero();
    double latest_slam_yaw_ = 0.0;

    // offset
    Eigen::Vector3d slam_offset_p_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond slam_offset_q_ = Eigen::Quaterniond::Identity();
    double slam_offset_yaw_ = 0.0;

    // imu propagate
    Eigen::Vector3d latest_slam_acc_0 = Eigen::Vector3d::Zero();
    Eigen::Vector3d latest_slam_gyr_0 = Eigen::Vector3d::Zero();
    Eigen::Vector3d g_ = Eigen::Vector3d(0, 0, 9.81); // Gravity constant

    // 积分
    Eigen::Vector3d integrated_v_imu_ = Eigen::Vector3d::Zero();

    bool SLAM_healthy_ = false;
    bool receiving_slam_ = false;

    rclcpp::Time last_get_slam_time_;
    rclcpp::Time last_slam_time_;
    rclcpp::Time last_propagate_time_;


    void slamCallback(const xion_msg::msg::ExtendedOdometry::SharedPtr msg);
    void timerCallback();
};