#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>

Eigen::Quaterniond deltaQ(const Eigen::Vector3d& theta);

double normalizeAngle(double angle);

Eigen::Matrix3d Rz(double yaw);

struct NavState {
    Eigen::Vector3d p{Eigen::Vector3d::Zero()};
    Eigen::Vector3d v{Eigen::Vector3d::Zero()};
    Eigen::Vector3d a{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};
    double yaw{0.0};

    Eigen::Vector3d Ba{Eigen::Vector3d::Zero()};
    Eigen::Vector3d Bg{Eigen::Vector3d::Zero()};

    nav_msgs::msg::Odometry toRosMsg(){
        nav_msgs::msg::Odometry msg;
        msg.pose.pose.position.x = p.x();
        msg.pose.pose.position.y = p.y();
        msg.pose.pose.position.z = p.z();
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();
        msg.pose.pose.orientation.w = q.w();
        msg.twist.twist.linear.x = v.x();
        msg.twist.twist.linear.y = v.y();
        msg.twist.twist.linear.z = v.z();
        msg.twist.twist.angular.x = a.x();
        msg.twist.twist.angular.y = a.y();
        msg.twist.twist.angular.z = a.z();
        return msg;
    }
};