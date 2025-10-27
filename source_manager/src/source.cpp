#include "source_manager/source.hpp"

SourceBase::SourceBase(std::shared_ptr<rclcpp::Node> node)
: node_(std::move(node))
{
    // if (!node_) {
    //     RCLCPP_WARN(rclcpp::get_logger("SourceBase"), "SourceBase constructed with null node");
    // }
}

void SourceBase::setImudata(const Eigen::Vector3d& linearAcceleration,
                            const Eigen::Vector3d& angularVelocity,
                            const Eigen::Quaterniond& orientation) {

    (void) linearAcceleration;
    (void) angularVelocity;
    (void) orientation;
}

void SourceBase::setHealthy(bool healthy) {
    (void) healthy;
}

void SourceBase::setCurrPose(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, const Eigen::Quaterniond& q) {
    (void) pos;
    (void) vel;
    (void) q;
}

bool SourceBase::isHealthy() {
    return true;
}

void SourceBase::setOffset(const Eigen::Vector3d& p, const Eigen::Quaterniond& q, double yaw) {
    (void)p;
    (void)q;
    (void)yaw;
}

ImuLite SourceBase::interpImu_(const ImuLite& a, const ImuLite& b, double t) {
    ImuLite s;
    s.t = t;
    double u = (t - a.t) / std::max(1e-9, (b.t - a.t));
    u = std::clamp(u, 0.0, 1.0);
    s.acc = (1.0 - u) * a.acc + u * b.acc;
    s.ori = a.ori.slerp(u, b.ori).normalized();
    return s;
}

bool SourceBase::extractImuInterval_(double t0, double t1, std::vector<ImuLite>& out) {
    if (t1 <= t0) return false;

    std::deque<ImuLite> buf;
    {
        std::lock_guard<std::mutex> lk(imu_mtx_);
        if (imu_buf_.empty()) return false;
        buf = imu_buf_;
    }

    // find L0
    ImuLite L0 = buf.front(), L1 = buf.front();
    bool okL = false;
    for (size_t i=1; i<buf.size(); ++i) {
        if (buf[i-1].t <= t0 && buf[i].t >= t0) { L0 = buf[i-1]; L1 = buf[i]; okL = true; break; }
    }
    if (!okL) return false;

    ImuLite R0 = buf.front(), R1 = buf.front();
    bool okR = false;
    for (size_t i=1; i<buf.size(); ++i) {
        if (buf[i-1].t <= t1 && buf[i].t >= t1) { R0 = buf[i-1]; R1 = buf[i]; okR = true; break; }
    }
    if (!okR) return false;

    out.clear();
    ImuLite L = interpImu_(L0, L1, t0);
    ImuLite R = interpImu_(R0, R1, t1);

    out.push_back(L);
    for (const auto& s : buf) if (s.t > t0 && s.t < t1) out.push_back(s);
    out.push_back(R);

    std::sort(out.begin(), out.end(), [](auto& a, auto& b){ return a.t < b.t; });
    
    out.erase(std::unique(out.begin(), out.end(),
                [](auto& a, auto& b){ return std::abs(a.t-b.t) < 1e-9; }), out.end()); // Remove duplicate elements
    return out.size() >= 2;

}

void SourceBase::integrateIntervalMidpoint_(const std::vector<ImuLite>& seg, Eigen::Vector3d& dvel_imu) {
    dvel_imu.setZero();
    for (size_t i=1; i<seg.size(); ++i) {
        double dt = seg[i].t - seg[i-1].t;
        if (dt <= 0.0 || dt > 0.1) continue; 
        Eigen::Vector3d acc_mid = 0.5 * (seg[i-1].acc + seg[i].acc);
        Eigen::Quaterniond q_mid = seg[i-1].ori.slerp(0.5, seg[i].ori).normalized();
        Eigen::Vector3d a_world = q_mid * acc_mid - g_;  // 用统一的 g_ 去重力
        if (a_world.norm() < 0.2) continue;              // 抑制零偏
        dvel_imu += a_world * dt;
    }
}