#include "source_manager/ekf.hpp"
#include <algorithm>


// 构造函数
SourceEKF::SourceEKF() {
    // --- 初始化位置 EKF ---
    x_hat_pos.setZero();
    P_pos.setIdentity(); P_pos *= 0.1;
    Q_pos.setIdentity(); Q_pos *= 0.1;
    R_pos.setIdentity(); R_pos *= 0.005;

    // --- 初始化速度EKF ---
    x_hat_vel.setZero();
    P_vel.setIdentity(); P_vel *= 0.1;
    Q_vel.setIdentity(); Q_vel *= 0.1;
    R_vel.setIdentity(); R_vel *= 0.002;

    // --- 初始化姿态EKF ---
    x_hat_ori.setZero();
    x_hat_ori(3) = 1.0; // (x,y,z,w) -> w=1
    P_ori.setIdentity(); P_ori *= 0.1;
    Q_ori.setIdentity(); Q_ori *= 0.1;
    R_ori.setIdentity(); R_ori *= 0.0005;

    cout << "SourceEKF initialized.\n";
}

// -------------------------------------------------------------------
// 核心 EKF 步骤 (预测和更新)
// -------------------------------------------------------------------

// 预测步骤 (Time Update) - 主函数：分别预测位置、速度和姿态
void SourceEKF::predict(const Vector3d& accel, const Vector3d& gyro, double dt) {
    predict_pos(accel, dt);
    predict_vel(accel, dt);
    predict_ori(gyro, dt);
}

// 更新步骤 (Measurement Update) - 主函数：分别更新位置、速度和姿态
bool SourceEKF::update(const Matrix<double, M_OBS_POS + M_OBS_VEL + M_OBS_ORI, 1>& z) {
    bool success = true;
    success &= update_pos(z.block<M_OBS_POS, 1>(0, 0));
    success &= update_vel(z.block<M_OBS_VEL, 1>(3, 0));
    success &= update_ori(z.block<M_OBS_ORI, 1>(6, 0));
    return success;
}

// --- 独立预测实现：位置 ---
void SourceEKF::predict_pos(const Vector3d& accel, double dt) {
    // 状态预测: p_k = p_{k-1} + v_{k-1}*dt + 0.5*a*dt^2
    x_hat_pos = x_hat_pos + x_hat_vel * dt + 0.5 * accel * dt * dt;

    // 状态转移矩阵 F (位置部分)
    Matrix<double, N_STATE_POS, N_STATE_POS> F_pos = Matrix<double, N_STATE_POS, N_STATE_POS>::Identity();
    
    // 协方差预测: P_k = F*P_{k-1}*F^T + Q
    P_pos = F_pos * P_pos * F_pos.transpose() + Q_pos;
}

// --- 独立预测实现：速度 ---
void SourceEKF::predict_vel(const Vector3d& accel, double dt) {
    // 状态预测: v_k = v_{k-1} + a*dt
    x_hat_vel = x_hat_vel + accel * dt;

    // 状态转移矩阵 F (速度部分)
    Matrix<double, N_STATE_VEL, N_STATE_VEL> F_vel = Matrix<double, N_STATE_VEL, N_STATE_VEL>::Identity();

    // 协方差预测: P_k = F*P_{k-1}*F^T + Q
    P_vel = F_vel * P_vel * F_vel.transpose() + Q_vel;
}

// --- 独立预测实现：姿态 ---
void SourceEKF::predict_ori(const Vector3d& gyro, double dt) {
    // 状态预测: q_k = q_{k-1} * delta_q(gyro*dt)
    Quaterniond q_prev(x_hat_ori(3), x_hat_ori(0), x_hat_ori(1), x_hat_ori(2)); // w, x, y, z
    Vector3d omega = gyro;
    Quaterniond omega_q(0, omega.x(), omega.y(), omega.z());
    Quaterniond q_dot = q_prev * omega_q;
    q_dot.coeffs() *= 0.5;

    Quaterniond q_new(q_prev.w() + q_dot.w() * dt,
                      q_prev.x() + q_dot.x() * dt,
                      q_prev.y() + q_dot.y() * dt,
                      q_prev.z() + q_dot.z() * dt);
    q_new.normalize();

    x_hat_ori << q_new.x(), q_new.y(), q_new.z(), q_new.w();

    // 状态转移矩阵 F (姿态部分)
    Matrix<double, N_STATE_ORI, N_STATE_ORI> F_ori;
    double wx = gyro.x(), wy = gyro.y(), wz = gyro.z();
    F_ori << 1.0, 0.5*wz*dt, -0.5*wy*dt, 0.5*wx*dt,
            -0.5*wz*dt, 1.0, 0.5*wx*dt, 0.5*wy*dt,
             0.5*wy*dt, -0.5*wx*dt, 1.0, 0.5*wz*dt,
            -0.5*wx*dt, -0.5*wy*dt, -0.5*wz*dt, 1.0;

    // 协方差预测: P_k = F*P_{k-1}*F^T + Q
    P_ori = F_ori * P_ori * F_ori.transpose() + Q_ori;
}

// --- 独立更新实现：位置 ---
bool SourceEKF::update_pos(const Matrix<double, M_OBS_POS, 1>& z_pos) {
    // 检查新息，判断观测是否有效
    InnovationData<M_OBS_POS> innov_data = checkInnovation<N_STATE_POS, M_OBS_POS>(x_hat_pos, P_pos, z_pos, R_pos);
    const bool all_valid = std::all_of(innov_data.is_valid.begin(), innov_data.is_valid.end(),
                                       [](bool v){ return v; });
    Matrix<double, M_OBS_POS, M_OBS_POS> R_prime = R_pos;
    // 如果观测无效，增大测量噪声协方差 R，使其对状态更新影响极小
    for (int i = 0; i < M_OBS_POS; ++i) {
        if (!innov_data.is_valid[i]) R_prime(i, i) *= HUGE_NUMBER;
    }

    Matrix<double, M_OBS_POS, N_STATE_POS> H = Matrix<double, M_OBS_POS, N_STATE_POS>::Identity();
    Matrix<double, M_OBS_POS, M_OBS_POS> S = H * P_pos * H.transpose() + R_prime;
    Matrix<double, N_STATE_POS, M_OBS_POS> K = P_pos * H.transpose() * S.inverse();

    x_hat_pos = x_hat_pos + K * innov_data.innovation;

    Matrix<double, N_STATE_POS, N_STATE_POS> I = Matrix<double, N_STATE_POS, N_STATE_POS>::Identity();
    Matrix<double, N_STATE_POS, N_STATE_POS> temp = I - K * H;
    P_pos = temp * P_pos * temp.transpose() + K * R_prime * K.transpose();
    P_pos = (P_pos + P_pos.transpose()) / 2.0;
    return all_valid;
}

// --- 独立更新实现：速度 ---
bool SourceEKF::update_vel(const Matrix<double, M_OBS_VEL, 1>& z_vel) {
    InnovationData<M_OBS_VEL> innov_data = checkInnovation<N_STATE_VEL, M_OBS_VEL>(x_hat_vel, P_vel, z_vel, R_vel);
    const bool all_valid = std::all_of(innov_data.is_valid.begin(), innov_data.is_valid.end(),
                                       [](bool v){ return v; });
    Matrix<double, M_OBS_VEL, M_OBS_VEL> R_prime = R_vel;
    // 如果观测无效，增大测量噪声协方差 R
    for (int i = 0; i < M_OBS_VEL; ++i) {
        if (!innov_data.is_valid[i]) R_prime(i, i) *= HUGE_NUMBER;
    }

    Matrix<double, M_OBS_VEL, N_STATE_VEL> H = Matrix<double, M_OBS_VEL, N_STATE_VEL>::Identity();
    Matrix<double, M_OBS_VEL, M_OBS_VEL> S = H * P_vel * H.transpose() + R_prime;
    Matrix<double, N_STATE_VEL, M_OBS_VEL> K = P_vel * H.transpose() * S.inverse();

    x_hat_vel = x_hat_vel + K * innov_data.innovation;

    Matrix<double, N_STATE_VEL, N_STATE_VEL> I = Matrix<double, N_STATE_VEL, N_STATE_VEL>::Identity();
    Matrix<double, N_STATE_VEL, N_STATE_VEL> temp = I - K * H;
    P_vel = temp * P_vel * temp.transpose() + K * R_prime * K.transpose();
    P_vel = (P_vel + P_vel.transpose()) / 2.0;
    return all_valid;
}

// --- 独立更新实现：姿态 ---
bool SourceEKF::update_ori(const Matrix<double, M_OBS_ORI, 1>& z_ori) {
    // 将测量四元数对齐到同一半球，避免因四元数双倍覆盖特性导致的虚假大新息
    Matrix<double, M_OBS_ORI, 1> z_adjusted = z_ori;
    Quaterniond meas_q(z_ori(3), z_ori(0), z_ori(1), z_ori(2));
    Quaterniond state_q(x_hat_ori(3), x_hat_ori(0), x_hat_ori(1), x_hat_ori(2));
    if (meas_q.dot(state_q) < 0.0) {
        meas_q.coeffs() *= -1.0;
        z_adjusted << meas_q.x(), meas_q.y(), meas_q.z(), meas_q.w();
    }

    InnovationData<M_OBS_ORI> innov_data = checkInnovation<N_STATE_ORI, M_OBS_ORI>(x_hat_ori, P_ori, z_adjusted, R_ori);
    const bool all_valid = std::all_of(innov_data.is_valid.begin(), innov_data.is_valid.end(),
                                       [](bool v){ return v; });
    Matrix<double, M_OBS_ORI, M_OBS_ORI> R_prime = R_ori;
    // 如果观测无效，增大测量噪声协方差 R
    for (int i = 0; i < M_OBS_ORI; ++i) {
        if (!innov_data.is_valid[i]) R_prime(i, i) *= HUGE_NUMBER;
    }

    Matrix<double, M_OBS_ORI, N_STATE_ORI> H = Matrix<double, M_OBS_ORI, N_STATE_ORI>::Identity();
    Matrix<double, M_OBS_ORI, M_OBS_ORI> S = H * P_ori * H.transpose() + R_prime;
    Matrix<double, N_STATE_ORI, M_OBS_ORI> K = P_ori * H.transpose() * S.inverse();

    x_hat_ori = x_hat_ori + K * innov_data.innovation;
    // 归一化四元数
    x_hat_ori.normalize();

    Matrix<double, N_STATE_ORI, N_STATE_ORI> I = Matrix<double, N_STATE_ORI, N_STATE_ORI>::Identity();
    Matrix<double, N_STATE_ORI, N_STATE_ORI> temp = I - K * H;
    P_ori = temp * P_ori * temp.transpose() + K * R_prime * K.transpose();
    P_ori = (P_ori + P_ori.transpose()) / 2.0;
    return all_valid;
}

// --- 通用新息检查模板函数 ---
template<int N_STATE, int M_OBS>
InnovationData<M_OBS> SourceEKF::checkInnovation(
    const Matrix<double, N_STATE, 1>& x_hat,
    const Matrix<double, N_STATE, N_STATE>& P,
    const Matrix<double, M_OBS, 1>& z,
    const Matrix<double, M_OBS, M_OBS>& R) const {

    InnovationData<M_OBS> data;
    
    // 1. 预测观测值 (h(x) = x)
    Matrix<double, M_OBS, 1> z_hat = x_hat.template block<M_OBS, 1>(0, 0);
    
    // 2. 计算新息
    data.innovation = z - z_hat;

    // 3. 计算观测雅可比 H (h(x) = x, so H is Identity)
    Matrix<double, M_OBS, N_STATE> H = Matrix<double, M_OBS, N_STATE>::Identity();

    // 4. 计算新息协方差 S
    data.innovation_cov = H * P * H.transpose() + R;

    // 5. 计算检验比 (Test Ratios) - 卡方检验
    data.test_ratios.setZero();
    data.is_valid.assign(M_OBS, false);
    for (int i = 0; i < M_OBS; ++i) {
        if (data.innovation_cov(i, i) > 1.0e-9) {
            data.test_ratios(i) = data.innovation(i) * data.innovation(i) / data.innovation_cov(i, i);
            data.is_valid[i] = data.test_ratios(i) <= INNOVATION_THRESHOLD;
        }
    }

    return data;
}
