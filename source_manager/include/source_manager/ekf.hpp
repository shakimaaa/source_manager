#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>
#include <cmath>
#include <stdexcept>

// 常量定义
#define HUGE_NUMBER 1.0e10
#define INNOVATION_THRESHOLD 4.0 // 创新检验阈值

using namespace Eigen;
using namespace std;

// 状态和观测维度定义 (分解为三个独立的EKF)
constexpr int N_STATE_POS = 3;  // Px, Py, Pz
constexpr int N_STATE_VEL = 3;  // Vx, Vy, Vz
constexpr int N_STATE_ORI = 4;  // Qx, Qy, Qz, Qw

constexpr int M_OBS_POS = 3;
constexpr int M_OBS_VEL = 3;
constexpr int M_OBS_ORI = 4;

/**
 * @brief 包含新息检验结果的结构体。
 * @tparam N 观测维度
 */
template<int N>
struct InnovationData {
    Matrix<double, N, 1> innovation;          // 新息向量 (z - h(x_hat))
    Matrix<double, N, N> innovation_cov;  // 新息协方差矩阵 S
    Matrix<double, N, 1> test_ratios;         // 标准化新息的平方 (gamma)
    vector<bool> is_valid;                        // 标记每个观测维度是否通过检验
};

class SourceEKF {
public:
    // --- EKF 状态量 (分解为三个) ---
    Matrix<double, N_STATE_POS, 1> x_hat_pos;
    Matrix<double, N_STATE_VEL, 1> x_hat_vel;
    Matrix<double, N_STATE_ORI, 1> x_hat_ori;

    Matrix<double, N_STATE_POS, N_STATE_POS> P_pos;
    Matrix<double, N_STATE_VEL, N_STATE_VEL> P_vel;
    Matrix<double, N_STATE_ORI, N_STATE_ORI> P_ori;

    // --- EKF 噪声矩阵 (分解为三个) ---
    Matrix<double, N_STATE_POS, N_STATE_POS> Q_pos;
    Matrix<double, N_STATE_VEL, N_STATE_VEL> Q_vel;
    Matrix<double, N_STATE_ORI, N_STATE_ORI> Q_ori;

    Matrix<double, M_OBS_POS, M_OBS_POS> R_pos;
    Matrix<double, M_OBS_VEL, M_OBS_VEL> R_vel;
    Matrix<double, M_OBS_ORI, M_OBS_ORI> R_ori;

    SourceEKF();

    // 核心 EKF 步骤
    void predict(const Vector3d& accel, const Vector3d& gyro, double dt);
    bool update(const Matrix<double, M_OBS_POS + M_OBS_VEL + M_OBS_ORI, 1>& z);

private:
    // --- 独立预测函数 ---
    void predict_pos(const Vector3d& accel, double dt);
    void predict_vel(const Vector3d& accel, double dt);
    void predict_ori(const Vector3d& gyro, double dt);

    // --- 独立更新函数 ---
    bool update_pos(const Matrix<double, M_OBS_POS, 1>& z_pos);
    bool update_vel(const Matrix<double, M_OBS_VEL, 1>& z_vel);
    bool update_ori(const Matrix<double, M_OBS_ORI, 1>& z_ori);

    // --- 独立的新息检查 ---
    template<int N_STATE, int M_OBS>
    InnovationData<M_OBS> checkInnovation(
        const Matrix<double, N_STATE, 1>& x_hat,
        const Matrix<double, N_STATE, N_STATE>& P,
        const Matrix<double, M_OBS, 1>& z,
        const Matrix<double, M_OBS, M_OBS>& R) const;
};