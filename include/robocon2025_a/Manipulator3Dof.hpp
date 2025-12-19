#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <tuple>

class Manipulator3DoF
{
private:
    float EPS = 1e-6;

    const float l1_;
    const float l2_;
    const float Eps_r_; // theta1の軸の半径Eps_r_円上にtheta2の軸がある

    float theta_[3];
    Eigen::Vector3d dtheta_;

public:
    Manipulator3DoF(float l1, float l2, float Eps_r) : l1_(l1), l2_(l2), Eps_r_(Eps_r) {}

    // 目標点 (x, y, z), リンク長 l1, l2, 回転中心半径 eps_r を指定して θ1, θ2, θ3を求める
    void inverseKinematics(float x, float y, float z)
    {
        // Step 1: θ1 の計算（第一関節から見た方向）
        theta_[0] = atan2(y, x);

        // Step 2: 第2関節の基準位置を求める（Eps_r円周上）
        float base_x = Eps_r_ * cos(theta_[0]);
        float base_y = Eps_r_ * sin(theta_[0]);

        // Step 3: エンドエフェクタまでの相対ベクトル（baseからの差）
        float dx = x - base_x;
        float dy = y - base_y;
        float dz = z;

        // Step 4: 第2,3関節の平面問題に落とし込む
        float r = std::sqrt(dx * dx + dy * dy);
        float d = std::sqrt(r * r + dz * dz);

        // 範囲チェック
        if (d > l1_ + l2_ || d < std::fabs(l1_ - l2_))
            throw std::runtime_error("Target out of reach");

        // Step 5: θ3（肘角）を求める
        float cos_theta3 = (r * r + dz * dz - l1_ * l1_ - l2_ * l2_) / (2 * l1_ * l2_);
        if (cos_theta3 < -1.0 || cos_theta3 > 1.0)
            throw std::runtime_error("acos out of range");

        theta_[2] = std::acos(cos_theta3); // 肘下げ解

        // Step 6: θ2 を求める（傾き - 内部角）
        float alpha = std::atan2(dz, r);
        float beta = std::atan2(l2_ * sin(theta_[2]), l1_ + l2_ * cos(theta_[2]));
        theta_[1] = alpha - beta;
    }

    void computeJointVelocities_ZYY(double theta1, double theta2, double theta3, double dx, double dy, double dz)
    {
        double t23 = theta2 + theta3;

        double c1 = cos(theta1), s1 = sin(theta1);
        double c2 = cos(theta2), s2 = sin(theta2);
        double c23 = cos(t23), s23 = sin(t23);

        double R = Eps_r_ + l1_ * c2 + l2_ * c23;
        double dR_dtheta2 = -l1_ * s2 - l2_ * s23;
        double dR_dtheta3 = -l2_ * s23;

        // ヤコビ行列 J（3x3）
        Eigen::Matrix3d J;

        // ∂x/∂θi
        J(0, 0) = -s1 * R;
        J(0, 1) = c1 * dR_dtheta2;
        J(0, 2) = c1 * dR_dtheta3;

        // ∂y/∂θi
        J(1, 0) = c1 * R;
        J(1, 1) = s1 * dR_dtheta2;
        J(1, 2) = s1 * dR_dtheta3;

        // ∂z/∂θi
        J(2, 0) = 0;
        J(2, 1) = l1_ * c2 + l2_ * c23;
        J(2, 2) = l2_ * c23;

        // 速度ベクトル
        Eigen::Vector3d v(dx, dy, dz);

        // 最小ノルム解（擬似逆行列 or LU分解）
        dtheta_ = J.fullPivLu().solve(v);
    }

    float getdtheta(int theta)
    {
        return dtheta_(theta);
    }
};