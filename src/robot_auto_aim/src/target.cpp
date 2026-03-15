#include "robot_auto_aim/target.hpp"
#include <cmath>
#include <algorithm>

namespace robot_auto_aim {

Target::Target() 
    : ukf_(0.001, 2.0, 0.0),
      armor_num_(4), priority_(0), last_time_(0), last_armor_id_(0), update_count_(0),
      q_x_(0.001), q_y_(0.001), q_z_(0.001), 
      q_vx_(0.1), q_vy_(0.01), q_vz_(0.1),
      q_yaw_(0.01), q_v_yaw_(0.001), q_geo_(0.0001),
      r_x_(0.5), r_y_(0.5), r_z_(0.5), r_yaw_(0.05), r_yaw_adaptive_factor_(50.0),
      dist_scale_coeff_(0.1), z_scale_coeff_(5.0),
      adaptive_tracking_(false), r_alpha_(0.1) { 
}

void Target::init(const TrackerArmor& armor) {
    name_ = armor.number;
    type_ = armor.type;
    priority_ = armor.priority;
    
    double r = 0.2; 
    double yaw = armor.yaw;
    
    double cx = armor.position.x() + r * std::cos(yaw);
    double cy = armor.position.y() + r * std::sin(yaw);
    double cz = armor.position.z();
    
    Eigen::Matrix<double, STATE_DIM, 1> x0;
    x0 << cx, 0, cy, 0, cz, 0, yaw, M_PI, r, 0, 0;
    
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> P0 = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Identity();
    P0.block<6, 6>(0, 0) *= 0.1;
    P0(1, 1) = 10.0; P0(3, 3) = 10.0; P0(5, 5) = 10.0;
    P0(6, 6) = 0.5;
    P0(7, 7) = 100.0;
    P0(8, 8) = 0.1; P0(9, 9) = 0.1; P0(10, 10) = 0.1;
    
    ukf_.init(x0, P0);
    
    R_adaptive_ = Eigen::Matrix4d::Identity();
    R_adaptive_(0, 0) = r_x_;
    R_adaptive_(1, 1) = r_y_;
    R_adaptive_(2, 2) = r_z_;
    R_adaptive_(3, 3) = r_yaw_;
    
    last_time_ = armor.timestamp;
    update_count_ = 1;
}

void Target::predict(const rclcpp::Time& time) {
    double dt = (time - last_time_).seconds();
    if (dt <= 0) return;
    
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> Q = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Zero();
    Q(0,0) = q_x_; Q(2,2) = q_y_; Q(4,4) = q_z_;
    Q(1,1) = q_vx_; Q(3,3) = q_vy_; Q(5,5) = q_vz_;
    Q(6,6) = q_yaw_;
    Q(7,7) = q_v_yaw_;
    Q(8,8) = Q(9,9) = Q(10,10) = q_geo_;
    
    auto f = [dt](const Eigen::Matrix<double, STATE_DIM, 1>& x) {
        Eigen::Matrix<double, STATE_DIM, 1> x_out = x;
        x_out(0) += x(1) * dt; 
        x_out(2) += x(3) * dt; 
        x_out(4) += x(5) * dt; 
        x_out(6) += x(7) * dt; 
        return x_out;
    };
    
    auto normalize_yaw = [](Eigen::Matrix<double, STATE_DIM, 1>& x) {
        x(6) = std::atan2(std::sin(x(6)), std::cos(x(6)));
    };
    
    ukf_.predict(f, Q, normalize_yaw);
    last_time_ = time;
}

bool Target::update(const TrackerArmor& armor) {
    const auto x = ukf_.getState();
    double current_yaw = x(6);
    
    int best_id = 0;
    double min_angle_err = 1e10;
    for (int i = 0; i < 4; ++i) {
        double armor_angle = current_yaw + i * M_PI / 2.0;
        double angle_err = std::abs(std::atan2(std::sin(armor.yaw - armor_angle), std::cos(armor.yaw - armor_angle)));
        if (angle_err < min_angle_err) {
            min_angle_err = angle_err;
            best_id = i;
        }
    }
    
    Eigen::Vector4d z;
    z << armor.position.x(), armor.position.y(), armor.position.z(), armor.yaw;
    
    // --- Dynamic R Calculation ---
    // 1. Base noises in camera frame (Z-axis is depth, noise is higher)
    // Distance scaling factor
    double dist = armor.position.norm();
    double dist_scale = 1.0 + dist_scale_coeff_ * dist * dist;
    
    // R in Camera Frame: X_c, Y_c, Z_c
    // Typically, lateral noise (X, Y) < depth noise (Z)
    Eigen::Matrix3d R_pos_cam = Eigen::Matrix3d::Zero();
    R_pos_cam(0, 0) = r_x_ * dist_scale;
    R_pos_cam(1, 1) = r_y_ * dist_scale;
    R_pos_cam(2, 2) = r_z_ * z_scale_coeff_ * dist_scale; // Z noise is significantly larger
    
    // 2. Rotate camera-frame noise to world frame (Odom)
    Eigen::Matrix3d rot = armor.orientation.toRotationMatrix();
    Eigen::Matrix3d R_pos_world = rot * R_pos_cam * rot.transpose();
    
    Eigen::Matrix4d R = Eigen::Matrix4d::Zero();
    R.block<3, 3>(0, 0) = R_pos_world;
    R(3, 3) = (best_id == last_armor_id_) ? r_yaw_ : r_yaw_ * r_yaw_adaptive_factor_;
    // -----------------------------

    auto h_func = [this, best_id](const Eigen::Matrix<double, STATE_DIM, 1>& x_in) {
        return this->h(x_in, best_id);
    };
    auto normalize_meas = [](Eigen::Vector4d& z_diff) {
        z_diff(3) = std::atan2(std::sin(z_diff(3)), std::cos(z_diff(3)));
    };
    auto normalize_state = [](Eigen::Matrix<double, STATE_DIM, 1>& x_diff) {
        x_diff(6) = std::atan2(std::sin(x_diff(6)), std::cos(x_diff(6)));
    };
    
    bool success = false;
    if (adaptive_tracking_) {
        // Sage-Husa will use our geometric R as prior
        success = ukf_.updateAdaptive<MEAS_DIM>(z, h_func, R_adaptive_, r_alpha_, normalize_meas, normalize_state, 15.0);
    } else {
        success = ukf_.update<MEAS_DIM>(z, h_func, R, normalize_meas, normalize_state, 15.0);
    }
    
    if (success) {
        update_count_++;
        last_armor_id_ = best_id;
        return true;
    }
    return false;
}

bool Target::isConverged() const {
    const auto& cov = ukf_.getCovariance().diagonal();
    // 速度方差由于过程噪声的注入（如 q_vx = 0.1），其稳态值不可能低于过程噪声。
    // 因此我们只检查位置（x:0, y:2, z:4）和角度（yaw:6）的方差是否收敛到一个合理的范围。
    return update_count_ > 10 && 
           cov(0) < 0.5 && cov(2) < 0.5 && cov(4) < 0.5 && 
           cov(6) < 0.2;
}

bool Target::isDiverged() const {
    const auto& x = ukf_.getState();
    const auto& cov = ukf_.getCovariance().diagonal();
    if (cov.head<3>().maxCoeff() > 100.0) return true;
    if (x.head<3>().norm() > 40.0) return true;
    if (x(8) < 0.05 || x(8) > 0.6) return true; 
    return false;
}

Eigen::Vector4d Target::h(const Eigen::VectorXd& x, int id) const {
    double yaw = x(6);
    double r = x(8);
    double l = x(9);
    double h_val = x(10);
    double angle = yaw + id * M_PI / 2.0;
    double current_r = (id % 2 == 0) ? r : r + l;
    double current_z = (id % 2 == 0) ? x(4) : x(4) + h_val;
    double ax = x(0) - current_r * std::cos(angle);
    double ay = x(2) - current_r * std::sin(angle);
    return Eigen::Vector4d(ax, ay, current_z, angle);
}

std::vector<Eigen::Vector4d> Target::getResolvedArmors() const {
    std::vector<Eigen::Vector4d> armors;
    const auto x = ukf_.getState();
    for (int i = 0; i < 4; ++i) {
        armors.push_back(h(x, i));
    }
    return armors;
}

Eigen::VectorXd Target::getPredictedState(const rclcpp::Time& time) const {
    double dt = (time - last_time_).seconds();
    auto x = ukf_.getState();
    if (dt > 0) {
        x(0) += x(1) * dt; 
        x(2) += x(3) * dt; 
        x(4) += x(5) * dt; 
        x(6) += x(7) * dt; 
        x(6) = std::atan2(std::sin(x(6)), std::cos(x(6)));
    }
    return x;
}

} // namespace robot_auto_aim
