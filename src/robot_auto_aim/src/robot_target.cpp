#include "robot_auto_aim/robot_target.hpp"
#include <cmath>
#include <algorithm>

namespace robot_auto_aim {

RobotTarget::RobotTarget() 
    : armor_num_(4), priority_(0), last_time_(0), last_armor_id_(0), update_count_(0),
      best_ukf_idx_(0), confirmation_state_(ConfirmationState::CONFIRMED),
      q_x_(0.001), q_y_(0.001), q_z_(0.001), 
      q_vx_(0.1), q_vy_(0.01), q_vz_(0.1),
      q_yaw_(0.01), q_v_yaw_(0.001), q_geo_(0.0001),
      r_x_(0.5), r_y_(0.5), r_z_(0.5), r_yaw_(0.05), r_yaw_adaptive_factor_(50.0),
      dist_scale_coeff_(0.1), z_scale_coeff_(5.0),
      min_update_count_(5), max_pos_cov_(3.0), max_yaw_cov_(1.0),
      adaptive_tracking_(false), q_alpha_(0.1) { 
    for (int i = 0; i < 2; ++i) {
        ukfs_[i] = robot_utils::UKF<STATE_DIM>(0.001, 2.0, 0.0);
        accumulated_errors_[i] = 0.0;
    }
}

void RobotTarget::init(const TrackerArmor& armor, const GeometricParams& init_geo) {
    name_ = armor.number;
    type_ = armor.type;
    priority_ = armor.priority;
    last_time_ = armor.timestamp;
    update_count_ = 1;
    armor_switch_count_ = 0;
    last_armor_id_ = 0;

    Eigen::Matrix<double, STATE_DIM, STATE_DIM> P0 = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Identity();
    P0.block<6, 6>(0, 0) *= 0.1;
    P0(1, 1) = 10.0; P0(3, 3) = 10.0; P0(5, 5) = 10.0;
    P0(6, 6) = 0.5;
    P0(7, 7) = 100.0;
    P0(8, 8) = 0.1; P0(9, 9) = 0.1; P0(10, 10) = 0.1;

    if (init_geo.r > 1e-3) {
        confirmation_state_ = ConfirmationState::CONFIRMING;
        best_ukf_idx_ = 0;

        // Hypothesis 0: Current armor is ID 0 or 2 (Even side)
        double r0 = init_geo.r;
        double l0 = init_geo.l;
        double h0 = init_geo.h;
        double cx0 = armor.position.x() + r0 * std::cos(armor.yaw);
        double cy0 = armor.position.y() + r0 * std::sin(armor.yaw);
        Eigen::Matrix<double, STATE_DIM, 1> x0;
        x0 << cx0, 0, cy0, 0, armor.position.z(), 0, armor.yaw, 0.0, r0, l0, h0;
        ukfs_[0].init(x0, P0);
        accumulated_errors_[0] = 0.0;

        // Hypothesis 1: Current armor is ID 1 or 3 (Odd side)
        // Rotate 90 degrees: r_new = r_old + l_old, l_new = -l_old, h_new = -h_old
        double r1 = init_geo.r + init_geo.l;
        double l1 = -init_geo.l;
        double h1 = -init_geo.h;
        double cx1 = armor.position.x() + r1 * std::cos(armor.yaw);
        double cy1 = armor.position.y() + r1 * std::sin(armor.yaw);
        Eigen::Matrix<double, STATE_DIM, 1> x1;
        x1 << cx1, 0, cy1, 0, armor.position.z() - h1, 0, armor.yaw, 0.0, r1, l1, h1;
        ukfs_[1].init(x1, P0);
        accumulated_errors_[1] = 0.0;

    } else {
        confirmation_state_ = ConfirmationState::CONFIRMED;
        best_ukf_idx_ = 0;
        double r = 0.2; 
        double cx = armor.position.x() + r * std::cos(armor.yaw);
        double cy = armor.position.y() + r * std::sin(armor.yaw);
        Eigen::Matrix<double, STATE_DIM, 1> x0;
        x0 << cx, 0, cy, 0, armor.position.z(), 0, armor.yaw, 0.0, r, 0, 0;
        ukfs_[0].init(x0, P0);
    }
    
    Q_adaptive_ = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Zero();
    Q_adaptive_(0,0) = q_x_; Q_adaptive_(2,2) = q_y_; Q_adaptive_(4,4) = q_z_;
    Q_adaptive_(1,1) = q_vx_; Q_adaptive_(3,3) = q_vy_; Q_adaptive_(5,5) = q_vz_;
    Q_adaptive_(6,6) = q_yaw_;
    Q_adaptive_(7,7) = q_v_yaw_;
    Q_adaptive_(8,8) = Q_adaptive_(9,9) = Q_adaptive_(10,10) = q_geo_;
}

void RobotTarget::predict(const rclcpp::Time& time) {
    double dt = (time - last_time_).seconds();
    if (dt <= 0) return;
    
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> Q = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Zero();
    Q(0,0) = q_x_ * dt; Q(2,2) = q_y_ * dt; Q(4,4) = q_z_ * dt;
    Q(1,1) = q_vx_ * dt; Q(3,3) = q_vy_ * dt; Q(5,5) = q_vz_ * dt;
    Q(6,6) = q_yaw_ * dt;
    Q(7,7) = q_v_yaw_ * dt;
    Q(8,8) = Q(9,9) = Q(10,10) = q_geo_ * dt;
    
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
    
    if (confirmation_state_ == ConfirmationState::CONFIRMING) {
        for (int i = 0; i < 2; ++i) {
            ukfs_[i].predict(f, adaptive_tracking_ ? Q_adaptive_ : Q, normalize_yaw);
        }
    } else {
        ukfs_[best_ukf_idx_].predict(f, adaptive_tracking_ ? Q_adaptive_ : Q, normalize_yaw);
    }
    last_time_ = time;
}

bool RobotTarget::update(const TrackerArmor& armor) {
    Eigen::Vector4d z;
    z << armor.position.x(), armor.position.y(), armor.position.z(), armor.yaw;
    
    double dist = armor.position.norm();
    double dist_scale = 1.0 + dist_scale_coeff_ * dist * dist;
    Eigen::Matrix3d R_pos_cam = Eigen::Matrix3d::Zero();
    R_pos_cam(0, 0) = r_x_ * dist_scale;
    R_pos_cam(1, 1) = r_y_ * dist_scale;
    R_pos_cam(2, 2) = r_z_ * z_scale_coeff_ * dist_scale;
    Eigen::Matrix3d rot = armor.orientation.toRotationMatrix();
    Eigen::Matrix3d R_pos_world = rot * R_pos_cam * rot.transpose();
    
    auto normalize_meas = [](Eigen::Vector4d& z_diff) {
        z_diff(3) = std::atan2(std::sin(z_diff(3)), std::cos(z_diff(3)));
    };
    auto normalize_state = [](Eigen::Matrix<double, STATE_DIM, 1>& x_diff) {
        x_diff(6) = std::atan2(std::sin(x_diff(6)), std::cos(x_diff(6)));
    };

    bool any_success = false;

    if (confirmation_state_ == ConfirmationState::CONFIRMING) {
        int current_best_id = 0;
        for (int k = 0; k < 2; ++k) {
            const auto x = ukfs_[k].getState();
            int best_id = 0;
            double min_combined_err = 1e10;
            for (int i = 0; i < 4; ++i) {
                Eigen::Vector4d pred = h(x, i);
                double pos_err = (armor.position - pred.head<3>()).norm();
                double ang_err = std::abs(std::atan2(std::sin(armor.yaw - pred(3)), std::cos(armor.yaw - pred(3))));
                double err = pos_err * 10.0 + ang_err;
                if (err < min_combined_err) {
                    min_combined_err = err;
                    best_id = i;
                }
            }
            accumulated_errors_[k] += min_combined_err;
            if (k == 0) current_best_id = best_id; 

            Eigen::Matrix4d R = Eigen::Matrix4d::Zero();
            R.block<3, 3>(0, 0) = R_pos_world;
            R(3, 3) = (best_id == last_armor_id_) ? r_yaw_ : r_yaw_ * r_yaw_adaptive_factor_;

            auto h_func = [this, best_id](const Eigen::Matrix<double, STATE_DIM, 1>& x_in) {
                return this->h(x_in, best_id);
            };
            if (ukfs_[k].update<MEAS_DIM>(z, h_func, R, normalize_meas, normalize_state, 15.0)) {
                any_success = true;
            }
        }

        if (current_best_id != last_armor_id_) {
            armor_switch_count_++;
            last_armor_id_ = current_best_id;
        }

        if (armor_switch_count_ >= 2 || update_count_ >= 100) {
            best_ukf_idx_ = (accumulated_errors_[0] < accumulated_errors_[1]) ? 0 : 1;
            confirmation_state_ = ConfirmationState::CONFIRMED;
            RCLCPP_INFO(rclcpp::get_logger("robot_target"), 
                "Hypothesis confirmed after %d switches! Best index: %d, Total updates: %d", 
                armor_switch_count_, best_ukf_idx_, update_count_);
        }
    } else {
        const auto x = ukfs_[best_ukf_idx_].getState();
        int best_id = 0;
        double min_angle_err = 1e10;
        for (int i = 0; i < 4; ++i) {
            double armor_angle = x(6) + i * M_PI / 2.0;
            double angle_err = std::abs(std::atan2(std::sin(armor.yaw - armor_angle), std::cos(armor.yaw - armor_angle)));
            if (angle_err < min_angle_err) {
                min_angle_err = angle_err;
                best_id = i;
            }
        }
        
        Eigen::Matrix4d R = Eigen::Matrix4d::Zero();
        R.block<3, 3>(0, 0) = R_pos_world;
        R(3, 3) = (best_id == last_armor_id_) ? r_yaw_ : r_yaw_ * r_yaw_adaptive_factor_;

        auto h_func = [this, best_id](const Eigen::Matrix<double, STATE_DIM, 1>& x_in) {
            return this->h(x_in, best_id);
        };
        
        if (adaptive_tracking_) {
            Eigen::Matrix<double, STATE_DIM, STATE_DIM> Q_base = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Zero();
            Q_base(0,0) = q_x_; Q_base(2,2) = q_y_; Q_base(4,4) = q_z_;
            Q_base(1,1) = q_vx_; Q_base(3,3) = q_vy_; Q_base(5,5) = q_vz_;
            Q_base(6,6) = q_yaw_; Q_base(7,7) = q_v_yaw_;
            Q_base(8,8) = Q_base(9,9) = Q_base(10,10) = q_geo_;
            if (ukfs_[best_ukf_idx_].updateAdaptiveQ<MEAS_DIM>(z, h_func, R, Q_adaptive_, Q_base, q_alpha_, normalize_meas, normalize_state, 15.0)) {
                any_success = true;
            }
        } else {
            if (ukfs_[best_ukf_idx_].update<MEAS_DIM>(z, h_func, R, normalize_meas, normalize_state, 15.0)) {
                any_success = true;
            }
        }
        last_armor_id_ = best_id;
    }
    
    if (any_success) {
        update_count_++;
        return true;
    }
    return false;
}

bool RobotTarget::isConverged() const {
    const auto& cov = ukfs_[best_ukf_idx_].getCovariance().diagonal();
    return update_count_ > min_update_count_ && 
           cov(0) < max_pos_cov_ && cov(2) < max_pos_cov_ && cov(4) < max_pos_cov_ && 
           cov(6) < max_yaw_cov_;
}

bool RobotTarget::isDiverged() const {
    const auto& x = ukfs_[best_ukf_idx_].getState();
    const auto& cov = ukfs_[best_ukf_idx_].getCovariance().diagonal();
    if (cov.head<3>().maxCoeff() > 1000.0) return true;
    if (x.head<3>().norm() > 400.0) return true;
    if (x(8) < 0.05 || x(8) > 0.6) return true; 
    return false;
}

Eigen::Vector4d RobotTarget::h(const Eigen::VectorXd& x, int id) const {
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

std::vector<Eigen::Vector4d> RobotTarget::getResolvedArmors() const {
    std::vector<Eigen::Vector4d> armors;
    const auto x = ukfs_[best_ukf_idx_].getState();
    for (int i = 0; i < 4; ++i) {
        armors.push_back(h(x, i));
    }
    return armors;
}

Eigen::VectorXd RobotTarget::getPredictedState(const rclcpp::Time& time) const {
    double dt = (time - last_time_).seconds();
    auto x = ukfs_[best_ukf_idx_].getState();
    if (dt > 0) {
        x(0) += x(1) * dt; 
        x(2) += x(3) * dt; 
        x(4) += x(5) * dt; 
        x(6) += x(7) * dt; 
        x(6) = std::atan2(std::sin(x(6)), std::cos(x(6)));
    }
    return x;
}

GeometricParams RobotTarget::getGeometricParams() const {
    const auto x = ukfs_[best_ukf_idx_].getState();
    return {x(8), x(9), x(10), type_};
}

} // namespace robot_auto_aim
