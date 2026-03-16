#include "robot_auto_aim/outpost_target.hpp"
#include <cmath>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>

namespace robot_auto_aim {

// Height difference between outpost armors
constexpr double HEIGHT_DIFF = 0.102;
// Angular separation between outpost armors
constexpr double ANGULAR_OFFSET = 2.0 * M_PI / 3.0;

OutpostTarget::OutpostTarget() 
    : best_ukf_idx_(0), confirmation_state_(ConfirmationState::CONFIRMING),
      armor_num_(3), priority_(0), last_time_(0), update_count_(0),
      q_x_(0.001), q_y_(0.001), q_z_(0.001), 
      q_vx_(0.1), q_vy_(0.1), q_vz_(0.1),
      q_yaw_(0.01), q_v_yaw_(0.1), q_geo_(0.0001),
      r_x_(0.5), r_y_(0.5), r_z_(0.5), r_yaw_(0.05), r_yaw_adaptive_factor_(50.0),
      dist_scale_coeff_(0.1), z_scale_coeff_(5.0),
      adaptive_tracking_(false), q_alpha_(0.1) {
    for (int i = 0; i < 3; ++i) {
        ukfs_[i] = robot_utils::UKF<STATE_DIM>(0.001, 2.0, 0.0);
        accumulated_errors_[i] = 0.0;
        last_armor_ids_[i] = 0;
    }
}

void OutpostTarget::init(const TrackerArmor& armor) {
    name_ = armor.number;
    type_ = armor.type;
    priority_ = armor.priority;
    
    confirmation_state_ = ConfirmationState::CONFIRMING;
    best_ukf_idx_ = 0;

    double r_init = 0.25; // Initial guess for outpost radius
    
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> P0 = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Identity();
    P0.block<6, 6>(0, 0) *= 0.1;
    P0(1, 1) = 1.0; P0(3, 3) = 1.0; P0(5, 5) = 1.0;
    P0(6, 6) = 0.5;
    P0(7, 7) = 10.0;
    P0(8, 8) = 0.1; 
    
    // Initialize 3 UKFs with 3 different hypotheses for the first armor ID
    for (int k = 0; k < 3; ++k) {
        double outpost_true_yaw = robot_utils::normalize_angle(armor.yaw - k * ANGULAR_OFFSET);
        double outpost_true_z = armor.position.z() + k * HEIGHT_DIFF;
        double outpost_true_cx = armor.position.x() + r_init * std::cos(armor.yaw);
        double outpost_true_cy = armor.position.y() + r_init * std::sin(armor.yaw);

        Eigen::Matrix<double, STATE_DIM, 1> x0;
        x0 << outpost_true_cx, 0, outpost_true_cy, 0, outpost_true_z, 0, outpost_true_yaw, 0, r_init;
        
        ukfs_[k].init(x0, P0);
        accumulated_errors_[k] = 0.0;
        observed_ids_[k].clear();
        observed_ids_[k].insert(k);
        last_armor_ids_[k] = k;
    }
    
    Q_adaptive_ = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Zero();
    Q_adaptive_(0,0) = q_x_; Q_adaptive_(2,2) = q_y_; Q_adaptive_(4,4) = q_z_;
    Q_adaptive_(1,1) = q_vx_; Q_adaptive_(3,3) = q_vy_; Q_adaptive_(5,5) = q_vz_;
    Q_adaptive_(6,6) = q_yaw_;
    Q_adaptive_(7,7) = q_v_yaw_;
    Q_adaptive_(8,8) = q_geo_;
    
    last_time_ = armor.timestamp;
    update_count_ = 1;
}

void OutpostTarget::predict(const rclcpp::Time& time) {
    double dt = (time - last_time_).seconds();
    if (dt <= 0) return;
    
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> Q = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Zero();
    Q(0,0) = q_x_; Q(2,2) = q_y_; Q(4,4) = q_z_;
    Q(1,1) = q_vx_; Q(3,3) = q_vy_; Q(5,5) = q_vz_;
    Q(6,6) = q_yaw_; Q(7,7) = q_v_yaw_; Q(8,8) = q_geo_;
    
    auto f = [dt](const Eigen::Matrix<double, STATE_DIM, 1>& x) {
        Eigen::Matrix<double, STATE_DIM, 1> x_out = x;
        x_out(0) += x(1) * dt; 
        x_out(2) += x(3) * dt; 
        x_out(4) += x(5) * dt; 
        x_out(6) += x(7) * dt; 
        return x_out;
    };
    
    auto normalize_yaw = [](Eigen::Matrix<double, STATE_DIM, 1>& x) {
        x(6) = robot_utils::normalize_angle(x(6));
    };
    
    if (confirmation_state_ == ConfirmationState::CONFIRMING) {
        for (int k = 0; k < 3; ++k) {
            ukfs_[k].predict(f, adaptive_tracking_ ? Q_adaptive_ : Q, normalize_yaw);
        }
    } else {
        ukfs_[best_ukf_idx_].predict(f, adaptive_tracking_ ? Q_adaptive_ : Q, normalize_yaw);
    }
    
    last_time_ = time;
}

bool OutpostTarget::update(const TrackerArmor& armor) {
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
    Eigen::Matrix4d R_base = Eigen::Matrix4d::Zero();
    R_base.block<3, 3>(0, 0) = R_pos_world;

    auto normalize_meas = [](Eigen::Vector4d& z_diff) {
        z_diff(3) = robot_utils::normalize_angle(z_diff(3));
    };
    auto normalize_state = [](Eigen::Matrix<double, STATE_DIM, 1>& x_diff) {
        x_diff(6) = robot_utils::normalize_angle(x_diff(6));
    };

    bool any_success = false;

    if (confirmation_state_ == ConfirmationState::CONFIRMING) {
        for (int k = 0; k < 3; ++k) {
            const auto x = ukfs_[k].getState();
            int best_id = 0;
            double min_combined_error = 1e10;

            for (int i = 0; i < 3; ++i) {
                Eigen::Vector4d predicted_armor_meas = h(x, i);
                Eigen::Vector3d pos_error_vec = armor.position - predicted_armor_meas.head<3>();
                double pos_error = pos_error_vec.norm();
                double angle_error = std::abs(robot_utils::normalize_angle(armor.yaw - predicted_armor_meas(3)));
                double combined_error = pos_error * 100 + angle_error; 
                if (combined_error < min_combined_error) {
                    min_combined_error = combined_error;
                    best_id = i;
                }
            }
            
            observed_ids_[k].insert(best_id);
            accumulated_errors_[k] += min_combined_error;

            Eigen::Matrix4d R = R_base;
            R(3, 3) = (best_id == last_armor_ids_[k]) ? r_yaw_ : r_yaw_ * r_yaw_adaptive_factor_;

            auto h_func = [this, best_id](const Eigen::Matrix<double, STATE_DIM, 1>& x_in) {
                return this->h(x_in, best_id);
            };
            
            bool success = ukfs_[k].update<MEAS_DIM>(z, h_func, R, normalize_meas, normalize_state, 15.0);
            if (success) {
                last_armor_ids_[k] = best_id;
                any_success = true;
            }
        }

        double min_err = 1e10;
        for(int k=0; k<3; ++k) {
            if(accumulated_errors_[k] < min_err) {
                min_err = accumulated_errors_[k];
                best_ukf_idx_ = k;
            }
        }

        if (observed_ids_[best_ukf_idx_].size() >= 3) {
            confirmation_state_ = ConfirmationState::CONFIRMED;
            RCLCPP_INFO(rclcpp::get_logger("outpost_target"), "Outpost confirmed! Best hypothesis: %d, Error: %.2f", best_ukf_idx_, min_err);
        }

    } else {
        const auto x = ukfs_[best_ukf_idx_].getState();
        int best_id = 0;
        double min_combined_error = 1e10;

        for (int i = 0; i < 3; ++i) {
            Eigen::Vector4d predicted_armor_meas = h(x, i);
            Eigen::Vector3d pos_error_vec = armor.position - predicted_armor_meas.head<3>();
            double pos_error = pos_error_vec.norm();
            double angle_error = std::abs(robot_utils::normalize_angle(armor.yaw - predicted_armor_meas(3)));
            double combined_error = pos_error * 100 + angle_error; 
            if (combined_error < min_combined_error) {
                min_combined_error = combined_error;
                best_id = i;
            }
        }

        Eigen::Matrix4d R = R_base;
        R(3, 3) = (best_id == last_armor_ids_[best_ukf_idx_]) ? r_yaw_ : r_yaw_ * r_yaw_adaptive_factor_;

        auto h_func = [this, best_id](const Eigen::Matrix<double, STATE_DIM, 1>& x_in) {
            return this->h(x_in, best_id);
        };

        any_success = ukfs_[best_ukf_idx_].update<MEAS_DIM>(z, h_func, R, normalize_meas, normalize_state, 15.0);
        if (any_success) {
            last_armor_ids_[best_ukf_idx_] = best_id;
        }
    }

    if (any_success) {
        update_count_++;
        return true;
    }
    return false;
}

void OutpostTarget::updateParams(double q_x, double q_y, double q_z, double q_vx, double q_vy, double q_vz, double q_yaw, double q_v_yaw, double q_geo, double r_x, double r_y, double r_z, double r_yaw, double r_yaw_adaptive_factor, bool adaptive_tracking, double q_alpha, double dist_scale_coeff, double z_scale_coeff) {
    q_x_ = q_x; q_y_ = q_y; q_z_ = q_z; q_vx_ = q_vx; q_vy_ = q_vy; q_vz_ = q_vz; q_yaw_ = q_yaw; q_v_yaw_ = q_v_yaw; q_geo_ = q_geo; r_x_ = r_x; r_y_ = r_y; r_z_ = r_z; r_yaw_ = r_yaw; r_yaw_adaptive_factor_ = r_yaw_adaptive_factor; adaptive_tracking_ = adaptive_tracking; q_alpha_ = q_alpha; dist_scale_coeff_ = dist_scale_coeff; z_scale_coeff_ = z_scale_coeff;
}
void OutpostTarget::setUKFParams(double alpha, double beta, double kappa) {
    for (int k=0; k<3; ++k) {
        Eigen::VectorXd current_x = ukfs_[k].getState(); 
        Eigen::MatrixXd current_P = ukfs_[k].getCovariance(); 
        ukfs_[k] = robot_utils::UKF<STATE_DIM>(alpha, beta, kappa); 
        if (current_x.size() == STATE_DIM) { ukfs_[k].init(current_x, current_P); }
    }
}
bool OutpostTarget::isConverged() const {
    const auto& cov = ukfs_[best_ukf_idx_].getCovariance().diagonal(); return update_count_ > 10 && cov(0) < 0.5 && cov(2) < 0.5 && cov(4) < 0.5 && cov(6) < 0.2;
}
bool OutpostTarget::isDiverged() const {
    const auto& x = ukfs_[best_ukf_idx_].getState(); const auto& cov = ukfs_[best_ukf_idx_].getCovariance().diagonal(); if (cov.head<3>().maxCoeff() > 100.0) return true; if (x.head<3>().norm() > 40.0) return true; if (x(8) < 0.1 || x(8) > 0.5) return true; return false;
}
Eigen::Vector4d OutpostTarget::h(const Eigen::VectorXd& x, int id) const {
    double yaw = x(6); double r = x(8); double angle = robot_utils::normalize_angle(yaw + id * ANGULAR_OFFSET); double ax = x(0) - r * std::cos(angle); double ay = x(2) - r * std::sin(angle); double az = x(4) - id * HEIGHT_DIFF; return Eigen::Vector4d(ax, ay, az, angle);
}
std::vector<Eigen::Vector4d> OutpostTarget::getResolvedArmors() const {
    std::vector<Eigen::Vector4d> armors; const auto x = ukfs_[best_ukf_idx_].getState(); for (int i = 0; i < 3; ++i) { armors.push_back(h(x, i)); } return armors;
}
Eigen::VectorXd OutpostTarget::getPredictedState(const rclcpp::Time& time) const {
    double dt = (time - last_time_).seconds(); auto x = ukfs_[best_ukf_idx_].getState(); if (dt > 0) { x(0) += x(1) * dt; x(2) += x(3) * dt; x(4) += x(5) * dt; x(6) += x(7) * dt; x(6) = robot_utils::normalize_angle(x(6)); } return x;
}

} // namespace robot_auto_aim
