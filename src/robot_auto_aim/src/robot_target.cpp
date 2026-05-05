#include "robot_auto_aim/robot_target.hpp"
#include <cmath>
#include <algorithm>

namespace robot_auto_aim {
    RobotTarget::RobotTarget()
        : best_ukf_idx_(0), confirmation_state_(ConfirmationState::CONFIRMED), armor_num_(4), priority_(0),
          last_time_(0), last_armor_ids_{0, 0},
          update_count_(0), sigma_pos_(20.0), sigma_yaw_(2.0), q_geo_(0.0001),
          omega_freeze_thresh_(0.5),
          r_range_(0.01), r_range_k_(0.5), r_angle_(0.0003),
          r_yaw_(0.05), r_yaw_adaptive_factor_(50.0), r_yaw_viewing_k_(10.0),
          mahalanobis_thresh_(15.0),
          p0_pos_(0.1), p0_vel_(10.0), p0_yaw_(0.5), p0_omega_(100.0), p0_geo_(0.1),
          min_update_count_(5), max_pos_cov_(3.0), max_yaw_cov_(1.0),
          adaptive_tracking_(false), q_alpha_(0.1) {
        for (int i = 0; i < 2; ++i) {
            ukfs_[i] = robot_utils::UKF<STATE_DIM>(0.001, 2.0, 0.0);
            accumulated_errors_[i] = 0.0;
        }
    }

    void RobotTarget::init(const TrackerArmor &armor, const GeometricParams &init_geo) {
        name_ = armor.number;
        type_ = armor.type;
        priority_ = armor.priority;
        last_time_ = armor.timestamp;
        update_count_ = 1;
        armor_switch_count_ = 0;
        last_armor_ids_[0] = 0;
        last_armor_ids_[1] = 0;

        Eigen::Matrix<double, STATE_DIM, STATE_DIM> P0 = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Identity();
        P0(0, 0) = p0_pos_;   // cx
        P0(1, 1) = p0_vel_;   // vx
        P0(2, 2) = p0_pos_;   // cy
        P0(3, 3) = p0_vel_;   // vy
        P0(4, 4) = p0_pos_;   // cz
        P0(5, 5) = p0_vel_;   // vz
        P0(6, 6) = p0_yaw_;   // yaw
        P0(7, 7) = p0_omega_; // omega
        P0(8, 8) = p0_geo_;   // r
        P0(9, 9) = p0_geo_;   // l
        P0(10, 10) = p0_geo_; // h

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

        // Initialize Q_adaptive_ with CWNA baseline (nominal dt=0.01 for 100Hz)
        Q_adaptive_ = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Zero();
        double sp2 = sigma_pos_ * sigma_pos_;
        double sy2 = sigma_yaw_ * sigma_yaw_;
        constexpr double nom_dt = 0.01;
        // Position-velocity diagonal (simplified, no cross-terms for adaptive baseline)
        Q_adaptive_(0, 0) = sp2 * nom_dt;  // cx
        Q_adaptive_(1, 1) = sp2 * nom_dt;  // vx
        Q_adaptive_(2, 2) = sp2 * nom_dt;  // cy
        Q_adaptive_(3, 3) = sp2 * nom_dt;  // vy
        Q_adaptive_(4, 4) = sp2 * nom_dt;  // cz
        Q_adaptive_(5, 5) = sp2 * nom_dt;  // vz
        Q_adaptive_(6, 6) = sy2 * nom_dt;  // yaw
        Q_adaptive_(7, 7) = sy2 * nom_dt;  // omega
        Q_adaptive_(8, 8) = Q_adaptive_(9, 9) = Q_adaptive_(10, 10) = q_geo_ * nom_dt;
    }

    void RobotTarget::predict(const rclcpp::Time &time) {
        double dt = (time - last_time_).seconds();
        if (dt <= 0) return;

        // CWNA (Continuous White Noise Acceleration) process noise matrix
        // For each [pos, vel] pair: Q = σ² * [dt³/3, dt²/2; dt²/2, dt]
        double sp2 = sigma_pos_ * sigma_pos_;
        double sy2 = sigma_yaw_ * sigma_yaw_;
        double dt2 = dt * dt;
        double dt3 = dt2 * dt;

        Eigen::Matrix<double, STATE_DIM, STATE_DIM> Q = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Zero();
        // cx-vx pair (indices 0, 1)
        Q(0, 0) = sp2 * dt3 / 3.0;  Q(0, 1) = sp2 * dt2 / 2.0;
        Q(1, 0) = sp2 * dt2 / 2.0;  Q(1, 1) = sp2 * dt;
        // cy-vy pair (indices 2, 3)
        Q(2, 2) = sp2 * dt3 / 3.0;  Q(2, 3) = sp2 * dt2 / 2.0;
        Q(3, 2) = sp2 * dt2 / 2.0;  Q(3, 3) = sp2 * dt;
        // cz-vz pair (indices 4, 5)
        Q(4, 4) = sp2 * dt3 / 3.0;  Q(4, 5) = sp2 * dt2 / 2.0;
        Q(5, 4) = sp2 * dt2 / 2.0;  Q(5, 5) = sp2 * dt;
        // yaw-omega pair (indices 6, 7)
        Q(6, 6) = sy2 * dt3 / 3.0;  Q(6, 7) = sy2 * dt2 / 2.0;
        Q(7, 6) = sy2 * dt2 / 2.0;  Q(7, 7) = sy2 * dt;
        // Geometric parameters (indices 8, 9, 10) - omega-adaptive random walk
        // When |omega| is small, geometry params have low observability (r mixes with cx/cy),
        // so scale down q_geo to prevent unconstrained drift
        double abs_omega = std::abs(ukfs_[best_ukf_idx_].getState()(7));
        double omega_scale = (omega_freeze_thresh_ > 1e-6)
            ? std::min(1.0, abs_omega / omega_freeze_thresh_) : 1.0;
        double effective_q_geo = q_geo_ * omega_scale;
        Q(8, 8) = Q(9, 9) = Q(10, 10) = effective_q_geo * dt;

        auto f = [dt](const Eigen::Matrix<double, STATE_DIM, 1> &x) {
            Eigen::Matrix<double, STATE_DIM, 1> x_out = x;
            x_out(0) += x(1) * dt;
            x_out(2) += x(3) * dt;
            x_out(4) += x(5) * dt;
            x_out(6) += x(7) * dt;
            return x_out;
        };

        auto normalize_yaw = [](Eigen::Matrix<double, STATE_DIM, 1> &x) {
            x(6) = std::atan2(std::sin(x(6)), std::cos(x(6)));
        };

        const auto& Q_used = adaptive_tracking_ ? Q_adaptive_ : Q;

        // State transitions for CONFIRMING states
        if (confirmation_state_ == ConfirmationState::CONFIRMING) {
            double abs_omega = std::abs(ukfs_[best_ukf_idx_].getState()(7));
            if (abs_omega < omega_freeze_thresh_) {
                // CONFIRMING → CONFIRMING_FROZEN: pick lower-error UKF, pause the other
                best_ukf_idx_ = (accumulated_errors_[0] < accumulated_errors_[1]) ? 0 : 1;
                confirmation_state_ = ConfirmationState::CONFIRMING_FROZEN;
                RCLCPP_INFO(rclcpp::get_logger("robot_target"),
                            "Low omega (%.2f < %.2f), freezing to single UKF[%d]",
                            abs_omega, omega_freeze_thresh_, best_ukf_idx_);
            }
        } else if (confirmation_state_ == ConfirmationState::CONFIRMING_FROZEN) {
            double abs_omega = std::abs(ukfs_[best_ukf_idx_].getState()(7));
            if (abs_omega >= omega_freeze_thresh_) {
                // CONFIRMING_FROZEN → CONFIRMING: re-derive paused hypothesis, resume dual UKF
                rederivePausedHypothesis();
                confirmation_state_ = ConfirmationState::CONFIRMING;
                RCLCPP_INFO(rclcpp::get_logger("robot_target"),
                            "Omega recovered (%.2f), resuming dual UKF", abs_omega);
            }
        }

        // Execute predict based on current state
        if (confirmation_state_ == ConfirmationState::CONFIRMING) {
            for (int i = 0; i < 2; ++i) {
                ukfs_[i].predict(f, Q_used, normalize_yaw);
            }
        } else {
            // CONFIRMING_FROZEN or CONFIRMED: single UKF
            ukfs_[best_ukf_idx_].predict(f, Q_used, normalize_yaw);
        }
        last_time_ = time;
    }

    bool RobotTarget::update(const TrackerArmor &armor) {
        // Convert observation to spherical coordinates
        auto sph = robot_utils::cartesianToSpherical(armor.position);
        Eigen::Vector4d z;
        z << sph[robot_utils::RANGE], sph[robot_utils::AZIMUTH], sph[robot_utils::ELEVATION], armor.yaw;
        measurement_ = z;

        double range = sph[robot_utils::RANGE];

        // Normalize functions for angular residuals
        auto normalize_meas = [](Eigen::Vector4d &z_diff) {
            z_diff(1) = std::atan2(std::sin(z_diff(1)), std::cos(z_diff(1)));  // azimuth
            z_diff(3) = std::atan2(std::sin(z_diff(3)), std::cos(z_diff(3)));  // yaw
        };
        auto normalize_state = [](Eigen::Matrix<double, STATE_DIM, 1> &x_diff) {
            x_diff(6) = std::atan2(std::sin(x_diff(6)), std::cos(x_diff(6)));
        };

        // Viewing-angle-dependent yaw noise: amplify when head-on (cos_va → 1)
        double cos_va = std::cos(armor.viewing_angle);
        double base_r_yaw = r_yaw_ * (1.0 + r_yaw_viewing_k_ * cos_va * cos_va);

        bool any_success = false;

        if (confirmation_state_ == ConfirmationState::CONFIRMING) {
            int old_armor_id_0 = last_armor_ids_[0];
            int current_best_id = 0;
            for (int k = 0; k < 2; ++k) {
                const auto x = ukfs_[k].getState();
                int best_id = 0;
                double min_combined_err = 1e10;
                for (int i = 0; i < 4; ++i) {
                    Eigen::Vector4d pred = getArmorCartesian(x, i);
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

                // Spherical R matrix — diagonal
                Eigen::Matrix4d R = Eigen::Matrix4d::Zero();
                R(0, 0) = r_range_ * (1.0 + r_range_k_ * range * range);
                R(1, 1) = r_angle_;
                R(2, 2) = r_angle_;
                R(3, 3) = (best_id == last_armor_ids_[k]) ? base_r_yaw : base_r_yaw * r_yaw_adaptive_factor_;

                auto h_func = [this, best_id](const Eigen::Matrix<double, STATE_DIM, 1> &x_in) {
                    return this->h(x_in, best_id);
                };
                if (ukfs_[k].update<MEAS_DIM>(z, h_func, R, normalize_meas, normalize_state, mahalanobis_thresh_)) {
                    any_success = true;
                }
                last_armor_ids_[k] = best_id;
            }

            if (current_best_id != old_armor_id_0) {
                armor_switch_count_++;
            }

            // CONFIRMING state guarantees |omega| >= thresh (low omega transitions to FROZEN in predict),
            // so no additional omega check needed here
            if (armor_switch_count_ >= 2 || update_count_ >= 100) {
                best_ukf_idx_ = (accumulated_errors_[0] < accumulated_errors_[1]) ? 0 : 1;
                confirmation_state_ = ConfirmationState::CONFIRMED;
                RCLCPP_INFO(rclcpp::get_logger("robot_target"),
                            "Hypothesis confirmed after %d switches! Best index: %d, Total updates: %d",
                            armor_switch_count_, best_ukf_idx_, update_count_);
            }
        } else if (confirmation_state_ == ConfirmationState::CONFIRMING_FROZEN) {
            // Single UKF mode: only update active hypothesis, no error accumulation or switch counting
            const auto x = ukfs_[best_ukf_idx_].getState();
            int best_id = 0;
            double min_angle_err = 1e10;
            for (int i = 0; i < 4; ++i) {
                double armor_angle = x(6) + i * M_PI / 2.0;
                double angle_err = std::abs(std::atan2(std::sin(armor.yaw - armor_angle),
                                                       std::cos(armor.yaw - armor_angle)));
                if (angle_err < min_angle_err) {
                    min_angle_err = angle_err;
                    best_id = i;
                }
            }

            Eigen::Matrix4d R = Eigen::Matrix4d::Zero();
            R(0, 0) = r_range_ * (1.0 + r_range_k_ * range * range);
            R(1, 1) = r_angle_;
            R(2, 2) = r_angle_;
            R(3, 3) = (best_id == last_armor_ids_[best_ukf_idx_]) ? base_r_yaw : base_r_yaw * r_yaw_adaptive_factor_;

            auto h_func = [this, best_id](const Eigen::Matrix<double, STATE_DIM, 1> &x_in) {
                return this->h(x_in, best_id);
            };
            if (ukfs_[best_ukf_idx_].template update<MEAS_DIM>(z, h_func, R, normalize_meas, normalize_state, mahalanobis_thresh_)) {
                any_success = true;
            }
            last_armor_ids_[best_ukf_idx_] = best_id;
        } else {
            const auto x = ukfs_[best_ukf_idx_].getState();
            int best_id = 0;
            double min_angle_err = 1e10;
            for (int i = 0; i < 4; ++i) {
                double armor_angle = x(6) + i * M_PI / 2.0;
                double angle_err = std::abs(std::atan2(std::sin(armor.yaw - armor_angle),
                                                       std::cos(armor.yaw - armor_angle)));
                if (angle_err < min_angle_err) {
                    min_angle_err = angle_err;
                    best_id = i;
                }
            }

            // Spherical R matrix — diagonal
            Eigen::Matrix4d R = Eigen::Matrix4d::Zero();
            R(0, 0) = r_range_ * (1.0 + r_range_k_ * range * range);
            R(1, 1) = r_angle_;
            R(2, 2) = r_angle_;
            R(3, 3) = (best_id == last_armor_ids_[best_ukf_idx_]) ? base_r_yaw : base_r_yaw * r_yaw_adaptive_factor_;

            auto h_func = [this, best_id](const Eigen::Matrix<double, STATE_DIM, 1> &x_in) {
                return this->h(x_in, best_id);
            };

            if (adaptive_tracking_) {
                Eigen::Matrix<double, STATE_DIM, STATE_DIM> Q_base = Eigen::Matrix<double, STATE_DIM,
                    STATE_DIM>::Zero();
                double sp2 = sigma_pos_ * sigma_pos_;
                double sy2 = sigma_yaw_ * sigma_yaw_;
                constexpr double nom_dt = 0.01;
                Q_base(0, 0) = Q_base(2, 2) = Q_base(4, 4) = sp2 * nom_dt;
                Q_base(1, 1) = Q_base(3, 3) = Q_base(5, 5) = sp2 * nom_dt;
                Q_base(6, 6) = sy2 * nom_dt;
                Q_base(7, 7) = sy2 * nom_dt;
                Q_base(8, 8) = Q_base(9, 9) = Q_base(10, 10) = q_geo_ * nom_dt;
                // Apply same omega-adaptive scaling to adaptive Q baseline
                double ada_abs_omega = std::abs(ukfs_[best_ukf_idx_].getState()(7));
                double ada_omega_scale = (omega_freeze_thresh_ > 1e-6)
                    ? std::min(1.0, ada_abs_omega / omega_freeze_thresh_) : 1.0;
                Q_base(8, 8) *= ada_omega_scale;
                Q_base(9, 9) *= ada_omega_scale;
                Q_base(10, 10) *= ada_omega_scale;
                if (ukfs_[best_ukf_idx_].template updateAdaptiveQ<MEAS_DIM>(z, h_func, R, Q_adaptive_, Q_base, q_alpha_,
                                                                   normalize_meas, normalize_state, mahalanobis_thresh_)) {
                    any_success = true;
                }
            } else {
                if (ukfs_[best_ukf_idx_].template update<MEAS_DIM>(z, h_func, R, normalize_meas, normalize_state, mahalanobis_thresh_)) {
                    any_success = true;
                }
            }
            last_armor_ids_[best_ukf_idx_] = best_id;
        }

        if (any_success) {
            update_count_++;
            return true;
        }
        return false;
    }

    bool RobotTarget::isConverged() const {
        const auto &cov = ukfs_[best_ukf_idx_].getCovariance().diagonal();
        return update_count_ > min_update_count_ &&
               cov(0) < max_pos_cov_ && cov(2) < max_pos_cov_ && cov(4) < max_pos_cov_ &&
               cov(6) < max_yaw_cov_;
    }

    bool RobotTarget::isDiverged() const {
        const auto &x = ukfs_[best_ukf_idx_].getState();
        const auto &cov = ukfs_[best_ukf_idx_].getCovariance().diagonal();
        // Position covariances: cx(0), cy(2), cz(4) - skip velocity indices
        if (cov(0) > 1000.0 || cov(2) > 1000.0 || cov(4) > 1000.0) return true;
        // Position norm: cx(0), cy(2), cz(4)
        double pos_norm = std::sqrt(x(0) * x(0) + x(2) * x(2) + x(4) * x(4));
        if (pos_norm > 400.0) return true;
        if (x(8) < 0.05 || x(8) > 0.6) return true;
        // Geometry covariance divergence: r/l/h uncertainty growing unbounded
        if (cov(8) > 1.0 || cov(9) > 1.0 || cov(10) > 1.0) return true;
        return false;
    }

    void RobotTarget::rederivePausedHypothesis() {
        int active = best_ukf_idx_;
        int paused = 1 - active;
        auto x = ukfs_[active].getState();
        auto P = ukfs_[active].getCovariance();

        // Re-derive alternative hypothesis: rotate geometry by 90°
        Eigen::Matrix<double, STATE_DIM, 1> x_alt = x;
        x_alt(8) = x(8) + x(9);     // r_alt = r + l
        x_alt(9) = -x(9);            // l_alt = -l
        x_alt(10) = -x(10);          // h_alt = -h

        // Preserve kinematic covariance, reset geometry covariance and cross-terms
        Eigen::Matrix<double, STATE_DIM, STATE_DIM> P_alt = P;
        for (int g = 8; g <= 10; ++g) {
            for (int j = 0; j < STATE_DIM; ++j) {
                P_alt(g, j) = P_alt(j, g) = 0.0;
            }
            P_alt(g, g) = p0_geo_;
        }

        ukfs_[paused].init(x_alt, P_alt);
        // Reset for fair comparison after resuming dual mode
        accumulated_errors_[0] = accumulated_errors_[1] = 0.0;
        armor_switch_count_ = 0;
    }

    Eigen::Vector4d RobotTarget::getArmorCartesian(const Eigen::VectorXd &x, int id) const {
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

    Eigen::Vector4d RobotTarget::h(const Eigen::VectorXd &x, int id) const {
        auto cart = getArmorCartesian(x, id);
        auto sph = robot_utils::cartesianToSpherical(cart.head<3>());
        return Eigen::Vector4d(sph[robot_utils::RANGE], sph[robot_utils::AZIMUTH],
                               sph[robot_utils::ELEVATION], cart(3));
    }

    std::vector<Eigen::Vector4d> RobotTarget::getResolvedArmors() const {
        std::vector<Eigen::Vector4d> armors;
        const auto x = ukfs_[best_ukf_idx_].getState();
        for (int i = 0; i < 4; ++i) {
            armors.push_back(getArmorCartesian(x, i));
        }
        return armors;
    }

    Eigen::VectorXd RobotTarget::getPredictedState(const rclcpp::Time &time) const {
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
