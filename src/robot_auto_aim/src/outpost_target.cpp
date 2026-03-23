#include "robot_auto_aim/outpost_target.hpp"
#include <cmath>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>

namespace robot_auto_aim {
    // Angular separation between outpost armors
    constexpr double ANGULAR_OFFSET = 2.0 * M_PI / 3.0;

    OutpostTarget::OutpostTarget()
        : best_ukf_idx_(0), confirmation_state_(ConfirmationState::CONFIRMING),
          armor_num_(3), priority_(0), last_time_(0), update_count_(0),
          sigma_pos_(0.1), sigma_yaw_(0.3), q_geo_(0.0001),
          omega_freeze_thresh_(0.5),
          r_range_(0.01), r_range_k_(0.5), r_angle_(0.0003),
          r_yaw_(0.05), r_yaw_adaptive_factor_(50.0), r_yaw_viewing_k_(10.0),
          mahalanobis_thresh_(15.0),
          p0_pos_(0.1), p0_yaw_(0.5), p0_omega_(10.0), p0_geo_(0.1),
          min_update_count_(5), max_pos_cov_(2.0), max_yaw_cov_(1.0),
          adaptive_tracking_(false), q_alpha_(0.1) {
        for (int i = 0; i < 3; ++i) {
            ukfs_[i] = robot_utils::UKF<STATE_DIM>(0.001, 2.0, 0.0);
            accumulated_errors_[i] = 0.0;
            last_armor_ids_[i] = 0;
        }
    }

    void OutpostTarget::init(const TrackerArmor &armor, const GeometricParams &init_geo) {
        name_ = armor.number;
        type_ = armor.type;
        priority_ = armor.priority;

        confirmation_state_ = ConfirmationState::CONFIRMING;
        best_ukf_idx_ = 0;

        double r_init = (init_geo.r > 0) ? init_geo.r : 0.25; // Initial guess for outpost radius
        double h_init = (init_geo.h > 0) ? init_geo.h : 0.102; // Initial guess for height difference

        Eigen::Matrix<double, STATE_DIM, STATE_DIM> P0 = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Identity();
        P0(0, 0) = p0_pos_;   // cx
        P0(1, 1) = p0_pos_;   // cy
        P0(2, 2) = p0_pos_;   // cz
        P0(3, 3) = p0_yaw_;   // yaw
        P0(4, 4) = p0_omega_; // v_yaw
        P0(5, 5) = p0_geo_;   // r
        P0(6, 6) = p0_geo_;   // h

        // Initialize 3 UKFs with 3 different hypotheses for the first armor ID
        for (int k = 0; k < 3; ++k) {
            double outpost_true_yaw = robot_utils::normalize_angle(armor.yaw - k * ANGULAR_OFFSET);
            double outpost_true_z = armor.position.z() + k * h_init;
            double outpost_true_cx = armor.position.x() + r_init * std::cos(armor.yaw);
            double outpost_true_cy = armor.position.y() + r_init * std::sin(armor.yaw);

            Eigen::Matrix<double, STATE_DIM, 1> x0;
            x0 << outpost_true_cx, outpost_true_cy, outpost_true_z, outpost_true_yaw, 0, r_init, h_init;

            ukfs_[k].init(x0, P0);
            accumulated_errors_[k] = 0.0;
            observed_ids_[k].clear();
            observed_ids_[k].insert(k);
            last_armor_ids_[k] = k;
        }

        // Initialize Q_adaptive_ with baseline values (nominal dt=0.01 for 100Hz)
        Q_adaptive_ = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Zero();
        double sp2 = sigma_pos_ * sigma_pos_;
        double sy2 = sigma_yaw_ * sigma_yaw_;
        constexpr double nom_dt = 0.01;
        Q_adaptive_(0, 0) = sp2 * nom_dt;  // cx
        Q_adaptive_(1, 1) = sp2 * nom_dt;  // cy
        Q_adaptive_(2, 2) = sp2 * nom_dt;  // cz
        Q_adaptive_(3, 3) = sy2 * nom_dt;  // yaw
        Q_adaptive_(4, 4) = sy2 * nom_dt;  // v_yaw
        Q_adaptive_(5, 5) = q_geo_ * nom_dt;  // r
        Q_adaptive_(6, 6) = q_geo_ * nom_dt;  // h

        last_time_ = armor.timestamp;
        update_count_ = 1;
    }

    void OutpostTarget::predict(const rclcpp::Time &time) {
        double dt = (time - last_time_).seconds();
        if (dt <= 0) return;

        double sp2 = sigma_pos_ * sigma_pos_;
        double sy2 = sigma_yaw_ * sigma_yaw_;
        double dt2 = dt * dt;
        double dt3 = dt2 * dt;

        Eigen::Matrix<double, STATE_DIM, STATE_DIM> Q = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Zero();
        // Position: random walk (no velocity states)
        Q(0, 0) = sp2 * dt;  // cx
        Q(1, 1) = sp2 * dt;  // cy
        Q(2, 2) = sp2 * dt;  // cz
        // yaw-v_yaw pair: CWNA (indices 3, 4)
        Q(3, 3) = sy2 * dt3 / 3.0;  Q(3, 4) = sy2 * dt2 / 2.0;
        Q(4, 3) = sy2 * dt2 / 2.0;  Q(4, 4) = sy2 * dt;
        // Geometric parameters: omega-adaptive random walk
        double abs_omega = std::abs(ukfs_[best_ukf_idx_].getState()(4));
        double omega_scale = (omega_freeze_thresh_ > 1e-6)
            ? std::min(1.0, abs_omega / omega_freeze_thresh_) : 1.0;
        double effective_q_geo = q_geo_ * omega_scale;
        Q(5, 5) = effective_q_geo * dt;  // r
        Q(6, 6) = effective_q_geo * dt;  // h

        auto f = [dt](const Eigen::Matrix<double, STATE_DIM, 1> &x) {
            Eigen::Matrix<double, STATE_DIM, 1> x_out = x;
            x_out(3) += x(4) * dt; // yaw += v_yaw * dt
            return x_out;
        };

        auto normalize_yaw = [](Eigen::Matrix<double, STATE_DIM, 1> &x) {
            x(3) = robot_utils::normalize_angle(x(3));
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

    bool OutpostTarget::update(const TrackerArmor &armor) {
        // Convert observation to spherical coordinates
        auto sph = robot_utils::cartesianToSpherical(armor.position);
        Eigen::Vector4d z;
        z << sph[robot_utils::RANGE], sph[robot_utils::AZIMUTH], sph[robot_utils::ELEVATION], armor.yaw;
        measurement_ = z;

        double range = sph[robot_utils::RANGE];

        auto normalize_meas = [](Eigen::Vector4d &z_diff) {
            z_diff(1) = std::atan2(std::sin(z_diff(1)), std::cos(z_diff(1)));  // azimuth
            z_diff(3) = robot_utils::normalize_angle(z_diff(3));               // yaw
        };
        auto normalize_state = [](Eigen::Matrix<double, STATE_DIM, 1> &x_diff) {
            x_diff(3) = robot_utils::normalize_angle(x_diff(3));
        };

        // Viewing-angle-dependent yaw noise
        double cos_va = std::cos(armor.viewing_angle);
        double base_r_yaw = r_yaw_ * (1.0 + r_yaw_viewing_k_ * cos_va * cos_va);

        bool any_success = false;

        if (confirmation_state_ == ConfirmationState::CONFIRMING) {
            for (int k = 0; k < 3; ++k) {
                const auto x = ukfs_[k].getState();
                int best_id = 0;
                double min_combined_error = 1e10;

                for (int i = 0; i < 3; ++i) {
                    Eigen::Vector4d predicted = getArmorCartesian(x, i);
                    Eigen::Vector3d pos_error_vec = armor.position - predicted.head<3>();
                    double pos_error = pos_error_vec.norm();
                    double angle_error = std::abs(robot_utils::normalize_angle(armor.yaw - predicted(3)));
                    double combined_error = pos_error * 100 + angle_error;
                    if (combined_error < min_combined_error) {
                        min_combined_error = combined_error;
                        best_id = i;
                    }
                }

                observed_ids_[k].insert(best_id);
                accumulated_errors_[k] += min_combined_error;

                // Spherical R matrix — diagonal
                Eigen::Matrix4d R = Eigen::Matrix4d::Zero();
                R(0, 0) = r_range_ * (1.0 + r_range_k_ * range * range);
                R(1, 1) = r_angle_;
                R(2, 2) = r_angle_;
                R(3, 3) = (best_id == last_armor_ids_[k]) ? base_r_yaw : base_r_yaw * r_yaw_adaptive_factor_;

                auto h_func = [this, best_id](const Eigen::Matrix<double, STATE_DIM, 1> &x_in) {
                    return this->h(x_in, best_id);
                };

                bool success = ukfs_[k].update<MEAS_DIM>(z, h_func, R, normalize_meas, normalize_state, mahalanobis_thresh_);
                if (success) {
                    last_armor_ids_[k] = best_id;
                    any_success = true;
                }
            }

            double min_err = 1e10;
            for (int k = 0; k < 3; ++k) {
                if (accumulated_errors_[k] < min_err) {
                    min_err = accumulated_errors_[k];
                    best_ukf_idx_ = k;
                }
            }

            if (observed_ids_[best_ukf_idx_].size() >= 3) {
                confirmation_state_ = ConfirmationState::CONFIRMED;
                RCLCPP_INFO(rclcpp::get_logger("outpost_target"), "Outpost confirmed! Best hypothesis: %d, Error: %.2f",
                            best_ukf_idx_, min_err);
            }
        } else {
            const auto x = ukfs_[best_ukf_idx_].getState();
            int best_id = 0;
            double min_combined_error = 1e10;

            for (int i = 0; i < 3; ++i) {
                Eigen::Vector4d predicted = getArmorCartesian(x, i);
                Eigen::Vector3d pos_error_vec = armor.position - predicted.head<3>();
                double pos_error = pos_error_vec.norm();
                double angle_error = std::abs(robot_utils::normalize_angle(armor.yaw - predicted(3)));
                double combined_error = pos_error * 100 + angle_error;
                if (combined_error < min_combined_error) {
                    min_combined_error = combined_error;
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

            any_success = ukfs_[best_ukf_idx_].update<MEAS_DIM>(z, h_func, R, normalize_meas, normalize_state, mahalanobis_thresh_);
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

    void OutpostTarget::updateParams(const TargetParams &params) {
        sigma_pos_ = params.sigma_pos;
        sigma_yaw_ = params.sigma_yaw;
        q_geo_ = params.q_geo;
        r_range_ = params.r_range;
        r_range_k_ = params.r_range_k;
        r_angle_ = params.r_angle;
        r_yaw_ = params.r_yaw;
        r_yaw_adaptive_factor_ = params.r_yaw_adaptive_factor;
        r_yaw_viewing_k_ = params.r_yaw_viewing_k;
        adaptive_tracking_ = params.adaptive_tracking;
        q_alpha_ = params.q_alpha;
        mahalanobis_thresh_ = params.mahalanobis_thresh;
        p0_pos_ = params.p0_pos;
        p0_yaw_ = params.p0_yaw;
        p0_omega_ = params.p0_omega;
        p0_geo_ = params.p0_geo;
        min_update_count_ = params.min_update_count;
        max_pos_cov_ = params.max_pos_cov;
        max_yaw_cov_ = params.max_yaw_cov;
    }

    void OutpostTarget::setUKFParams(double alpha, double beta, double kappa) {
        for (int k = 0; k < 3; ++k) {
            Eigen::VectorXd current_x = ukfs_[k].getState();
            Eigen::MatrixXd current_P = ukfs_[k].getCovariance();
            ukfs_[k] = robot_utils::UKF<STATE_DIM>(alpha, beta, kappa);
            if (current_x.size() == STATE_DIM) { ukfs_[k].init(current_x, current_P); }
        }
    }

    bool OutpostTarget::isConverged() const {
        const auto &cov = ukfs_[best_ukf_idx_].getCovariance().diagonal();
        return update_count_ > min_update_count_ &&
               cov(0) < max_pos_cov_ && cov(1) < max_pos_cov_ && cov(2) < max_pos_cov_ &&
               cov(3) < max_yaw_cov_;
    }

    bool OutpostTarget::isDiverged() const {
        const auto &x = ukfs_[best_ukf_idx_].getState();
        const auto &cov = ukfs_[best_ukf_idx_].getCovariance().diagonal();
        if (cov.head<3>().maxCoeff() > 100.0) return true;
        if (x.head<3>().norm() > 40.0) return true;
        if (x(5) < 0.1 || x(5) > 0.5) return true; // r
        if (x(6) < 0.05 || x(6) > 0.2) return true; // h
        return false;
    }

    Eigen::Vector4d OutpostTarget::getArmorCartesian(const Eigen::VectorXd &x, int id) const {
        double cx = x(0);
        double cy = x(1);
        double cz = x(2);
        double yaw = x(3);
        double r = x(5);
        double h = x(6);
        double angle = robot_utils::normalize_angle(yaw + id * ANGULAR_OFFSET);
        double ax = cx - r * std::cos(angle);
        double ay = cy - r * std::sin(angle);
        double az = cz - id * h;
        return Eigen::Vector4d(ax, ay, az, angle);
    }

    Eigen::Vector4d OutpostTarget::h(const Eigen::VectorXd &x, int id) const {
        auto cart = getArmorCartesian(x, id);
        auto sph = robot_utils::cartesianToSpherical(cart.head<3>());
        return Eigen::Vector4d(sph[robot_utils::RANGE], sph[robot_utils::AZIMUTH],
                               sph[robot_utils::ELEVATION], cart(3));
    }

    std::vector<Eigen::Vector4d> OutpostTarget::getResolvedArmors() const {
        std::vector<Eigen::Vector4d> armors;
        const auto x = ukfs_[best_ukf_idx_].getState();
        for (int i = 0; i < 3; ++i) { armors.push_back(getArmorCartesian(x, i)); }
        return armors;
    }

    Eigen::VectorXd OutpostTarget::getPredictedState(const rclcpp::Time &time) const {
        double dt = (time - last_time_).seconds();
        auto x = ukfs_[best_ukf_idx_].getState();
        if (dt > 0) {
            x(3) += x(4) * dt;
            x(3) = robot_utils::normalize_angle(x(3));
        }
        return x;
    }

    GeometricParams OutpostTarget::getGeometricParams() const {
        const auto x = ukfs_[best_ukf_idx_].getState();
        return {x(5), 0, x(6)};
    }
} // namespace robot_auto_aim
