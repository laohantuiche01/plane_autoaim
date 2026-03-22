#ifndef ROBOT_AUTO_AIM__ROBOT_TARGET_HPP_
#define ROBOT_AUTO_AIM__ROBOT_TARGET_HPP_

#include "robot_auto_aim/target_base.hpp"
#include "robot_utils/math_utils.hpp"

namespace robot_auto_aim {

class RobotTarget : public TargetBase {
public:
    static constexpr int STATE_DIM = 11;
    static constexpr int MEAS_DIM = 4;

    RobotTarget();
    
    void init(const TrackerArmor& armor, const GeometricParams& init_geo = GeometricParams()) override;

    void predict(const rclcpp::Time& time) override;

    bool update(const TrackerArmor& armor) override;

    void updateParams(const TargetParams& params) override {
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
        p0_vel_ = params.p0_vel;
        p0_yaw_ = params.p0_yaw;
        p0_omega_ = params.p0_omega;
        p0_geo_ = params.p0_geo;
        min_update_count_ = params.min_update_count;
        max_pos_cov_ = params.max_pos_cov;
        max_yaw_cov_ = params.max_yaw_cov;
    }

    void setUKFParams(double alpha, double beta, double kappa) override {
        for (int i = 0; i < 2; ++i) {
            Eigen::VectorXd current_x = ukfs_[i].getState();
            Eigen::MatrixXd current_P = ukfs_[i].getCovariance();
            ukfs_[i] = robot_utils::UKF<STATE_DIM>(alpha, beta, kappa);
            if (current_x.size() == STATE_DIM) {
                ukfs_[i].init(current_x, current_P);
            }
        }
    }

    bool isConverged() const override;

    bool isDiverged() const override;

    // Getters
    Eigen::VectorXd getState() const override { return ukfs_[best_ukf_idx_].getState(); }
    Eigen::VectorXd getMeasurement() const override { return measurement_; }
    Eigen::MatrixXd getCovariance() const override { return ukfs_[best_ukf_idx_].getCovariance(); }
    const std::string& getName() const override { return name_; }
    ArmorType getType() const override { return type_; }
    int getArmorNum() const override { return armor_num_; }
    double getPriority() const override { return priority_; }
    int getUpdateCount() const override { return update_count_; }

    std::vector<Eigen::Vector4d> getResolvedArmors() const override;
    
    Eigen::VectorXd getPredictedState(const rclcpp::Time& time) const override;

    GeometricParams getGeometricParams() const override;

private:
    enum class ConfirmationState {
        CONFIRMING,
        CONFIRMED
    };

    // Cartesian armor position — for matching and getResolvedArmors()
    Eigen::Vector4d getArmorCartesian(const Eigen::VectorXd& x, int id) const;
    // Spherical observation — for UKF update
    Eigen::Vector4d h(const Eigen::VectorXd& x, int id) const;

    robot_utils::UKF<STATE_DIM> ukfs_[2];
    Eigen::VectorXd measurement_;
    double accumulated_errors_[2];
    int best_ukf_idx_;
    ConfirmationState confirmation_state_;
    int armor_switch_count_;
    
    std::string name_;
    ArmorType type_;
    int armor_num_;
    double priority_;
    
    rclcpp::Time last_time_;
    int last_armor_ids_[2];  // Per-UKF armor ID tracking
    int update_count_;

    // CWNA process noise parameters
    double sigma_pos_;   // Linear acceleration std dev (m/s²)
    double sigma_yaw_;   // Angular acceleration std dev (rad/s²)
    double q_geo_;       // Geometric parameter random walk noise

    // Spherical R parameters
    double r_range_, r_range_k_;     // Range noise: r_range * (1 + r_range_k * range²)
    double r_angle_;                 // Azimuth/elevation noise (constant)
    double r_yaw_, r_yaw_adaptive_factor_, r_yaw_viewing_k_;

    // Outlier rejection & initial covariance
    double mahalanobis_thresh_;
    double p0_pos_, p0_vel_, p0_yaw_, p0_omega_, p0_geo_;
    
    // Convergence
    int min_update_count_;
    double max_pos_cov_, max_yaw_cov_;
    
    // Adaptive Q
    bool adaptive_tracking_;
    double q_alpha_;
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> Q_adaptive_;
};

} // namespace robot_auto_aim

#endif // ROBOT_AUTO_AIM__ROBOT_TARGET_HPP_
