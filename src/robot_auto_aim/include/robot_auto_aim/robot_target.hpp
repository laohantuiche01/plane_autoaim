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
        q_x_ = params.q_x; q_y_ = params.q_y; q_z_ = params.q_z;
        q_vx_ = params.q_vx; q_vy_ = params.q_vy; q_vz_ = params.q_vz;
        q_yaw_ = params.q_yaw; q_v_yaw_ = params.q_v_yaw; q_geo_ = params.q_geo;
        r_x_ = params.r_x; r_y_ = params.r_y; r_z_ = params.r_z;
        r_yaw_ = params.r_yaw;
        r_yaw_adaptive_factor_ = params.r_yaw_adaptive_factor;
        adaptive_tracking_ = params.adaptive_tracking;
        q_alpha_ = params.q_alpha;
        dist_scale_coeff_ = params.dist_scale_coeff;
        z_scale_coeff_ = params.z_scale_coeff;
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
    int last_armor_id_;
    int update_count_;

    // Params
    double q_x_, q_y_, q_z_;
    double q_vx_, q_vy_, q_vz_;
    double q_yaw_, q_v_yaw_, q_geo_;
    double r_x_, r_y_, r_z_, r_yaw_, r_yaw_adaptive_factor_;
    double dist_scale_coeff_, z_scale_coeff_;
    
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
