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
    
    void init(const TrackerArmor& armor) override;

    void predict(const rclcpp::Time& time) override;

    bool update(const TrackerArmor& armor) override;

    void updateParams(double q_x, double q_y, double q_z, 
                      double q_vx, double q_vy, double q_vz,
                      double q_yaw, double q_v_yaw, double q_geo,
                      double r_x, double r_y, double r_z, double r_yaw, 
                      double r_yaw_adaptive_factor,
                      bool adaptive_tracking, double q_alpha,
                      double dist_scale_coeff, double z_scale_coeff) override {
        q_x_ = q_x; q_y_ = q_y; q_z_ = q_z;
        q_vx_ = q_vx; q_vy_ = q_vy; q_vz_ = q_vz;
        q_yaw_ = q_yaw; q_v_yaw_ = q_v_yaw; q_geo_ = q_geo;
        r_x_ = r_x; r_y_ = r_y; r_z_ = r_z;
        r_yaw_ = r_yaw;
        r_yaw_adaptive_factor_ = r_yaw_adaptive_factor;
        adaptive_tracking_ = adaptive_tracking;
        q_alpha_ = q_alpha;
        dist_scale_coeff_ = dist_scale_coeff;
        z_scale_coeff_ = z_scale_coeff;
    }

    void setUKFParams(double alpha, double beta, double kappa) override {
        Eigen::VectorXd current_x = ukf_.getState();
        Eigen::MatrixXd current_P = ukf_.getCovariance();
        ukf_ = robot_utils::UKF<STATE_DIM>(alpha, beta, kappa);
        if (current_x.size() == STATE_DIM) {
            ukf_.init(current_x, current_P);
        }
    }

    bool isConverged() const override;

    bool isDiverged() const override;

    // Getters
    Eigen::VectorXd getState() const override { return ukf_.getState(); }
    Eigen::MatrixXd getCovariance() const override { return ukf_.getCovariance(); }
    const std::string& getName() const override { return name_; }
    ArmorType getType() const override { return type_; }
    int getArmorNum() const override { return armor_num_; }
    double getPriority() const override { return priority_; }
    int getUpdateCount() const override { return update_count_; }

    std::vector<Eigen::Vector4d> getResolvedArmors() const override;
    
    Eigen::VectorXd getPredictedState(const rclcpp::Time& time) const override;

private:
    Eigen::Vector4d h(const Eigen::VectorXd& x, int id) const;

    robot_utils::UKF<STATE_DIM> ukf_;
    
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
    
    // Adaptive Q
    bool adaptive_tracking_;
    double q_alpha_;
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> Q_adaptive_;
};

} // namespace robot_auto_aim

#endif // ROBOT_AUTO_AIM__ROBOT_TARGET_HPP_
