#ifndef ROBOT_AUTO_AIM__TARGET_HPP_
#define ROBOT_AUTO_AIM__TARGET_HPP_

#include <Eigen/Dense>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "robot_auto_aim/types.hpp"
#include "robot_utils/math_utils.hpp"
#include "robot_utils/ukf.hpp"

namespace robot_auto_aim {

struct TrackerArmor {
    std::string number;
    ArmorType type;
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation; // Rotation from camera to odom
    double yaw;
    double priority;
    rclcpp::Time timestamp;
};

class Target {
public:
    static constexpr int STATE_DIM = 11;
    static constexpr int MEAS_DIM = 4;

    Target();
    
    void init(const TrackerArmor& armor);

    void predict(const rclcpp::Time& time);

    bool update(const TrackerArmor& armor);

    void updateParams(double q_x, double q_y, double q_z, 
                      double q_vx, double q_vy, double q_vz,
                      double q_yaw, double q_v_yaw, double q_geo,
                      double r_x, double r_y, double r_z, double r_yaw, 
                      double r_yaw_adaptive_factor,
                      bool adaptive_tracking, double r_alpha,
                      double dist_scale_coeff, double z_scale_coeff) {
        q_x_ = q_x; q_y_ = q_y; q_z_ = q_z;
        q_vx_ = q_vx; q_vy_ = q_vy; q_vz_ = q_vz;
        q_yaw_ = q_yaw; q_v_yaw_ = q_v_yaw; q_geo_ = q_geo;
        r_x_ = r_x; r_y_ = r_y; r_z_ = r_z;
        r_yaw_ = r_yaw;
        r_yaw_adaptive_factor_ = r_yaw_adaptive_factor;
        adaptive_tracking_ = adaptive_tracking;
        r_alpha_ = r_alpha;
        dist_scale_coeff_ = dist_scale_coeff;
        z_scale_coeff_ = z_scale_coeff;
    }

    void setUKFParams(double alpha, double beta, double kappa) {
        Eigen::VectorXd current_x = ukf_.getState();
        Eigen::MatrixXd current_P = ukf_.getCovariance();
        ukf_ = robot_utils::UKF<STATE_DIM>(alpha, beta, kappa);
        if (current_x.size() == STATE_DIM) {
            ukf_.init(current_x, current_P);
        }
    }

    bool isConverged() const;

    bool isDiverged() const;

    // Getters
    Eigen::VectorXd getState() const { return ukf_.getState(); }
    Eigen::MatrixXd getCovariance() const { return ukf_.getCovariance(); }
    const std::string& getName() const { return name_; }
    ArmorType getType() const { return type_; }
    int getArmorNum() const { return armor_num_; }
    double getPriority() const { return priority_; }
    int getUpdateCount() const { return update_count_; }

    std::vector<Eigen::Vector4d> getResolvedArmors() const;
    
    Eigen::VectorXd getPredictedState(const rclcpp::Time& time) const;

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
    
    // Adaptive R
    bool adaptive_tracking_;
    double r_alpha_;
    Eigen::Matrix4d R_adaptive_;
};

} // namespace robot_auto_aim

#endif // ROBOT_AUTO_AIM__TARGET_HPP_
