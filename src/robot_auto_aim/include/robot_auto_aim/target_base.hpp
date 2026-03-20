#ifndef ROBOT_AUTO_AIM__TARGET_BASE_HPP_
#define ROBOT_AUTO_AIM__TARGET_BASE_HPP_

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "robot_auto_aim/types.hpp"
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

struct TargetParams {
    double q_x, q_y, q_z;
    double q_vx, q_vy, q_vz;
    double q_yaw, q_v_yaw, q_geo;
    double r_x, r_y, r_z, r_yaw, r_yaw_adaptive_factor;
    bool adaptive_tracking;
    double q_alpha;
    double dist_scale_coeff, z_scale_coeff;

    // Convergence Params
    int min_update_count;
    double max_pos_cov;
    double max_yaw_cov;
};

class TargetBase {
public:
    virtual ~TargetBase() = default;

    virtual void init(const TrackerArmor& armor) = 0;
    virtual void predict(const rclcpp::Time& time) = 0;
    virtual bool update(const TrackerArmor& armor) = 0;

    virtual void updateParams(const TargetParams& params) = 0;
    
    virtual void setUKFParams(double alpha, double beta, double kappa) = 0;

    virtual bool isConverged() const = 0;
    virtual bool isDiverged() const = 0;

    // Getters
    virtual Eigen::VectorXd getState() const = 0;
    virtual Eigen::MatrixXd getCovariance() const = 0;
    virtual const std::string& getName() const = 0;
    virtual ArmorType getType() const = 0;
    virtual int getArmorNum() const = 0;
    virtual double getPriority() const = 0;
    virtual int getUpdateCount() const = 0;

    virtual std::vector<Eigen::Vector4d> getResolvedArmors() const = 0;
    virtual Eigen::VectorXd getPredictedState(const rclcpp::Time& time) const = 0;
};

} // namespace robot_auto_aim

#endif // ROBOT_AUTO_AIM__TARGET_BASE_HPP_
