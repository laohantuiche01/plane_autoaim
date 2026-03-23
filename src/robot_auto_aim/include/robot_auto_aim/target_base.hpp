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
    double viewing_angle;  // Angle between armor normal and line-of-sight [0, π/2]
    rclcpp::Time timestamp;
};

struct TargetParams {
    // Process noise - CWNA model (Continuous White Noise Acceleration)
    // RobotTarget: acceleration std dev (m/s²), generates cross-coupled Q for pos-vel pairs
    // OutpostTarget: position drift std dev for random walk (no velocity states for position)
    double sigma_pos;
    // Angular acceleration std dev (rad/s²), CWNA for yaw-omega pair
    double sigma_yaw;
    // Geometric parameter noise (r, l, h), simple random walk
    double q_geo;

    // Spherical measurement noise (R matrix)
    double r_range;              // Base range noise variance (m²)
    double r_range_k;            // Range noise growth: R_range = r_range * (1 + r_range_k * range²)
    double r_angle;              // Azimuth/elevation noise variance (rad², constant)
    double r_yaw;                // Base yaw noise variance (rad²)
    double r_yaw_adaptive_factor; // Yaw noise multiplier on armor ID switch
    double r_yaw_viewing_k;      // Yaw noise amplification at head-on: R_yaw *= (1 + k * cos²(va))

    // Adaptive Q (Sage-Husa)
    bool adaptive_tracking;
    double q_alpha;

    // Outlier rejection
    double mahalanobis_thresh;   // Chi-square threshold for Mahalanobis gating (4 DoF: 9.49=p0.05, 15.0=p0.005)

    // Initial covariance P0
    double p0_pos;               // Position initial variance
    double p0_vel;               // Velocity initial variance (RobotTarget only)
    double p0_yaw;               // Yaw initial variance
    double p0_omega;             // Angular velocity initial variance
    double p0_geo;               // Geometric parameters (r, l, h) initial variance

    // Convergence
    int min_update_count;
    double max_pos_cov;
    double max_yaw_cov;

    // Observability protection
    double omega_freeze_thresh;  // |omega| below this → scale down q_geo to freeze geometry params (rad/s)
};

struct GeometricParams {
    double r = 0;
    double l = 0;
    double h = 0;
    ArmorType even_armor_type = ArmorType::INVALID;
};

class TargetBase {
public:
    virtual ~TargetBase() = default;

    virtual void init(const TrackerArmor& armor, const GeometricParams& init_geo = GeometricParams()) = 0;
    virtual void predict(const rclcpp::Time& time) = 0;
    virtual bool update(const TrackerArmor& armor) = 0;

    virtual void updateParams(const TargetParams& params) = 0;
    
    virtual void setUKFParams(double alpha, double beta, double kappa) = 0;

    virtual bool isConverged() const = 0;
    virtual bool isDiverged() const = 0;

    // Getters
    virtual Eigen::VectorXd getState() const = 0;
    virtual Eigen::VectorXd getMeasurement() const = 0;
    virtual Eigen::MatrixXd getCovariance() const = 0;
    virtual const std::string& getName() const = 0;
    virtual ArmorType getType() const = 0;
    virtual int getArmorNum() const = 0;
    virtual double getPriority() const = 0;
    virtual int getUpdateCount() const = 0;

    virtual std::vector<Eigen::Vector4d> getResolvedArmors() const = 0;
    virtual Eigen::VectorXd getPredictedState(const rclcpp::Time& time) const = 0;
    virtual GeometricParams getGeometricParams() const = 0;
};

} // namespace robot_auto_aim

#endif // ROBOT_AUTO_AIM__TARGET_BASE_HPP_
