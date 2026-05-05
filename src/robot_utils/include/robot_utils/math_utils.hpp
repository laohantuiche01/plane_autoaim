#ifndef ROBOT_UTILS__MATH_UTILS_HPP_
#define ROBOT_UTILS__MATH_UTILS_HPP_

#include <cmath>
#include <algorithm>

#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include <opencv2/calib3d.hpp>
#include <tf2/LinearMath/Matrix3x3.h>

namespace robot_utils {

inline Eigen::Matrix3d tf2ToEigen(const tf2::Matrix3x3& tf2_matrix) {
    Eigen::Matrix3d res;
    res << tf2_matrix.getRow(0)[0], tf2_matrix.getRow(0)[1], tf2_matrix.getRow(0)[2],
           tf2_matrix.getRow(1)[0], tf2_matrix.getRow(1)[1], tf2_matrix.getRow(1)[2],
           tf2_matrix.getRow(2)[0], tf2_matrix.getRow(2)[1], tf2_matrix.getRow(2)[2];
    return res;
}

inline Eigen::Matrix3d cvToEigen(const cv::Mat& mat) {
    Eigen::Matrix3d res;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            res(i, j) = mat.at<double>(i, j);
    return res;
}

inline Eigen::Vector3d cvToEigenVec(const cv::Mat& mat) {
    Eigen::Vector3d res;
    res(0) = mat.at<double>(0);
    res(1) = mat.at<double>(1);
    res(2) = mat.at<double>(2);
    return res;
}

/**
 * @brief Clamp a value between a minimum and maximum range.
 */
template <typename T>
T clamp(T val, T min, T max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

/**
 * @brief Simple utility function example.
 */
double normalize_angle(double angle);

/**
 * @brief Unwrap a sequence of angles to remove 2pi discontinuities.
 */
void unwrap_angles(std::vector<double>& angles);

/**
 * @brief Convert degrees to radians.
 */
inline double deg_to_rad(double deg) {
    return deg * M_PI / 180.0;
}

/**
 * @brief Convert radians to degrees.
 */
inline double rad_to_deg(double rad) {
    return rad * 180.0 / M_PI;
}

// --- Spherical coordinate utilities ---

/// 球坐标分量索引
enum SphericalIdx : int { RANGE = 0, AZIMUTH = 1, ELEVATION = 2 };

/// Cartesian (x,y,z) → Spherical (range, azimuth, elevation)
/// azimuth ∈ [-π, π], elevation ∈ [-π/2, π/2]
inline Eigen::Vector3d cartesianToSpherical(const Eigen::Vector3d& p) {
    double range = p.norm();
    double azimuth = std::atan2(p.y(), p.x());
    double elevation = std::asin(std::clamp(p.z() / std::max(range, 1e-9), -1.0, 1.0));
    return {range, azimuth, elevation};
}

/// Spherical (range, azimuth, elevation) → Cartesian (x,y,z)
inline Eigen::Vector3d sphericalToCartesian(const Eigen::Vector3d& sph) {
    double cos_el = std::cos(sph[ELEVATION]);
    return {sph[RANGE] * cos_el * std::cos(sph[AZIMUTH]),
            sph[RANGE] * cos_el * std::sin(sph[AZIMUTH]),
            sph[RANGE] * std::sin(sph[ELEVATION])};
}

/// 计算装甲板法向与视线方向的夹角 [0, π/2]
/// position: 装甲板在 odom 系中的位置, orientation: 装甲板姿态四元数
inline double computeViewingAngle(const Eigen::Vector3d& position,
                                   const Eigen::Quaterniond& orientation) {
    Eigen::Vector3d normal = orientation.toRotationMatrix().col(0);
    return std::acos(std::clamp(std::abs(normal.dot(position.normalized())), 0.0, 1.0));
}

/**
 * @brief Reproject 3D trajectory points from odom frame to 2D image plane.
 *
 * Transforms odom-frame 3D points into camera_optical_frame via
 * p_cam = R_camera_odom * p_odom + t_camera_odom, filters points
 * behind the camera (z <= 0), then projects onto the image using
 * camera intrinsics and distortion model (radial + tangential).
 *
 * Projection model:
 *   p_cam  = R · p_odom + t                      // Euclidean transform
 *   u_norm = p_cam.head<2>() / p_cam.z()          // perspective division
 *   u_dist = u_norm + δ_radial(u_norm) + δ_tang(u_norm)   // OpenCV distortion
 *
 * @param points_odom       3D trajectory points in odom frame
 * @param R_camera_odom     3x3 rotation from odom to camera_optical_frame
 * @param t_camera_odom     3x1 translation from odom to camera_optical_frame
 * @param camera_matrix     3x3 intrinsic matrix K = [fx, 0, cx; 0, fy, cy; 0, 0, 1]
 * @param dist_coeffs       Distortion coefficients [k1, k2, p1, p2, k3, ...]
 * @return 2D image points in pixel coordinates. Points with z <= 0 after transform
 *         are filtered — the output vector may be shorter than the input.
 */
inline std::vector<cv::Point2f> projectTrajectoryToImage(
    const std::vector<Eigen::Vector3d>& points_odom,
    const Eigen::Matrix3d& R_camera_odom,
    const Eigen::Vector3d& t_camera_odom,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs)
{
    // Step 1: Transform all points from odom to camera_optical_frame
    std::vector<cv::Point3f> cam_points;
    for (const auto& pt : points_odom) {
        Eigen::Vector3d p_cam = R_camera_odom * pt + t_camera_odom;
        if (p_cam.z() > 0.0) {
            cam_points.emplace_back(p_cam.x(), p_cam.y(), p_cam.z());
        }
    }

    if (cam_points.empty()) return {};

    // Step 2: Perspective projection + distortion via OpenCV
    // rvec=tvec=zero since points are already in the camera frame
    std::vector<cv::Point2f> image_points;
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
    cv::projectPoints(cam_points, rvec, tvec, camera_matrix, dist_coeffs, image_points);

    return image_points;
}

}  // namespace robot_utils

#endif  // ROBOT_UTILS__MATH_UTILS_HPP_
