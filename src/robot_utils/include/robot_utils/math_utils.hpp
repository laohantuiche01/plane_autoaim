#ifndef ROBOT_UTILS__MATH_UTILS_HPP_
#define ROBOT_UTILS__MATH_UTILS_HPP_

#include <cmath>

#include <opencv2/core.hpp>
#include <Eigen/Dense>

namespace robot_utils {

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

}  // namespace robot_utils

#endif  // ROBOT_UTILS__MATH_UTILS_HPP_
