#ifndef ROBOT_UTILS__MATH_UTILS_HPP_
#define ROBOT_UTILS__MATH_UTILS_HPP_

#include <cmath>

namespace robot_utils {

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
