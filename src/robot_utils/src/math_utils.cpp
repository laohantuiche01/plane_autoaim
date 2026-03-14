#include "robot_utils/math_utils.hpp"

namespace robot_utils {

double normalize_angle(double angle) {
    angle = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (angle < 0) {
        angle += 2.0 * M_PI;
    }
    return angle - M_PI;
}

}  // namespace robot_utils
