#include "robot_utils/math_utils.hpp"

namespace robot_utils {

double normalize_angle(double angle) {
    angle = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (angle < 0) {
        angle += 2.0 * M_PI;
    }
    return angle - M_PI;
}

void unwrap_angles(std::vector<double>& angles) {
    if (angles.empty()) return;
    for (size_t i = 1; i < angles.size(); ++i) {
        double diff = angles[i] - angles[i - 1];
        if (diff > M_PI) {
            while (angles[i] - angles[i - 1] > M_PI) angles[i] -= 2.0 * M_PI;
        } else if (diff < -M_PI) {
            while (angles[i] - angles[i - 1] < -M_PI) angles[i] += 2.0 * M_PI;
        }
    }
}

}  // namespace robot_utils
