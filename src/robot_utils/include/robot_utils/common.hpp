#ifndef ROBOT_UTILS__COMMON_HPP_
#define ROBOT_UTILS__COMMON_HPP_

#include <string>

namespace robot_utils {

enum class EnemyColor { RED, BLUE, WHITE };

inline std::string enemyColorToString(EnemyColor color) {
    switch (color) {
        case EnemyColor::RED: return "RED";
        case EnemyColor::BLUE: return "BLUE";
        case EnemyColor::WHITE: return "WHITE";
        default: return "UNKNOWN";
    }
}

enum class VisionMode { AUTO_AIM_RED, AUTO_AIM_BLUE, UNKNOWN };

inline std::string visionModeToString(VisionMode mode) {
    switch (mode) {
        case VisionMode::AUTO_AIM_RED: return "AUTO_AIM_RED";
        case VisionMode::AUTO_AIM_BLUE: return "AUTO_AIM_BLUE";
        default: return "UNKNOWN";
    }
}

} // namespace robot_utils

#endif // ROBOT_UTILS__COMMON_HPP_