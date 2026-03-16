#ifndef ROBOT_BALLISTICS__BALLISTICS_CALCULATOR_HPP_
#define ROBOT_BALLISTICS__BALLISTICS_CALCULATOR_HPP_

#include <Eigen/Dense>

namespace robot_ballistics {

class BallisticsCalculator {
public:
    BallisticsCalculator() = default;
    ~BallisticsCalculator() = default;

    /**
     * @brief Calculate predicted time of flight and additional delays.
     * 
     * @param target_pos The current position of the target relative to the shooter
     * @param bullet_speed The speed of the projectile (m/s)
     * @param pipeline_latency The dynamically measured latency from image capture to now
     * @param manual_offset Manual offset to add/subtract to compensate for unmodeled delays
     * @return double Total forward prediction time (seconds)
     */
    static double calculatePredictTime(
        const Eigen::Vector3d& target_pos, 
        double bullet_speed, 
        double pipeline_latency, 
        double manual_offset);

    /**
     * @brief Calculate the required pitch angle to hit a target considering gravity.
     * 
     * @param target_pos Position of the target in gimbal frame (m)
     * @param bullet_speed Speed of the projectile (m/s)
     * @return double Required pitch angle (rad)
     */
    static double calculatePitch(const Eigen::Vector3d& target_pos, double bullet_speed);
};

} // namespace robot_ballistics

#endif // ROBOT_BALLISTICS__BALLISTICS_CALCULATOR_HPP_
