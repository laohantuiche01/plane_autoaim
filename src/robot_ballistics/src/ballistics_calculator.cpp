#include "robot_ballistics/ballistics_calculator.hpp"

namespace robot_ballistics {

double BallisticsCalculator::calculatePredictTime(
    const Eigen::Vector3d& target_pos, 
    double bullet_speed, 
    double pipeline_latency, 
    double manual_offset) {
    
    // Safety check
    if (bullet_speed <= 0.0) {
        return pipeline_latency + manual_offset;
    }

    double distance = target_pos.norm();
    
    // Simplest Time Of Flight calculation
    // TODO: This can be expanded to use iterative calculation with air resistance if needed
    double time_of_flight = distance / bullet_speed;

    return time_of_flight + pipeline_latency + manual_offset;
}

} // namespace robot_ballistics
