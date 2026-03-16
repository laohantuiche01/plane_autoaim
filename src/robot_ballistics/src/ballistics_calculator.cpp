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

double BallisticsCalculator::calculatePitch(const Eigen::Vector3d& target_pos, double bullet_speed) {
    double x = target_pos.x();
    double y = target_pos.y();
    double z = target_pos.z();
    double d = std::sqrt(x * x + y * y);
    double v = bullet_speed;
    double g = 9.8;

    // z = d*u - (g*d^2)/(2*v^2) * (1 + u^2) where u = tan(pitch)
    // 0 = (g*d^2)/(2*v^2) * u^2 - d * u + [z + (g*d^2)/(2*v^2)]
    double a = (g * d * d) / (2 * v * v);
    double b = -d;
    double c = z + a;
    double discriminant = b * b - 4 * a * c;

    if (discriminant >= 0) {
        double u = (-b - std::sqrt(discriminant)) / (2 * a);
        return std::atan(u);
    } else {
        // Fallback to linear pitch if target is unreachable
        return std::atan2(z, d);
    }
}

} // namespace robot_ballistics
