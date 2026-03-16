#include "robot_utils/savitzky_golay.hpp"
#include <cmath>
#include <stdexcept>
#include <Eigen/Dense>

namespace robot_utils {

SavitzkyGolayFilter::SavitzkyGolayFilter(int window_size, int poly_order)
    : window_size_(window_size), poly_order_(poly_order) {
    if (window_size_ < 3) {
        throw std::invalid_argument("Window size must be at least 3.");
    }
    if (poly_order_ >= window_size_) {
        throw std::invalid_argument("Polynomial order must be less than window size.");
    }
}

std::pair<double, double> SavitzkyGolayFilter::fit(const std::vector<double>& times, const std::vector<double>& values, double target_time) const {
    if (times.size() != static_cast<size_t>(window_size_) || values.size() != static_cast<size_t>(window_size_)) {
        throw std::invalid_argument("Input vector sizes must match window size.");
    }

    // Construct Vandermonde matrix X: X_ij = (t_i - target_time)^j
    Eigen::MatrixXd X(window_size_, poly_order_ + 1);
    Eigen::VectorXd Y(window_size_);

    for (int i = 0; i < window_size_; ++i) {
        double t = times[i] - target_time;
        Y(i) = values[i];
        for (int j = 0; j <= poly_order_; ++j) {
            X(i, j) = std::pow(t, j);
        }
    }

    // Solve least squares: (X^T * X) * beta = X^T * Y
    // Use ColPivHouseholderQR for robustness
    Eigen::VectorXd beta = X.colPivHouseholderQr().solve(Y);

    // beta_0 is the value at target_time, beta_1 is the first derivative
    double value = beta(0);
    double derivative = (poly_order_ >= 1) ? beta(1) : 0.0;

    return {value, derivative};
}

} // namespace robot_utils
