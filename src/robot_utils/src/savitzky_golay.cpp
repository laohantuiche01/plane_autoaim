#include "robot_utils/savitzky_golay.hpp"
#include <cmath>
#include <stdexcept>

namespace robot_utils {

SavitzkyGolayFilter::SavitzkyGolayFilter(int window_size, int poly_order, int deriv_order, double dt)
    : window_size_(window_size), poly_order_(poly_order), deriv_order_(deriv_order), dt_(dt) {
    
    if (window_size_ % 2 == 0 || window_size_ < 3) {
        throw std::invalid_argument("Window size must be an odd number >= 3.");
    }
    if (poly_order_ >= window_size_) {
        throw std::invalid_argument("Polynomial order must be less than window size.");
    }
    
    int m = window_size_ / 2;
    Eigen::MatrixXd X(window_size_, poly_order_ + 1);
    for (int i = -m; i <= m; ++i) {
        for (int j = 0; j <= poly_order_; ++j) {
            X(i + m, j) = std::pow(i, j);
        }
    }
    
    Eigen::MatrixXd XT_X_inv_XT = (X.transpose() * X).inverse() * X.transpose();
    coeffs_ = XT_X_inv_XT.row(deriv_order_);
    
    double factorial = 1.0;
    for (int i = 1; i <= deriv_order_; ++i) {
        factorial *= i;
    }
    if (deriv_order_ > 0) {
        coeffs_ *= (factorial / std::pow(dt_, deriv_order_));
    }
}

double SavitzkyGolayFilter::filterCenter(const std::vector<double>& data) const {
    if (data.size() != static_cast<size_t>(window_size_)) {
        throw std::invalid_argument("Data size must equal window size.");
    }
    double result = 0.0;
    for (int i = 0; i < window_size_; ++i) {
        result += coeffs_(i) * data[i];
    }
    return result;
}

} // namespace robot_utils
