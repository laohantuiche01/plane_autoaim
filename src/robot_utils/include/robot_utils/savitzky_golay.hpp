#ifndef ROBOT_UTILS__SAVITZKY_GOLAY_HPP_
#define ROBOT_UTILS__SAVITZKY_GOLAY_HPP_

#include <vector>
#include <Eigen/Dense>

namespace robot_utils {

class SavitzkyGolayFilter {
public:
    /**
     * @brief Construct a new Savitzky Golay Filter object
     * 
     * @param window_size Number of data points in the smoothing window (must be odd)
     * @param poly_order Order of the polynomial to fit (must be less than window_size)
     * @param deriv_order Order of the derivative to compute (0 for smoothed value, 1 for first derivative, etc.)
     * @param dt Time step between data points (for scaling derivatives)
     */
    SavitzkyGolayFilter(int window_size, int poly_order, int deriv_order = 0, double dt = 1.0);
    
    /**
     * @brief Compute the filtered value or derivative for the center of the given data window.
     * 
     * @param data Vector of data points. Its size must equal window_size.
     * @return double The filtered value or derivative evaluated at the center of the window.
     */
    double filterCenter(const std::vector<double>& data) const;

private:
    int window_size_;
    int poly_order_;
    int deriv_order_;
    double dt_;
    Eigen::VectorXd coeffs_;
};

} // namespace robot_utils

#endif // ROBOT_UTILS__SAVITZKY_GOLAY_HPP_
