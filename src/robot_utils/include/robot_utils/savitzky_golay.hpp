#ifndef ROBOT_UTILS__SAVITZKY_GOLAY_HPP_
#define ROBOT_UTILS__SAVITZKY_GOLAY_HPP_

#include <vector>
#include <Eigen/Dense>

namespace robot_utils {

class SavitzkyGolayFilter {
public:
    /**
     * @brief Construct a new Polynomial Filter object
     * 
     * @param window_size Number of data points in the window
     * @param poly_order Order of the polynomial to fit
     */
    SavitzkyGolayFilter(int window_size, int poly_order);
    
    /**
     * @brief Perform a least-squares polynomial fit on the given time-value pairs 
     *        and return the value and its first derivative at the target time.
     * 
     * @param times Vector of time offsets for each data point
     * @param values Vector of data values
     * @param target_time The time at which to evaluate the fit (usually 0 for center)
     * @return std::pair<double, double> {value, first_derivative}
     */
    std::pair<double, double> fit(const std::vector<double>& times, const std::vector<double>& values, double target_time = 0.0) const;

private:
    int window_size_;
    int poly_order_;
};

} // namespace robot_utils

#endif // ROBOT_UTILS__SAVITZKY_GOLAY_HPP_
