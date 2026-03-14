#ifndef ROBOT_UTILS__PNP_SOLVER_HPP_
#define ROBOT_UTILS__PNP_SOLVER_HPP_

#include <array>
#include <string>
#include <unordered_map>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

namespace robot_utils {

class PnPSolver {
public:
    PnPSolver(const std::array<double, 9>& camera_matrix, const std::vector<double>& dist_coeffs);
    void setObjectPoints(const std::string& name, const std::vector<cv::Point3f>& points);
    bool solvePnPGeneric(const std::vector<cv::Point2f>& image_points, std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs, const std::string& name = "small");
    double calculateDistanceToCenter(const cv::Point2f& image_point);
    double calculateReprojectionError(const std::vector<cv::Point2f>& image_points, const cv::Mat& rvec, const cv::Mat& tvec, const std::string& name = "small");

private:
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    std::unordered_map<std::string, std::vector<cv::Point3f>> object_points_;
};

} // namespace robot_utils

#endif // ROBOT_UTILS__PNP_SOLVER_HPP_