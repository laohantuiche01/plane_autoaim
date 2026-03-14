#include "robot_utils/pnp_solver.hpp"

namespace robot_utils {

PnPSolver::PnPSolver(const std::array<double, 9>& camera_matrix, const std::vector<double>& dist_coeffs) {
    camera_matrix_ = cv::Mat(3, 3, CV_64F);
    for (int i = 0; i < 9; ++i) camera_matrix_.at<double>(i / 3, i % 3) = camera_matrix[i];
    dist_coeffs_ = cv::Mat(dist_coeffs).clone();
}

void PnPSolver::setObjectPoints(const std::string& name, const std::vector<cv::Point3f>& points) {
    object_points_[name] = points;
}

bool PnPSolver::solvePnPGeneric(const std::vector<cv::Point2f>& image_points, std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs, const std::string& name) {
    if (object_points_.find(name) == object_points_.end()) return false;
    cv::solvePnPGeneric(object_points_[name], image_points, camera_matrix_, dist_coeffs_, rvecs, tvecs, false, cv::SOLVEPNP_IPPE);
    return rvecs.size() > 0;
}

double PnPSolver::calculateDistanceToCenter(const cv::Point2f& image_point) {
    cv::Point2f center(camera_matrix_.at<double>(0, 2), camera_matrix_.at<double>(1, 2));
    return cv::norm(image_point - center);
}

double PnPSolver::calculateReprojectionError(const std::vector<cv::Point2f>& image_points, const cv::Mat& rvec, const cv::Mat& tvec, const std::string& name) {
    if (object_points_.find(name) == object_points_.end()) return 1e9;
    std::vector<cv::Point2f> projected_points;
    cv::projectPoints(object_points_[name], rvec, tvec, camera_matrix_, dist_coeffs_, projected_points);
    double err = 0.0;
    for (size_t i = 0; i < image_points.size(); ++i) {
        err += cv::norm(image_points[i] - projected_points[i]);
    }
    return err / image_points.size();
}

} // namespace robot_utils