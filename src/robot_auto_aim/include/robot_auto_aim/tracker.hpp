#ifndef ROBOT_AUTO_AIM__TRACKER_HPP_
#define ROBOT_AUTO_AIM__TRACKER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "robot_auto_aim/target.hpp"
#include "robot_auto_aim/types.hpp"

namespace robot_auto_aim {

class Tracker {
public:
    enum class State {
        LOST,
        DETECTING,
        TRACKING,
        TEMP_LOST
    };

    Tracker(rclcpp::Clock::SharedPtr clock, double max_lost_duration = 0.5, int min_detect_count = 3);

    /**
     * @brief Main tracking entry point
     * @param armors List of detected armors from detector
     * @return Best target if tracking, nullptr otherwise
     */
    std::shared_ptr<Target> track(const std::vector<TrackerArmor>& armors, const rclcpp::Time& time);

    std::shared_ptr<Target> getTarget() { return target_; }
    
    void handleTimeouts(const rclcpp::Time& time);

    State getState() const { return state_; }
    std::string getStateString() const;
    static std::string stateToString(State state);

    void updateParams(double q_x, double q_y, double q_z, 
                      double q_vx, double q_vy, double q_vz,
                      double q_yaw, double q_v_yaw, double q_geo,
                      double r_x, double r_y, double r_z, double r_yaw, 
                      double r_yaw_adaptive_factor,
                      bool adaptive_tracking, double q_alpha,
                      double dist_scale_coeff, double z_scale_coeff);
    
    void updateUKFParams(double alpha, double beta, double kappa);
    
    void reset();
    
    void setMaxLostDuration(double duration) { max_lost_duration_ = duration; }
    void setMinDetectCount(int count) { min_detect_count_ = count; }

private:
    void initTarget(const TrackerArmor& armor);
    bool updateTarget(const std::vector<TrackerArmor>& armors, const rclcpp::Time& time);
    
    State state_;
    std::shared_ptr<Target> target_;
    
    rclcpp::Clock::SharedPtr clock_;
    
    double max_lost_duration_;
    int min_detect_count_;
    
    rclcpp::Time last_seen_time_;
    int detect_count_;

    // Params
    double q_x_, q_y_, q_z_;
    double q_vx_, q_vy_, q_vz_;
    double q_yaw_, q_v_yaw_, q_geo_;
    double r_x_, r_y_, r_z_, r_yaw_, r_yaw_adaptive_factor_;
    double dist_scale_coeff_, z_scale_coeff_;
    bool adaptive_tracking_;
    double q_alpha_;
    double ukf_alpha_, ukf_beta_, ukf_kappa_;
};

} // namespace robot_auto_aim

#endif // ROBOT_AUTO_AIM__TRACKER_HPP_
