#ifndef ROBOT_AUTO_AIM__TRACKER_HPP_
#define ROBOT_AUTO_AIM__TRACKER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "robot_auto_aim/target_base.hpp"
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
    std::shared_ptr<TargetBase> track(const std::vector<TrackerArmor>& armors, const rclcpp::Time& time);

    std::shared_ptr<TargetBase> getTarget() { return target_; }
    
    void handleTimeouts(const rclcpp::Time& time);

    State getState() const { return state_; }
    std::string getStateString() const;
    static std::string stateToString(State state);

    void updateParams(const TargetParams& robot_params, const TargetParams& outpost_params);
    
    void updateUKFParams(double alpha, double beta, double kappa);
    
    void reset();
    
    void setMaxLostDuration(double duration) { max_lost_duration_ = duration; }
    void setMinDetectCount(int count) { min_detect_count_ = count; }

private:
    void initTarget(const TrackerArmor& armor);
    bool updateTarget(const std::vector<TrackerArmor>& armors, const rclcpp::Time& time);
    
    State state_;
    std::shared_ptr<TargetBase> target_;
    
    rclcpp::Clock::SharedPtr clock_;
    
    double max_lost_duration_;
    int min_detect_count_;
    
    rclcpp::Time last_seen_time_;
    int detect_count_;

    // Params
    TargetParams robot_params_;
    TargetParams outpost_params_;
    double ukf_alpha_, ukf_beta_, ukf_kappa_;
};

} // namespace robot_auto_aim

#endif // ROBOT_AUTO_AIM__TRACKER_HPP_
