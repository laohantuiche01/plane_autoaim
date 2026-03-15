#include "robot_auto_aim/tracker.hpp"
#include <algorithm>
#include <rclcpp/rclcpp.hpp>

namespace robot_auto_aim {

Tracker::Tracker(rclcpp::Clock::SharedPtr clock, double max_lost_duration, int min_detect_count)
    : state_(State::LOST),
      clock_(clock),
      max_lost_duration_(max_lost_duration),
      min_detect_count_(min_detect_count),
      last_seen_time_(0),
      detect_count_(0),
      adaptive_tracking_(false),
      r_alpha_(0.1),
      ukf_alpha_(0.001),
      ukf_beta_(2.0),
      ukf_kappa_(0.0) {
}

std::shared_ptr<Target> Tracker::track(const std::vector<TrackerArmor>& armors, const rclcpp::Time& time) {
    auto logger = rclcpp::get_logger("tracker");
    State last_state = state_;

    if (state_ == State::LOST) {
        if (!armors.empty()) {
            auto best_armor = armors[0];
            initTarget(best_armor);
            state_ = State::DETECTING;
            detect_count_ = 1;
            last_seen_time_ = time;
            RCLCPP_INFO(logger, "Target found! Number: %s", best_armor.number.c_str());
        }
    } else {
        bool found = updateTarget(armors, time);
        
        if (found) {
            last_seen_time_ = time;
            if (state_ == State::DETECTING) {
                detect_count_++;
                if (detect_count_ >= min_detect_count_) {
                    state_ = State::TRACKING;
                }
            } else if (state_ == State::TEMP_LOST) {
                state_ = State::TRACKING;
            }
        }
    }
    
    if (target_ && target_->isDiverged()) {
        RCLCPP_ERROR(logger, "Filter diverged! Resetting tracker.");
        state_ = State::LOST;
        target_ = nullptr;
    }

    if (state_ != last_state) {
        bool is_temp_lost_transition = (last_state == State::TRACKING && state_ == State::TEMP_LOST) ||
                                       (last_state == State::TEMP_LOST && state_ == State::TRACKING);
        if (!is_temp_lost_transition) {
            RCLCPP_INFO(logger, "State changed from %s to %s", 
                stateToString(last_state).c_str(), stateToString(state_).c_str());
        }
    }

    if ((state_ == State::TRACKING || state_ == State::TEMP_LOST) && target_) {
        RCLCPP_INFO_THROTTLE(logger, *clock_, 1000, 
            "Tracking target: %s, Converged: %s", 
            target_->getName().c_str(), target_->isConverged() ? "YES" : "NO");
    }

    return (state_ == State::TRACKING || state_ == State::TEMP_LOST) ? target_ : nullptr;
}

void Tracker::reset() {
    state_ = State::LOST;
    target_ = nullptr;
    detect_count_ = 0;
    last_seen_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

void Tracker::handleTimeouts(const rclcpp::Time& time) {
    if (state_ == State::LOST || !target_) return;

    double lost_duration = (time - last_seen_time_).seconds();
    
    if (lost_duration > max_lost_duration_) {
        RCLCPP_ERROR(rclcpp::get_logger("tracker"), "Target lost permanently after %.3fs.", lost_duration);
        state_ = State::LOST;
        target_ = nullptr;
    } else if (lost_duration > 0.1 && state_ == State::TRACKING) {
        state_ = State::TEMP_LOST;
    }
}

void Tracker::initTarget(const TrackerArmor& armor) {
    target_ = std::make_shared<Target>();
    target_->setUKFParams(ukf_alpha_, ukf_beta_, ukf_kappa_);
    target_->updateParams(q_x_, q_y_, q_z_, q_vx_, q_vy_, q_vz_, q_yaw_, q_v_yaw_, q_geo_, r_x_, r_y_, r_z_, r_yaw_, r_yaw_adaptive_factor_, adaptive_tracking_, r_alpha_, dist_scale_coeff_, z_scale_coeff_);
    target_->init(armor);
}

bool Tracker::updateTarget(const std::vector<TrackerArmor>& armors, const rclcpp::Time& time) {
    target_->predict(time);
    
    if (armors.empty()) return false;
    
    bool found = false;
    for (const auto& armor : armors) {
        if (armor.number == target_->getName()) {
            if (target_->update(armor)) {
                found = true;
                break;
            }
        }
    }
    return found;
}

void Tracker::updateParams(double q_x, double q_y, double q_z, 
                          double q_vx, double q_vy, double q_vz,
                          double q_yaw, double q_v_yaw, double q_geo,
                          double r_x, double r_y, double r_z, double r_yaw, 
                          double r_yaw_adaptive_factor,
                          bool adaptive_tracking, double r_alpha,
                          double dist_scale_coeff, double z_scale_coeff) {
    q_x_ = q_x; q_y_ = q_y; q_z_ = q_z;
    q_vx_ = q_vx; q_vy_ = q_vy; q_vz_ = q_vz;
    q_yaw_ = q_yaw; q_v_yaw_ = q_v_yaw; q_geo_ = q_geo;
    r_x_ = r_x; r_y_ = r_y; r_z_ = r_z;
    r_yaw_ = r_yaw;
    r_yaw_adaptive_factor_ = r_yaw_adaptive_factor;
    adaptive_tracking_ = adaptive_tracking;
    r_alpha_ = r_alpha;
    dist_scale_coeff_ = dist_scale_coeff;
    z_scale_coeff_ = z_scale_coeff;
    
    if (target_) {
        target_->updateParams(q_x, q_y, q_z, q_vx, q_vy, q_vz, q_yaw, q_v_yaw, q_geo, r_x, r_y, r_z, r_yaw, r_yaw_adaptive_factor, adaptive_tracking, r_alpha, dist_scale_coeff, z_scale_coeff);
    }
}

void Tracker::updateUKFParams(double alpha, double beta, double kappa) {
    ukf_alpha_ = alpha;
    ukf_beta_ = beta;
    ukf_kappa_ = kappa;
    if (target_) {
        target_->setUKFParams(alpha, beta, kappa);
    }
}

std::string Tracker::getStateString() const {
    return stateToString(state_);
}

std::string Tracker::stateToString(State state) {
    switch (state) {
        case State::LOST: return "LOST";
        case State::DETECTING: return "DETECTING";
        case State::TRACKING: return "TRACKING";
        case State::TEMP_LOST: return "TEMP_LOST";
        default: return "UNKNOWN";
    }
}

} // namespace robot_auto_aim
