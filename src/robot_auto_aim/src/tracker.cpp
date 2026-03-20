#include "robot_auto_aim/tracker.hpp"
#include <algorithm>
#include <rclcpp/rclcpp.hpp>

#include "robot_auto_aim/robot_target.hpp"
#include "robot_auto_aim/outpost_target.hpp"

namespace robot_auto_aim {

Tracker::Tracker(rclcpp::Clock::SharedPtr clock, double max_lost_duration, int min_detect_count)
    : state_(State::LOST),
      clock_(clock),
      max_lost_duration_(max_lost_duration),
      min_detect_count_(min_detect_count),
      last_seen_time_(0),
      detect_count_(0),
      ukf_alpha_(0.001),
      ukf_beta_(2.0),
      ukf_kappa_(0.0) {
}

std::shared_ptr<TargetBase> Tracker::track(const std::vector<TrackerArmor>& armors, const rclcpp::Time& time) {
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
        bool converged = target_->isConverged();
        if (converged) {
            geo_cache_[target_->getName()] = target_->getGeometricParams();
        }
        RCLCPP_INFO_THROTTLE(logger, *clock_, 1000, 
            "Tracking target: %s, Converged: %s", 
            target_->getName().c_str(), converged ? "YES" : "NO");
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
    GeometricParams init_geo;
    if (geo_cache_.count(armor.number)) {
        init_geo = geo_cache_[armor.number];
        
        // Disambiguation for Robot targets (4 armors)
        if (armor.number != "outpost") {
            // If current armor type differs from cached ID 0 type, 
            // it means we are likely seeing the 'Odd' side (ID 1 or 3).
            if (init_geo.even_armor_type != ArmorType::INVALID && armor.type != init_geo.even_armor_type) {
                double old_r = init_geo.r;
                double old_l = init_geo.l;
                double old_h = init_geo.h;
                // 'Rotate' the geometry 90 degrees to make the current armor the new ID 0
                init_geo.r = old_r + old_l;
                init_geo.l = -old_l;
                init_geo.h = -old_h;
                // Note: init_geo.even_armor_type would now be armor.type, 
                // which will be updated naturally when it converges again.
            }
        }

        // Basic range validation to ensure cached params are sane
        if (armor.number == "outpost") {
            if (init_geo.r < 0.1 || init_geo.r > 0.5 || init_geo.h < 0.05 || init_geo.h > 0.2) {
                init_geo = GeometricParams(); 
            }
        } else {
            if (init_geo.r < 0.05 || init_geo.r > 0.6) {
                init_geo = GeometricParams(); 
            }
        }
    }

    if (armor.number == "outpost") {
        target_ = std::make_shared<OutpostTarget>();
        RCLCPP_INFO(rclcpp::get_logger("tracker"), "Initializing OutpostTarget.");
        target_->setUKFParams(ukf_alpha_, ukf_beta_, ukf_kappa_);
        target_->updateParams(outpost_params_);
    } else {
        target_ = std::make_shared<RobotTarget>();
        RCLCPP_INFO(rclcpp::get_logger("tracker"), "Initializing RobotTarget.");
        target_->setUKFParams(ukf_alpha_, ukf_beta_, ukf_kappa_);
        target_->updateParams(robot_params_);
    }
    target_->init(armor, init_geo);
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

void Tracker::updateParams(const TargetParams& robot_params, const TargetParams& outpost_params) {
    robot_params_ = robot_params;
    outpost_params_ = outpost_params;
    
    if (target_) {
        if (target_->getName() == "outpost") {
            target_->updateParams(outpost_params_);
        } else {
            target_->updateParams(robot_params_);
        }
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
