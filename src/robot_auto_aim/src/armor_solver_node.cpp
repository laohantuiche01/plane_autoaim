#include "robot_auto_aim/armor_solver_node.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "rclcpp_components/register_node_macro.hpp"
#include "lifecycle_msgs/msg/state.hpp"

namespace robot_auto_aim {

ArmorSolverNode::ArmorSolverNode(const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("armor_solver", options) {
}

ArmorSolverNode::CallbackReturn ArmorSolverNode::on_configure(const rclcpp_lifecycle::State& /*state*/) {
    RCLCPP_INFO(get_logger(), "Configuring ArmorSolverNode...");

    debug_ = declare_parameter("debug", true);
    debug_img_freq_ = declare_parameter("debug_img_freq", 60.0);
    last_debug_img_time_ = this->now();
    odom_frame_ = declare_parameter("odom_frame", "odom");
    double max_lost_duration = declare_parameter("max_lost_duration", 1.0);
    int min_detect_count = declare_parameter("min_detect_count", 5);

    auto declare_target_params = [this](const std::string& prefix, TargetParams& params, bool has_velocity) {
        params.q_x = declare_parameter(prefix + ".q_x", 0.1);
        params.q_y = declare_parameter(prefix + ".q_y", 0.1);
        params.q_z = declare_parameter(prefix + ".q_z", 0.02);
        if (has_velocity) {
            params.q_vx = declare_parameter(prefix + ".q_vx", 5.0);
            params.q_vy = declare_parameter(prefix + ".q_vy", 5.0);
            params.q_vz = declare_parameter(prefix + ".q_vz", 1.0);
        } else {
            params.q_vx = 0.0; params.q_vy = 0.0; params.q_vz = 0.0;
        }
        params.q_yaw = declare_parameter(prefix + ".q_yaw", 0.01);
        params.q_v_yaw = declare_parameter(prefix + ".q_v_yaw", 0.1);
        params.q_geo = declare_parameter(prefix + ".q_geo", 0.0001);
        params.r_x = declare_parameter(prefix + ".r_x", 0.5);
        params.r_y = declare_parameter(prefix + ".r_y", 0.5);
        params.r_z = declare_parameter(prefix + ".r_z", 0.5);
        params.r_yaw = declare_parameter(prefix + ".r_yaw", 0.05);
        params.r_yaw_adaptive_factor = declare_parameter(prefix + ".r_yaw_adaptive_factor", 50.0);
        params.adaptive_tracking = declare_parameter(prefix + ".adaptive_tracking", false);
        params.q_alpha = declare_parameter(prefix + ".q_alpha", 0.1);
        params.dist_scale_coeff = declare_parameter(prefix + ".dist_scale_coeff", 0.1);
        params.z_scale_coeff = declare_parameter(prefix + ".z_scale_coeff", 5.0);
        params.min_update_count = declare_parameter(prefix + ".min_update_count", 5);
        params.max_pos_cov = declare_parameter(prefix + ".max_pos_cov", 3.0);
        params.max_yaw_cov = declare_parameter(prefix + ".max_yaw_cov", 1.0);
    };

    declare_target_params("robot", robot_params_, true);
    declare_target_params("outpost", outpost_params_, false);
    
    // UKF Hyperparams
    ukf_alpha_ = declare_parameter("ukf_alpha", 0.001);
    ukf_beta_ = declare_parameter("ukf_beta", 2.0);
    ukf_kappa_ = declare_parameter("ukf_kappa", 0.0);

    // Ballistics & Trajectory Params
    bullet_speed_ = declare_parameter("bullet_speed", 25.0);
    hit_delay_offset_ = declare_parameter("trajectory.hit_delay_offset", 0.0);
    aim_delay_offset_ = declare_parameter("trajectory.aim_delay_offset", 0.0);
    trajectory_num_points_ = declare_parameter("trajectory.num_points", 11);
    trajectory_dt_ = declare_parameter("trajectory.dt", 0.05);
    trajectory_omega_low_ = declare_parameter("trajectory.omega_low", 1.5);
    trajectory_omega_high_ = declare_parameter("trajectory.omega_high", 4.0);
    trajectory_switch_concentration_ = declare_parameter("trajectory.switch_concentration", 20.0);

    tracker_ = std::make_unique<Tracker>(this->get_clock(), max_lost_duration, min_detect_count);
    updateTrackerParams();
    tracker_->updateUKFParams(ukf_alpha_, ukf_beta_, ukf_kappa_);

    // Parameter callback
    on_set_parameters_callback_handle_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& parameters) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            bool should_reset = false;

            auto update_param = [](const rclcpp::Parameter& param, const std::string& prefix, TargetParams& params, bool has_velocity) {
                const auto& name = param.get_name();
                if (name.find(prefix + ".") != 0) return false;
                std::string key = name.substr(prefix.length() + 1);
                if (key == "q_x") params.q_x = param.as_double();
                else if (key == "q_y") params.q_y = param.as_double();
                else if (key == "q_z") params.q_z = param.as_double();
                else if (has_velocity && key == "q_vx") params.q_vx = param.as_double();
                else if (has_velocity && key == "q_vy") params.q_vy = param.as_double();
                else if (has_velocity && key == "q_vz") params.q_vz = param.as_double();
                else if (key == "q_yaw") params.q_yaw = param.as_double();
                else if (key == "q_v_yaw") params.q_v_yaw = param.as_double();
                else if (key == "q_geo") params.q_geo = param.as_double();
                else if (key == "r_x") params.r_x = param.as_double();
                else if (key == "r_y") params.r_y = param.as_double();
                else if (key == "r_z") params.r_z = param.as_double();
                else if (key == "r_yaw") params.r_yaw = param.as_double();
                else if (key == "r_yaw_adaptive_factor") params.r_yaw_adaptive_factor = param.as_double();
                else if (key == "adaptive_tracking") params.adaptive_tracking = param.as_bool();
                else if (key == "q_alpha") params.q_alpha = param.as_double();
                else if (key == "dist_scale_coeff") params.dist_scale_coeff = param.as_double();
                else if (key == "z_scale_coeff") params.z_scale_coeff = param.as_double();
                else if (key == "min_update_count") params.min_update_count = param.as_int();
                else if (key == "max_pos_cov") params.max_pos_cov = param.as_double();
                else if (key == "max_yaw_cov") params.max_yaw_cov = param.as_double();
                else return false;
                return true;
            };

            for (const auto& param : parameters) {
                if (update_param(param, "robot", robot_params_, true)) should_reset = true;
                else if (update_param(param, "outpost", outpost_params_, false)) should_reset = true;
                else if (param.get_name() == "debug") {
                    debug_ = param.as_bool();
                    if (debug_ && this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
                        createDebugPublishers();
                    } else if (!debug_) {
                        destroyDebugPublishers();
                    }
                }
                else if (param.get_name() == "debug_img_freq") { debug_img_freq_ = param.as_double(); }
                else if (param.get_name() == "ukf_alpha") { ukf_alpha_ = param.as_double(); should_reset = true; }
                else if (param.get_name() == "ukf_beta") { ukf_beta_ = param.as_double(); should_reset = true; }
                else if (param.get_name() == "ukf_kappa") { ukf_kappa_ = param.as_double(); should_reset = true; }
                else if (param.get_name() == "max_lost_duration") tracker_->setMaxLostDuration(param.as_double());
                else if (param.get_name() == "min_detect_count") tracker_->setMinDetectCount(param.as_int());
                else if (param.get_name() == "bullet_speed") bullet_speed_ = param.as_double();
                else if (param.get_name() == "trajectory.hit_delay_offset") hit_delay_offset_ = param.as_double();
                else if (param.get_name() == "trajectory.aim_delay_offset") aim_delay_offset_ = param.as_double();
                else if (param.get_name() == "trajectory.num_points") trajectory_num_points_ = param.as_int();
                else if (param.get_name() == "trajectory.dt") trajectory_dt_ = param.as_double();
                else if (param.get_name() == "trajectory.omega_low") trajectory_omega_low_ = param.as_double();
                else if (param.get_name() == "trajectory.omega_high") trajectory_omega_high_ = param.as_double();
                else if (param.get_name() == "trajectory.switch_concentration") trajectory_switch_concentration_ = param.as_double();
            }

            if (should_reset) {
                RCLCPP_INFO(this->get_logger(), "Parameters updating... Resetting tracker.");
                updateTrackerParams();
                tracker_->updateUKFParams(ukf_alpha_, ukf_beta_, ukf_kappa_);
                tracker_->reset();
            }
            return result;
        });

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    aim_pub_ = create_publisher<robot_interfaces::msg::Aim>("armor_solver/aim", rclcpp::SensorDataQoS());
    target_state_pub_ = create_publisher<robot_interfaces::msg::TargetState>("armor_solver/target_state", rclcpp::SensorDataQoS());
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("armor_solver/markers", 10);
    trajectory_pub_ = create_publisher<robot_interfaces::msg::TargetTrajectory>("armor_solver/trajectory", 10);

    // Create timer for prediction and publication (100Hz)
    timer_ = create_wall_timer(std::chrono::milliseconds(10), std::bind(&ArmorSolverNode::timerCallback, this));

    return CallbackReturn::SUCCESS;
}

ArmorSolverNode::CallbackReturn ArmorSolverNode::on_activate(const rclcpp_lifecycle::State& /*state*/) {
    RCLCPP_INFO(get_logger(), "Activating ArmorSolverNode...");
    
    armors_sub_ = create_subscription<robot_interfaces::msg::Armors>(
        "armor_detector/armors", rclcpp::SensorDataQoS(), std::bind(&ArmorSolverNode::armorsCallback, this, std::placeholders::_1));

    aim_pub_->on_activate();
    target_state_pub_->on_activate();
    marker_pub_->on_activate();
    trajectory_pub_->on_activate();

    if (debug_) {
        createDebugPublishers();
    }

    return CallbackReturn::SUCCESS;
}

ArmorSolverNode::CallbackReturn ArmorSolverNode::on_deactivate(const rclcpp_lifecycle::State& /*state*/) {
    RCLCPP_INFO(get_logger(), "Deactivating ArmorSolverNode...");
    
    armors_sub_.reset();
    aim_pub_->on_deactivate();
    target_state_pub_->on_deactivate();
    marker_pub_->on_deactivate();
    trajectory_pub_->on_deactivate();

    if (debug_) {
        destroyDebugPublishers();
    }

    return CallbackReturn::SUCCESS;
}

ArmorSolverNode::CallbackReturn ArmorSolverNode::on_cleanup(const rclcpp_lifecycle::State& /*state*/) {
    RCLCPP_INFO(get_logger(), "Cleaning up ArmorSolverNode...");

    if (debug_) {
        destroyDebugPublishers();
    }
    
    return CallbackReturn::SUCCESS;
}

ArmorSolverNode::CallbackReturn ArmorSolverNode::on_shutdown(const rclcpp_lifecycle::State& /*state*/) {
    RCLCPP_INFO(get_logger(), "Shutting down ArmorSolverNode...");
    return CallbackReturn::SUCCESS;
}

void ArmorSolverNode::armorsCallback(const robot_interfaces::msg::Armors::SharedPtr armors_msg) {
    rclcpp::Time timestamp = armors_msg->header.stamp;
    std::vector<TrackerArmor> detector_armors;

    try {
        auto transform = tf_buffer_->lookupTransform(
            odom_frame_, armors_msg->header.frame_id, timestamp,
            rclcpp::Duration::from_seconds(0.01));

        for (const auto& msg_armor : armors_msg->armors) {
            TrackerArmor armor;
            armor.number = msg_armor.number;
            armor.type = (msg_armor.type == "small") ? ArmorType::SMALL : ArmorType::LARGE;
            armor.timestamp = timestamp;
            
            // Transform position
            geometry_msgs::msg::PoseStamped ps_in, ps_out;
            ps_in.header = armors_msg->header;
            ps_in.pose = msg_armor.pose;
            tf2::doTransform(ps_in, ps_out, transform);
            
            armor.position = Eigen::Vector3d(ps_out.pose.position.x, ps_out.pose.position.y, ps_out.pose.position.z);
            armor.orientation = Eigen::Quaterniond(ps_out.pose.orientation.w, ps_out.pose.orientation.x, 
                                                   ps_out.pose.orientation.y, ps_out.pose.orientation.z);
            
            // Extract yaw from orientation
            tf2::Quaternion q(ps_out.pose.orientation.x, ps_out.pose.orientation.y, 
                              ps_out.pose.orientation.z, ps_out.pose.orientation.w);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            armor.yaw = yaw;
            
            // Priority (can be based on distance to center)
            armor.priority = msg_armor.distance_to_image_center;
            
            detector_armors.push_back(armor);
        }
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "TF Exception: %s", ex.what());
    }

    // Sensor update ONLY
    tracker_->track(detector_armors, timestamp);
}

void ArmorSolverNode::timerCallback() {
    auto now = this->now();

    // 1. Let tracker handle internal state machine timeouts based on wall time
    tracker_->handleTimeouts(now);

    // 2. Query target
    auto target = tracker_->getTarget();

    if (target) {
        // Smooth extrinsic state for control and visualization
        auto state = target->getPredictedState(now);
        auto cov = target->getCovariance();

        std_msgs::msg::Header header;
        header.stamp = now;
        header.frame_id = odom_frame_;

        // Publish state for debugging
        robot_interfaces::msg::TargetState state_msg;

        if (target->getArmorNum() == 3) { // Outpost
            state_msg.x = state(0); state_msg.v_x = 0;
            state_msg.y = state(1); state_msg.v_y = 0;
            state_msg.z = state(2); state_msg.v_z = 0;
            state_msg.yaw = state(3); state_msg.v_yaw = state(4);
            state_msg.r = state(5);
            state_msg.l = 0;
            state_msg.h = state(6);
            
            state_msg.p_x = cov(0,0); state_msg.p_vx = 0;
            state_msg.p_y = cov(1,1); state_msg.p_vy = 0;
            state_msg.p_z = cov(2,2); state_msg.p_vz = 0;
            state_msg.p_yaw = cov(3,3); state_msg.p_vyaw = cov(4,4);
            state_msg.p_r = cov(5,5);
            state_msg.p_l = 0;
            state_msg.p_h = cov(6,6);
        } else { // Robot
            state_msg.x = state(0); state_msg.v_x = state(1);
            state_msg.y = state(2); state_msg.v_y = state(3);
            state_msg.z = state(4); state_msg.v_z = state(5);
            state_msg.yaw = state(6); state_msg.v_yaw = state(7);
            state_msg.r = state(8);
            state_msg.l = state(9);
            state_msg.h = state(10);
            
            state_msg.p_x = cov(0,0); state_msg.p_vx = cov(1,1);
            state_msg.p_y = cov(2,2); state_msg.p_vy = cov(3,3);
            state_msg.p_z = cov(4,4); state_msg.p_vz = cov(5,5);
            state_msg.p_yaw = cov(6,6); state_msg.p_vyaw = cov(7,7);
            state_msg.p_r = cov(8,8);
            state_msg.p_l = cov(9,9);
            state_msg.p_h = cov(10,10);
        }

        target_state_pub_->publish(state_msg);

        Eigen::Vector3d central_hit_pt(0,0,0);
        Eigen::Vector3d central_aim_pt(0,0,0);
        bool has_prediction = false;

        // --- Trajectory Generation ---
        if ((tracker_->getState() == Tracker::State::TRACKING || tracker_->getState() == Tracker::State::TEMP_LOST) && target->isConverged()) {
            double pipeline_latency = (now - tracker_->getLastSeenTime()).seconds();
            double tof = robot_ballistics::BallisticsCalculator::calculatePredictTime(
                Eigen::Vector3d(state_msg.x, state_msg.y, state_msg.z), 
                bullet_speed_, 0.0, 0.0); // Pure TOF
            
            double t_predict_hit = tof + pipeline_latency + hit_delay_offset_;
            double t_predict_aim = tof + pipeline_latency + aim_delay_offset_;

            robot_interfaces::msg::TargetTrajectory traj_msg;
            traj_msg.header = header;
            
            int num_points = trajectory_num_points_ > 0 && trajectory_num_points_ % 2 != 0 ? trajectory_num_points_ : 11;
            int half_points = num_points / 2;

            for (int i = -half_points; i <= half_points; ++i) {
                double dt_i_hit = t_predict_hit + i * trajectory_dt_;
                double dt_i_aim = t_predict_aim + i * trajectory_dt_;

                auto future_state_hit = target->getPredictedState(tracker_->getLastSeenTime() + rclcpp::Duration::from_seconds(dt_i_hit));
                auto future_state_aim = target->getPredictedState(tracker_->getLastSeenTime() + rclcpp::Duration::from_seconds(dt_i_aim));
                
                auto get_hit_pt = [&](const Eigen::VectorXd& state, bool blend) {
                    double cx, cy, cz, yaw, v_yaw, r, vx, vy, vz;
                    int armor_num = target->getArmorNum();
                    if (armor_num == 3) {
                        cx = state(0); cy = state(1); cz = state(2);
                        vx = 0; vy = 0; vz = 0;
                        yaw = state(3); v_yaw = state(4); r = state(5);
                    } else {
                        cx = state(0); cy = state(2); cz = state(4);
                        vx = state(1); vy = state(3); vz = state(5);
                        yaw = state(6); v_yaw = state(7); r = state(8);
                    }

                    double dir_to_origin = std::atan2(-cy, -cx);
                    double total_weight = 0.0;
                    double blended_ax = 0.0, blended_ay = 0.0, blended_az = 0.0;
                    double max_weight = -1.0;
                    double best_ax = cx, best_ay = cy, best_az = cz;

                    for (int j = 0; j < armor_num; ++j) {
                        double armor_angle, h_offset = 0;
                        double current_r = r;
                        if (armor_num == 3) {
                            armor_angle = robot_utils::normalize_angle(yaw + j * 2.0 * M_PI / 3.0);
                            h_offset = -j * state(6);
                        } else {
                            armor_angle = robot_utils::normalize_angle(yaw + j * M_PI / 2.0);
                            if (j % 2 != 0) {
                                current_r += state(9);
                                h_offset = state(10);
                            }
                        }
                        double armor_facing = robot_utils::normalize_angle(armor_angle + M_PI);
                        double angle_diff = std::abs(robot_utils::normalize_angle(dir_to_origin - armor_facing));
                        double ax = cx - current_r * std::cos(armor_angle);
                        double ay = cy - current_r * std::sin(armor_angle);
                        double az = cz + h_offset;

                        double weight = std::pow(std::max(0.0, std::cos(angle_diff)), trajectory_switch_concentration_);
                        blended_ax += ax * weight; blended_ay += ay * weight; blended_az += az * weight;
                        total_weight += weight;
                        if (weight > max_weight) {
                            max_weight = weight;
                            best_ax = ax; best_ay = ay; best_az = az;
                        }
                    }

                    Eigen::Vector3d res;
                    if (blend && total_weight > 1e-6) {
                        res << blended_ax / total_weight, blended_ay / total_weight, blended_az / total_weight;
                    } else {
                        res << best_ax, best_ay, best_az;
                    }

                    if (blend) {
                        double omega = std::abs(v_yaw);
                        double r_ratio = 1.0;
                        if (omega > trajectory_omega_high_) r_ratio = 0.0;
                        else if (omega > trajectory_omega_low_) r_ratio = (trajectory_omega_high_ - omega) / (trajectory_omega_high_ - trajectory_omega_low_);
                        res(0) = cx + (res(0) - cx) * r_ratio;
                        res(1) = cy + (res(1) - cy) * r_ratio;
                    }

                    return std::make_pair(res, Eigen::Vector3d(vx, vy, vz));
                };

                auto hit_info = get_hit_pt(future_state_hit, false);
                auto aim_info = get_hit_pt(future_state_aim, true);

                robot_interfaces::msg::TargetTrajectoryPoint true_pt, aim_pt;
                true_pt.time_offset = i * trajectory_dt_;
                true_pt.x = hit_info.first.x(); true_pt.y = hit_info.first.y(); true_pt.z = hit_info.first.z();
                true_pt.v_x = hit_info.second.x(); true_pt.v_y = hit_info.second.y(); true_pt.v_z = hit_info.second.z();

                aim_pt.time_offset = i * trajectory_dt_;
                aim_pt.x = aim_info.first.x(); aim_pt.y = aim_info.first.y(); aim_pt.z = aim_info.first.z();
                aim_pt.v_x = aim_info.second.x(); aim_pt.v_y = aim_info.second.y(); aim_pt.v_z = aim_info.second.z();

                traj_msg.true_trajectory.push_back(true_pt);
                traj_msg.aim_trajectory.push_back(aim_pt);

                if (i == 0) {
                    central_hit_pt = Eigen::Vector3d(true_pt.x, true_pt.y, true_pt.z);
                    central_aim_pt = Eigen::Vector3d(aim_pt.x, aim_pt.y, aim_pt.z);
                    has_prediction = true;
                }
            }
            trajectory_pub_->publish(traj_msg);
        }
        // -----------------------------

        publishMarkers(target, header, has_prediction, central_hit_pt, central_aim_pt);
    }
}

void ArmorSolverNode::publishMarkers(const std::shared_ptr<TargetBase>& target, 
                                    const std_msgs::msg::Header& header,
                                    bool has_prediction,
                                    const Eigen::Vector3d& hit_pt,
                                    const Eigen::Vector3d& aim_pt) {
    visualization_msgs::msg::MarkerArray marker_array;
    auto state = target->getPredictedState(header.stamp);

    // Center marker
    visualization_msgs::msg::Marker center_marker;
    center_marker.header = header;
    center_marker.ns = "center";
    center_marker.id = 0;
    center_marker.type = visualization_msgs::msg::Marker::SPHERE;
    center_marker.action = visualization_msgs::msg::Marker::ADD;
    
    int armor_num = target->getArmorNum();
    if (armor_num == 3) {
        center_marker.pose.position.x = state(0);
        center_marker.pose.position.y = state(1);
        center_marker.pose.position.z = state(2);
    } else {
        center_marker.pose.position.x = state(0);
        center_marker.pose.position.y = state(2);
        center_marker.pose.position.z = state(4);
    }
    
    center_marker.scale.x = center_marker.scale.y = center_marker.scale.z = 0.1;
    center_marker.color.a = 1.0;
    center_marker.color.g = 1.0;
    marker_array.markers.push_back(center_marker);

    // Hit and Aim Point Visualization
    if (has_prediction) {
        visualization_msgs::msg::Marker hit_marker;
        hit_marker.header = header;
        hit_marker.ns = "hit_aim";
        hit_marker.id = 0;
        hit_marker.type = visualization_msgs::msg::Marker::SPHERE;
        hit_marker.action = visualization_msgs::msg::Marker::ADD;
        hit_marker.pose.position.x = hit_pt.x();
        hit_marker.pose.position.y = hit_pt.y();
        hit_marker.pose.position.z = hit_pt.z();
        hit_marker.scale.x = hit_marker.scale.y = hit_marker.scale.z = 0.08;
        hit_marker.color.a = 1.0;
        hit_marker.color.r = 1.0; hit_marker.color.b = 1.0; // Magenta for hit point
        marker_array.markers.push_back(hit_marker);

        visualization_msgs::msg::Marker aim_marker;
        aim_marker.header = header;
        aim_marker.ns = "hit_aim";
        aim_marker.id = 1;
        aim_marker.type = visualization_msgs::msg::Marker::SPHERE;
        aim_marker.action = visualization_msgs::msg::Marker::ADD;
        aim_marker.pose.position.x = aim_pt.x();
        aim_marker.pose.position.y = aim_pt.y();
        aim_marker.pose.position.z = aim_pt.z();
        aim_marker.scale.x = aim_marker.scale.y = aim_marker.scale.z = 0.08;
        aim_marker.color.a = 1.0;
        aim_marker.color.g = 1.0; aim_marker.color.b = 1.0; // Cyan for aim point
        marker_array.markers.push_back(aim_marker);
    }

    // Armor markers
    double yaw, r;
    if (armor_num == 3) {
        yaw = state(3);
        r = state(5);
    } else {
        yaw = state(6);
        r = state(8);
    }

    std::vector<geometry_msgs::msg::Point> armor_positions;

    for (int i = 0; i < armor_num; ++i) {
        visualization_msgs::msg::Marker armor_marker;
        armor_marker.header = header;
        armor_marker.ns = "armors";
        armor_marker.id = i;
        armor_marker.type = visualization_msgs::msg::Marker::CUBE;
        armor_marker.action = visualization_msgs::msg::Marker::ADD;

        double angle;
        geometry_msgs::msg::Point p;

        if (armor_num == 3) { // Outpost
            double h = state(6);
            angle = yaw + i * 2.0 * M_PI / 3.0;
            p.x = state(0) - r * std::cos(angle);
            p.y = state(1) - r * std::sin(angle);
            p.z = state(2) - i * h;
            armor_positions.push_back(p);
        } else { // Robot
            double l = state(9);
            double h_val = state(10);
            angle = yaw + i * M_PI / 2.0;
            double current_r = (i % 2 == 0) ? r : r + l;
            double current_z = (i % 2 == 0) ? state(4) : state(4) + h_val;
            p.x = state(0) - current_r * std::cos(angle);
            p.y = state(2) - current_r * std::sin(angle);
            p.z = current_z;
            armor_positions.push_back(p);
        }

        armor_marker.pose.position = p;

        tf2::Quaternion q;
        q.setRPY(0, 0, angle);
        armor_marker.pose.orientation = tf2::toMsg(q);

        armor_marker.scale.x = 0.02;
        armor_marker.scale.y = 0.135;
        armor_marker.scale.z = 0.06;
        armor_marker.color.a = 1.0;
        armor_marker.color.r = 1.0;
        marker_array.markers.push_back(armor_marker);
    }

    if (armor_num == 3 && armor_positions.size() == 3) {
        visualization_msgs::msg::Marker line_list;
        line_list.header = header;
        line_list.ns = "outpost_lines";
        line_list.id = 0;
        line_list.type = visualization_msgs::msg::Marker::LINE_LIST;
        line_list.action = visualization_msgs::msg::Marker::ADD;
        line_list.pose.orientation.w = 1.0;
        line_list.scale.x = 0.01; // Line width
        line_list.color.a = 1.0;
        line_list.color.b = 1.0; // Blue color for lines

        line_list.points.push_back(armor_positions[0]);
        line_list.points.push_back(armor_positions[1]);
        line_list.points.push_back(armor_positions[1]);
        line_list.points.push_back(armor_positions[2]);
        line_list.points.push_back(armor_positions[2]);
        line_list.points.push_back(armor_positions[0]);
        marker_array.markers.push_back(line_list);
    }

    marker_pub_->publish(marker_array);
}

void ArmorSolverNode::updateTrackerParams() {
    tracker_->updateParams(robot_params_, outpost_params_);
}

void ArmorSolverNode::createDebugPublishers() {
    debug_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("armor_solver/debug_image", rclcpp::SensorDataQoS());
    debug_img_pub_->on_activate();

    cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/image_raw_info", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::CameraInfo::SharedPtr camera_info) {
            cam_info_ = camera_info;
            camera_matrix_ = cv::Mat(3, 3, CV_64F, camera_info->k.data()).clone();
            dist_coeffs_ = cv::Mat(1, 5, CV_64F, camera_info->d.data()).clone();
            cam_info_sub_.reset();
        });

    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw", rclcpp::SensorDataQoS(),
        std::bind(&ArmorSolverNode::imageCallback, this, std::placeholders::_1));
}

void ArmorSolverNode::destroyDebugPublishers() {
    if (debug_img_pub_) {
        debug_img_pub_->on_deactivate();
        debug_img_pub_.reset();
    }
    cam_info_sub_.reset();
    img_sub_.reset();
}

void ArmorSolverNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr img_msg) {
    if (!cam_info_ || !tracker_ || !tracker_->getTarget() || !debug_) return;

    auto now = this->now();
    if ((now - last_debug_img_time_).seconds() < (1.0 / debug_img_freq_)) return;
    last_debug_img_time_ = now;

    auto target = tracker_->getTarget();
    if (tracker_->getState() == Tracker::State::LOST) return;

    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_buffer_->lookupTransform(
            img_msg->header.frame_id, odom_frame_, img_msg->header.stamp,
            rclcpp::Duration::from_seconds(0.01));
    } catch (const tf2::TransformException &ex) {
        return;
    }

    auto state = target->getPredictedState(img_msg->header.stamp);
    int armor_num = target->getArmorNum();
    
    double w = (target->getType() == ArmorType::SMALL) ? SMALL_ARMOR_WIDTH : LARGE_ARMOR_WIDTH;
    double h = (target->getType() == ArmorType::SMALL) ? SMALL_ARMOR_HEIGHT : LARGE_ARMOR_HEIGHT;

    std::vector<cv::Point3f> object_points;
    auto add_armor_corners = [&](double ax, double ay, double az, double angle, double pitch) {
        // Rotation around vertical axis (yaw/angle) then around horizontal axis (pitch)
        // tx, ty is the direction along the width of the armor
        double tx = -std::sin(angle);
        double ty = std::cos(angle);
        
        // n is the normal of the armor (in horizontal plane)
        double nx = std::cos(angle);
        double ny = std::sin(angle);

        // Corner offsets considering pitch (tilt)
        // The vertical direction of the armor is tilted by 'pitch'
        // New vertical direction: ( -nx*sin(pitch), -ny*sin(pitch), cos(pitch) )
        double vx = -nx * std::sin(pitch);
        double vy = -ny * std::sin(pitch);
        double vz = std::cos(pitch);

        object_points.emplace_back(ax + (w/2)*tx + (h/2)*vx, ay + (w/2)*ty + (h/2)*vy, az + (h/2)*vz);
        object_points.emplace_back(ax - (w/2)*tx + (h/2)*vx, ay - (w/2)*ty + (h/2)*vy, az + (h/2)*vz);
        object_points.emplace_back(ax - (w/2)*tx - (h/2)*vx, ay - (w/2)*ty - (h/2)*vy, az - (h/2)*vz);
        object_points.emplace_back(ax + (w/2)*tx - (h/2)*vx, ay + (w/2)*ty - (h/2)*vy, az - (h/2)*vz);
    };

    double yaw, r;
    if (armor_num == 3) {
        yaw = state(3); r = state(5);
        for (int i = 0; i < 3; ++i) {
            double angle = robot_utils::normalize_angle(yaw + i * 2.0 * M_PI / 3.0);
            double ax = state(0) - r * std::cos(angle);
            double ay = state(1) - r * std::sin(angle);
            double az = state(2) - i * state(6);
            add_armor_corners(ax, ay, az, angle, FIFTEEN_DEGREE_RAD);
        }
    } else {
        yaw = state(6); r = state(8);
        for (int i = 0; i < armor_num; ++i) {
            double angle = robot_utils::normalize_angle(yaw + i * M_PI / 2.0);
            double current_r = (i % 2 == 0) ? r : r + state(9);
            double current_z = (i % 2 == 0) ? state(4) : state(4) + state(10);
            double ax = state(0) - current_r * std::cos(angle);
            double ay = state(2) - current_r * std::sin(angle);
            double az = current_z;
            add_armor_corners(ax, ay, az, angle, -FIFTEEN_DEGREE_RAD);
        }
    }

    // Transform points to camera frame
    std::vector<cv::Point3f> cam_points;
    std::vector<int> valid_indices;
    for (size_t i = 0; i < object_points.size(); ++i) {
        const auto& pt = object_points[i];
        geometry_msgs::msg::PointStamped ps_in, ps_out;
        ps_in.header.frame_id = odom_frame_;
        ps_in.header.stamp = img_msg->header.stamp;
        ps_in.point.x = pt.x; ps_in.point.y = pt.y; ps_in.point.z = pt.z;
        tf2::doTransform(ps_in, ps_out, transform);
        if (ps_out.point.z > 0) {
            cam_points.emplace_back(ps_out.point.x, ps_out.point.y, ps_out.point.z);
            valid_indices.push_back(i);
        }
    }

    if (cam_points.empty()) return;

    std::vector<cv::Point2f> image_points;
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
    cv::projectPoints(cam_points, rvec, tvec, camera_matrix_, dist_coeffs_, image_points);

    // Map projected points back to their original indices for drawing lines
    std::map<int, cv::Point2f> index_to_pt;
    for (size_t i = 0; i < valid_indices.size(); ++i) {
        index_to_pt[valid_indices[i]] = image_points[i];
    }

    try {
        cv::Mat img = cv_bridge::toCvCopy(img_msg, "rgb8")->image;
        cv::Scalar color = target->isConverged() ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 255, 255);
        for (int i = 0; i < armor_num; ++i) {
            int base = i * 4;
            std::vector<int> armor_indices = {base, base + 1, base + 2, base + 3};
            for (int j = 0; j < 4; ++j) {
                int idx1 = armor_indices[j];
                int idx2 = armor_indices[(j + 1) % 4];
                if (index_to_pt.count(idx1) && index_to_pt.count(idx2)) {
                    cv::line(img, index_to_pt[idx1], index_to_pt[idx2], color, 2);
                }
            }
        }
        
        debug_img_pub_->publish(*cv_bridge::CvImage(img_msg->header, "rgb8", img).toImageMsg());
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    }
}

} // namespace robot_auto_aim

RCLCPP_COMPONENTS_REGISTER_NODE(robot_auto_aim::ArmorSolverNode)
