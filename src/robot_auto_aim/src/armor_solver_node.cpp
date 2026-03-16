#include "robot_auto_aim/armor_solver_node.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "rclcpp_components/register_node_macro.hpp"

namespace robot_auto_aim {

ArmorSolverNode::ArmorSolverNode(const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("armor_solver", options) {
}

ArmorSolverNode::CallbackReturn ArmorSolverNode::on_configure(const rclcpp_lifecycle::State& /*state*/) {
    RCLCPP_INFO(get_logger(), "Configuring ArmorSolverNode...");

    odom_frame_ = declare_parameter("odom_frame", "odom");
    double max_lost_duration = declare_parameter("max_lost_duration", 1.0);
    int min_detect_count = declare_parameter("min_detect_count", 5);

    auto declare_target_params = [this](const std::string& prefix, TargetParams& params, bool has_velocity) {
        params.q_x = declare_parameter(prefix + ".q_x", 0.001);
        params.q_y = declare_parameter(prefix + ".q_y", 0.001);
        params.q_z = declare_parameter(prefix + ".q_z", 0.001);
        if (has_velocity) {
            params.q_vx = declare_parameter(prefix + ".q_vx", 0.1);
            params.q_vy = declare_parameter(prefix + ".q_vy", 0.1);
            params.q_vz = declare_parameter(prefix + ".q_vz", 0.1);
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
    };

    declare_target_params("robot", robot_params_, true);
    declare_target_params("outpost", outpost_params_, false);
    
    // UKF Hyperparams
    ukf_alpha_ = declare_parameter("ukf_alpha", 0.001);
    ukf_beta_ = declare_parameter("ukf_beta", 2.0);
    ukf_kappa_ = declare_parameter("ukf_kappa", 0.0);

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
                else return false;
                return true;
            };

            for (const auto& param : parameters) {
                if (update_param(param, "robot", robot_params_, true)) should_reset = true;
                else if (update_param(param, "outpost", outpost_params_, false)) should_reset = true;
                else if (param.get_name() == "ukf_alpha") { ukf_alpha_ = param.as_double(); should_reset = true; }
                else if (param.get_name() == "ukf_beta") { ukf_beta_ = param.as_double(); should_reset = true; }
                else if (param.get_name() == "ukf_kappa") { ukf_kappa_ = param.as_double(); should_reset = true; }
                else if (param.get_name() == "max_lost_duration") tracker_->setMaxLostDuration(param.as_double());
                else if (param.get_name() == "min_detect_count") tracker_->setMinDetectCount(param.as_int());
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

    return CallbackReturn::SUCCESS;
}

ArmorSolverNode::CallbackReturn ArmorSolverNode::on_deactivate(const rclcpp_lifecycle::State& /*state*/) {
    RCLCPP_INFO(get_logger(), "Deactivating ArmorSolverNode...");
    
    armors_sub_.reset();
    aim_pub_->on_deactivate();
    target_state_pub_->on_deactivate();
    marker_pub_->on_deactivate();

    return CallbackReturn::SUCCESS;
}

ArmorSolverNode::CallbackReturn ArmorSolverNode::on_cleanup(const rclcpp_lifecycle::State& /*state*/) {
    RCLCPP_INFO(get_logger(), "Cleaning up ArmorSolverNode...");
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

        robot_interfaces::msg::Aim aim_msg;
        aim_msg.header.stamp = now;
        aim_msg.header.frame_id = odom_frame_;
        aim_msg.success = (tracker_->getState() == Tracker::State::TRACKING);
        
        // Publish state for debugging
        robot_interfaces::msg::TargetState state_msg;
        state_msg.header = aim_msg.header;

        if (target->getArmorNum() == 3) { // Outpost
            aim_msg.yaw = state(3);
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
            aim_msg.yaw = state(6);
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
        
        aim_pub_->publish(aim_msg);
        target_state_pub_->publish(state_msg);

        publishMarkers(target, aim_msg.header);
    } else {
        robot_interfaces::msg::Aim aim_msg;
        aim_msg.header.stamp = now;
        aim_msg.success = false;
        aim_pub_->publish(aim_msg);
    }
}

void ArmorSolverNode::publishMarkers(const std::shared_ptr<TargetBase>& target, const std_msgs::msg::Header& header) {
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

} // namespace robot_auto_aim

RCLCPP_COMPONENTS_REGISTER_NODE(robot_auto_aim::ArmorSolverNode)
