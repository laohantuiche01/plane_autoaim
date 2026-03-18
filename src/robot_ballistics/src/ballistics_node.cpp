#include "robot_ballistics/ballistics_node.hpp"
#include <cmath>

namespace robot_ballistics {

BallisticsNode::BallisticsNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("ballistics_node", options) {
    
    // Parameters
    bullet_speed_ = this->declare_parameter("bullet_speed", 25.0);
    gimbal_frame_ = this->declare_parameter("gimbal_frame", "gimbal_link");
    true_angle_tolerance_ = this->declare_parameter("true_angle_tolerance", 0.05);
    aim_angle_tolerance_ = this->declare_parameter("aim_angle_tolerance", 0.05);
    
    hit_yaw_offset_ = this->declare_parameter("hit_yaw_offset", 0.0);
    hit_pitch_offset_ = this->declare_parameter("hit_pitch_offset", 0.0);
    aim_yaw_offset_ = this->declare_parameter("aim_yaw_offset", 0.0);
    aim_pitch_offset_ = this->declare_parameter("aim_pitch_offset", 0.0);

    enable_sg_yaw_ = this->declare_parameter("enable_sg_yaw", true);
    sg_yaw_order_ = this->declare_parameter("sg_yaw_order", 2);
    enable_sg_pitch_ = this->declare_parameter("enable_sg_pitch", true);
    sg_pitch_order_ = this->declare_parameter("sg_pitch_order", 2);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    trajectory_sub_ = this->create_subscription<robot_interfaces::msg::TargetTrajectory>(
        "armor_solver/trajectory", rclcpp::SensorDataQoS(),
        std::bind(&BallisticsNode::trajectoryCallback, this, std::placeholders::_1));

    aim_pub_ = this->create_publisher<robot_interfaces::msg::Aim>("/robot/aim", 10);
    debug_pub_ = this->create_publisher<robot_interfaces::msg::BallisticsDebug>("/ballistics/debug", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/ballistics/trajectory_marker", 10);

    // Parameter Callback
    on_set_parameters_callback_handle_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& parameters) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            for (const auto& param : parameters) {
                if (param.get_name() == "bullet_speed") bullet_speed_ = param.as_double();
                else if (param.get_name() == "gimbal_frame") gimbal_frame_ = param.as_string();
                else if (param.get_name() == "true_angle_tolerance") true_angle_tolerance_ = param.as_double();
                else if (param.get_name() == "aim_angle_tolerance") aim_angle_tolerance_ = param.as_double();
                else if (param.get_name() == "hit_yaw_offset") hit_yaw_offset_ = param.as_double();
                else if (param.get_name() == "hit_pitch_offset") hit_pitch_offset_ = param.as_double();
                else if (param.get_name() == "aim_yaw_offset") aim_yaw_offset_ = param.as_double();
                else if (param.get_name() == "aim_pitch_offset") aim_pitch_offset_ = param.as_double();
                else if (param.get_name() == "enable_sg_yaw") enable_sg_yaw_ = param.as_bool();
                else if (param.get_name() == "sg_yaw_order") sg_yaw_order_ = param.as_int();
                else if (param.get_name() == "enable_sg_pitch") enable_sg_pitch_ = param.as_bool();
                else if (param.get_name() == "sg_pitch_order") sg_pitch_order_ = param.as_int();
            }
            return result;
        });
}

void BallisticsNode::trajectoryCallback(const robot_interfaces::msg::TargetTrajectory::SharedPtr msg) {
    if (msg->true_trajectory.empty() || msg->aim_trajectory.empty() || 
        msg->true_trajectory.size() != msg->aim_trajectory.size()) {
        return;
    }

    size_t num_points = msg->true_trajectory.size();
    if (num_points < 3 || num_points % 2 == 0) {
        RCLCPP_WARN(this->get_logger(), "Trajectory size must be odd and >= 3.");
        return;
    }

    geometry_msgs::msg::TransformStamped odom_to_gimbal;
    geometry_msgs::msg::TransformStamped gimbal_to_odom;
    try {
        // odom_to_gimbal: takes points from world to gimbal frame (for success check)
        odom_to_gimbal = tf_buffer_->lookupTransform(
            gimbal_frame_, msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.005));
        // gimbal_to_odom: gives gimbal position in world frame (for absolute angle calculation)
        gimbal_to_odom = tf_buffer_->lookupTransform(
            msg->header.frame_id, gimbal_frame_, msg->header.stamp, rclcpp::Duration::from_seconds(0.005));
    } catch (const tf2::ExtrapolationException& ex) {
        // If message time is slightly in the future, fallback to latest available transform
        try {
            odom_to_gimbal = tf_buffer_->lookupTransform(
                gimbal_frame_, msg->header.frame_id, tf2::TimePointZero);
            gimbal_to_odom = tf_buffer_->lookupTransform(
                msg->header.frame_id, gimbal_frame_, tf2::TimePointZero);
        } catch (const tf2::TransformException& ex2) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "TF fallback error: %s", ex2.what());
            return;
        }
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "TF error: %s", ex.what());
        return;
    }

    std::vector<double> true_yaw(num_points), true_pitch(num_points);
    std::vector<double> aim_yaw(num_points), aim_pitch(num_points);

    auto process_point = [&](const robot_interfaces::msg::TargetTrajectoryPoint& pt, double& yaw, double& pitch) {
        // Calculate relative vector in world frame (odom) to get absolute angles
        double dx = pt.x - gimbal_to_odom.transform.translation.x;
        double dy = pt.y - gimbal_to_odom.transform.translation.y;
        double dz = pt.z - gimbal_to_odom.transform.translation.z;

        yaw = std::atan2(dy, dx);
        pitch = BallisticsCalculator::calculatePitch(Eigen::Vector3d(dx, dy, dz), bullet_speed_);
    };

    int center_idx = num_points / 2;

    for (size_t i = 0; i < num_points; ++i) {
        process_point(msg->true_trajectory[i], true_yaw[i], true_pitch[i]);
        true_yaw[i] += hit_yaw_offset_;
        true_pitch[i] += hit_pitch_offset_;

        process_point(msg->aim_trajectory[i], aim_yaw[i], aim_pitch[i]);
        aim_yaw[i] += aim_yaw_offset_;
        aim_pitch[i] += aim_pitch_offset_;
    }

    std::vector<double> times(num_points);
    for (size_t i = 0; i < num_points; ++i) {
        times[i] = msg->true_trajectory[i].time_offset;
    }

    double final_aim_yaw, final_aim_pitch, final_w_yaw, final_w_pitch;

    // Unwrap angles to prevent S-G filter spikes at 2pi boundaries
    robot_utils::unwrap_angles(true_yaw);
    robot_utils::unwrap_angles(aim_yaw);
    // Pitch typically doesn't wrap but we handle it for consistency
    robot_utils::unwrap_angles(true_pitch);
    robot_utils::unwrap_angles(aim_pitch);

    if (enable_sg_yaw_ && sg_yaw_order_ < static_cast<int>(num_points)) {
        robot_utils::SavitzkyGolayFilter sg_yaw(num_points, sg_yaw_order_);
        auto result = sg_yaw.fit(times, aim_yaw, 0.0);
        final_aim_yaw = robot_utils::normalize_angle(result.first);
        final_w_yaw = result.second;
    } else {
        final_aim_yaw = robot_utils::normalize_angle(aim_yaw[center_idx]);
        double dt = (num_points > 1) ? (times[center_idx+1] - times[center_idx-1]) : 0.05;
        final_w_yaw = (dt > 0) ? (aim_yaw[center_idx+1] - aim_yaw[center_idx-1]) / (2 * dt) : 0.0;
    }

    if (enable_sg_pitch_ && sg_pitch_order_ < static_cast<int>(num_points)) {
        robot_utils::SavitzkyGolayFilter sg_pitch(num_points, sg_pitch_order_);
        auto result = sg_pitch.fit(times, aim_pitch, 0.0);
        final_aim_pitch = result.first;
        final_w_pitch = result.second;
    } else {
        final_aim_pitch = aim_pitch[center_idx];
        double dt = (num_points > 1) ? (times[center_idx+1] - times[center_idx-1]) : 0.05;
        final_w_pitch = (dt > 0) ? (aim_pitch[center_idx+1] - aim_pitch[center_idx-1]) / (2 * dt) : 0.0;
    }

    // Calculate current gimbal X axis in odom frame
    tf2::Quaternion q(
        gimbal_to_odom.transform.rotation.x,
        gimbal_to_odom.transform.rotation.y,
        gimbal_to_odom.transform.rotation.z,
        gimbal_to_odom.transform.rotation.w);
    tf2::Vector3 gimbal_x = tf2::Matrix3x3(q).getColumn(0);
    double current_yaw = std::atan2(gimbal_x.y(), gimbal_x.x());
    double current_pitch = std::atan2(gimbal_x.z(), std::sqrt(gimbal_x.x()*gimbal_x.x() + gimbal_x.y()*gimbal_x.y()));

    double aim_yaw_error = robot_utils::normalize_angle(current_yaw - final_aim_yaw);
    double aim_pitch_error = robot_utils::normalize_angle(current_pitch - final_aim_pitch);
    double true_yaw_error = robot_utils::normalize_angle(current_yaw - true_yaw[center_idx]);

    bool success = false;
    // Check if aim yaw, aim pitch, and true yaw are all within tolerance
    if (std::abs(aim_yaw_error) < aim_angle_tolerance_ && 
        std::abs(aim_pitch_error) < aim_angle_tolerance_ && 
        std::abs(true_yaw_error) < true_angle_tolerance_) {
        success = true;
    }

    // Assign to debug message (mapping error magnitudes to analogous fields)
    double true_angle_to_x = std::abs(true_yaw_error);
    double aim_angle_to_x = std::sqrt(aim_yaw_error*aim_yaw_error + aim_pitch_error*aim_pitch_error);

    robot_interfaces::msg::Aim aim_cmd;
    aim_cmd.header = msg->header;
    aim_cmd.yaw = final_aim_yaw;
    aim_cmd.pitch = final_aim_pitch;
    aim_cmd.w_yaw = final_w_yaw;
    aim_cmd.w_pitch = final_w_pitch;
	aim_cmd.target_number = 1;
    aim_cmd.success = success;
    
    aim_pub_->publish(aim_cmd);

    robot_interfaces::msg::BallisticsDebug debug_msg;
    debug_msg.header = msg->header;
    debug_msg.raw_yaw = aim_yaw[center_idx];
    debug_msg.raw_pitch = aim_pitch[center_idx];
    debug_msg.true_angle_to_x = true_angle_to_x;
    debug_msg.aim_angle_to_x = aim_angle_to_x;
    debug_msg.success = success;
    debug_pub_->publish(debug_msg);

    // Publish parabolic trajectory visualization
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = msg->header.stamp;
    marker.ns = "ballistics_trajectory";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.02; // Line width

    if (success) {
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0; // Green when allowed to hit
    } else {
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0; // Yellow otherwise
    }

    double vx = bullet_speed_ * std::cos(current_pitch) * std::cos(current_yaw);
    double vy = bullet_speed_ * std::cos(current_pitch) * std::sin(current_yaw);
    double vz = bullet_speed_ * std::sin(current_pitch);

    // Calculate trajectory points starting from the gimbal
    for (double t = 0; t <= 2.0; t += 0.05) {
        geometry_msgs::msg::Point p;
        p.x = gimbal_to_odom.transform.translation.x + vx * t;
        p.y = gimbal_to_odom.transform.translation.y + vy * t;
        p.z = gimbal_to_odom.transform.translation.z + vz * t - 0.5 * 9.8 * t * t;
        marker.points.push_back(p);
        
        // Stop drawing if it hits the ground
        if (p.z < 0.0) break;
    }

    marker_pub_->publish(marker);
}

} // namespace robot_ballistics

RCLCPP_COMPONENTS_REGISTER_NODE(robot_ballistics::BallisticsNode)
