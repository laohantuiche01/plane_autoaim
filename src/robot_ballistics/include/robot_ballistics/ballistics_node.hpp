#ifndef ROBOT_BALLISTICS__BALLISTICS_NODE_HPP_
#define ROBOT_BALLISTICS__BALLISTICS_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "robot_interfaces/msg/target_trajectory.hpp"
#include "robot_interfaces/msg/aim.hpp"
#include "robot_interfaces/msg/ballistics_debug.hpp"
#include "robot_utils/savitzky_golay.hpp"
#include "robot_utils/math_utils.hpp"

namespace robot_ballistics {

class BallisticsNode : public rclcpp::Node {
public:
    explicit BallisticsNode(const rclcpp::NodeOptions& options);

private:
    void trajectoryCallback(const robot_interfaces::msg::TargetTrajectory::SharedPtr msg);

    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Pub/Sub
    rclcpp::Subscription<robot_interfaces::msg::TargetTrajectory>::SharedPtr trajectory_sub_;
    rclcpp::Publisher<robot_interfaces::msg::Aim>::SharedPtr aim_pub_;
    rclcpp::Publisher<robot_interfaces::msg::BallisticsDebug>::SharedPtr debug_pub_;

    // Parameters
    double bullet_speed_;
    std::string gimbal_frame_;
    double true_angle_tolerance_;
    double aim_angle_tolerance_;

    bool enable_sg_yaw_;
    int sg_yaw_order_;
    bool enable_sg_pitch_;
    int sg_pitch_order_;

    // Parameters Callback
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;
};

} // namespace robot_ballistics

#endif // ROBOT_BALLISTICS__BALLISTICS_NODE_HPP_
