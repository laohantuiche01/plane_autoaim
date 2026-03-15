#ifndef ROBOT_AUTO_AIM__ARMOR_SOLVER_NODE_HPP_
#define ROBOT_AUTO_AIM__ARMOR_SOLVER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

#include "robot_auto_aim/tracker.hpp"
#include "robot_interfaces/msg/armors.hpp"
#include "robot_interfaces/msg/aim.hpp"
#include "robot_interfaces/msg/target_state.hpp"

namespace robot_auto_aim {

class ArmorSolverNode : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit ArmorSolverNode(const rclcpp::NodeOptions& options);

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

private:
    void armorsCallback(const robot_interfaces::msg::Armors::SharedPtr armors_msg);
    
    void timerCallback();
    
    void publishMarkers(const std::shared_ptr<Target>& target, const std_msgs::msg::Header& header);

    void updateTrackerParams();

    // Params
    std::string odom_frame_;
    double q_x_, q_y_, q_z_;
    double q_vx_, q_vy_, q_vz_;
    double q_yaw_, q_v_yaw_, q_geo_;
    double r_x_, r_y_, r_z_, r_yaw_, r_yaw_adaptive_factor_;
    double dist_scale_coeff_, z_scale_coeff_;
    bool adaptive_tracking_;
    double q_alpha_;
    double ukf_alpha_, ukf_beta_, ukf_kappa_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Pub/Sub
    rclcpp::Subscription<robot_interfaces::msg::Armors>::SharedPtr armors_sub_;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<robot_interfaces::msg::Aim>> aim_pub_;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<robot_interfaces::msg::TargetState>> target_state_pub_;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>> marker_pub_;

    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Params Callback
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

    // Tracker
    std::unique_ptr<Tracker> tracker_;
};

} // namespace robot_auto_aim

#endif // ROBOT_AUTO_AIM__ARMOR_SOLVER_NODE_HPP_
