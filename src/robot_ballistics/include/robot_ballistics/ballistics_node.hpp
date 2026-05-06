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
#include "std_msgs/msg/float64.hpp"

#include "robot_interfaces/msg/target_trajectory.hpp"
#include "robot_interfaces/msg/aim.hpp"
#include "robot_interfaces/msg/ballistics_debug.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "robot_ballistics/ballistics_calculator.hpp"
#include <opencv2/core.hpp>

#include "robot_utils/savitzky_golay.hpp"
#include "robot_utils/math_utils.hpp"

#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>

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
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    //debug发布器
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr debug_pub_array_;

    // Debug image projection (ballistics trajectory → 2D overlay)
    bool imshow_ballistics_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_img_pub_;
    sensor_msgs::msg::Image::SharedPtr latest_image_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    // Parameters
    double bullet_speed_;
    std::string gimbal_frame_;
    double true_angle_tolerance_;
    double aim_angle_tolerance_;

    double hit_yaw_offset_;
    double hit_pitch_offset_;
    double aim_yaw_offset_;
    double aim_pitch_offset_;

    bool enable_sg_yaw_;
    int sg_yaw_order_;
    bool enable_sg_pitch_;
    int sg_pitch_order_;

    // 动态开火容差
    double tolerance_coefficient_;

    // 击发延时补偿
    double fire_delay_;

    // 解析法+SG融合前馈
    bool use_analytical_w_yaw_;
    double analytical_w_yaw_alpha_;

    // Parameters Callback
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;
};

} // namespace robot_ballistics

#endif // ROBOT_BALLISTICS__BALLISTICS_NODE_HPP_
