#ifndef ROBOT_AUTO_AIM__ARMOR_DETECTOR_NODE_HPP_
#define ROBOT_AUTO_AIM__ARMOR_DETECTOR_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <image_transport/image_transport.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "robot_auto_aim/armor_detector.hpp"
#include "robot_auto_aim/armor_pose_estimator.hpp"
#include "robot_interfaces/msg/armors.hpp"
#include "robot_interfaces/msg/debug_lights.hpp"
#include "robot_interfaces/msg/debug_armors.hpp"

namespace robot_auto_aim {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ArmorDetectorNode : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit ArmorDetectorNode(const rclcpp::NodeOptions & options);
    ~ArmorDetectorNode() override;

protected:
    CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & img_msg);

    std::unique_ptr<robot_auto_aim::Detector> initDetector();
    std::vector<robot_auto_aim::Armor> detectArmors(const sensor_msgs::msg::Image::ConstSharedPtr & img_msg);

    rcl_interfaces::msg::SetParametersResult onSetParameters(std::vector<rclcpp::Parameter> parameters);
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

    std::unique_ptr<robot_auto_aim::Detector> detector_;
    std::unique_ptr<robot_auto_aim::ArmorPoseEstimator> armor_pose_estimator_;
    bool use_ba_;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    cv::Point2f cam_center_;
    std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<robot_interfaces::msg::Armors>> armors_pub_;

    std::string odom_frame_;
    Eigen::Matrix3d imu_to_camera_;
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

    robot_interfaces::msg::Armors armors_msg_;
    visualization_msgs::msg::Marker armor_marker_;
    visualization_msgs::msg::Marker text_marker_;
    visualization_msgs::msg::MarkerArray marker_array_;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>> marker_pub_;

    bool debug_;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<robot_interfaces::msg::DebugLights>> lights_data_pub_;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<robot_interfaces::msg::DebugArmors>> armors_data_pub_;
    
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>> binary_img_pub_;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>> number_img_pub_;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>> result_img_pub_;

    double debug_img_freq_;
    rclcpp::Time last_debug_img_time_;

    int frame_count_;
    rclcpp::Time last_fps_time_;
    double total_detect_duration_ms_;

    void createDebugPublishers();
    void destroyDebugPublishers();
};

}  // namespace robot_auto_aim

#endif  // ROBOT_AUTO_AIM__ARMOR_DETECTOR_NODE_HPP_