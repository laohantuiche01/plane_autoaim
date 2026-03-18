//
// Created by mijiao on 24-4-4.
//

#ifndef ROBOT_COMMUNICATION_ROBOT_COMMUNICATION_H
#define ROBOT_COMMUNICATION_ROBOT_COMMUNICATION_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <robot_interfaces/msg/aim.hpp>
#include <robot_interfaces/msg/gimbal.hpp>
#include <robot_interfaces/msg/mode.hpp>

#include "serial_pro/robot_comm.h"
#include "robot_message.h"
#include "robot_utils/math_utils.hpp"

class RobotCommunication : public rclcpp::Node {
private:
    using TransformStamped = geometry_msgs::msg::TransformStamped;
    robot::RobotSerial serial;
    rclcpp::Subscription<robot_interfaces::msg::Aim>::SharedPtr aimSubscription;
    rclcpp::Publisher<robot_interfaces::msg::Gimbal>::SharedPtr gimbalPublisher;
    rclcpp::Publisher<robot_interfaces::msg::Mode>::SharedPtr modePublisher;
    rclcpp::Publisher<TransformStamped>::SharedPtr gimbalTransformPublisher;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;

    // Camera TF Parameters
    double camera_x_ = 0.0;
    double camera_y_ = 0.0;
    double camera_z_ = 0.0;
    double camera_roll_ = 0.0;
    double camera_pitch_ = 0.0;
    double camera_yaw_ = 0.0;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

public:
    RobotCommunication();

    void aimCallback(const robot_interfaces::msg::Aim &aim) {
        aim_data_t aimData{};
        aimData.pitch = aim.pitch;
        aimData.yaw = aim.yaw;
        aimData.w_pitch = aim.w_pitch;
        aimData.w_yaw = aim.w_yaw;
        aimData.success = aim.success;
        aimData.target_number = aim.target_number;
        aimData.target_rate = aim.target_rate;
        if (!serial.write(0x81, aimData)) {
            RCLCPP_FATAL_STREAM(get_logger(), "Sending data failed!");
            exit(-1);
        }
    }
};

inline RobotCommunication::RobotCommunication() : Node("robot_communication") {
    declare_parameter("serial_name", "/dev/ttyACM0");
    
    // Declare Camera TF Parameters
    camera_x_ = declare_parameter("camera_x", 0.0);
    camera_y_ = declare_parameter("camera_y", 0.0);
    camera_z_ = declare_parameter("camera_z", 0.0);
    camera_roll_ = declare_parameter("camera_roll", 0.0);
    camera_pitch_ = declare_parameter("camera_pitch", 0.0);
    camera_yaw_ = declare_parameter("camera_yaw", 0.0);

    param_callback_handle_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &parameters) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            for (const auto &param : parameters) {
                if (param.get_name() == "camera_x") camera_x_ = param.as_double();
                else if (param.get_name() == "camera_y") camera_y_ = param.as_double();
                else if (param.get_name() == "camera_z") camera_z_ = param.as_double();
                else if (param.get_name() == "camera_roll") camera_roll_ = param.as_double();
                else if (param.get_name() == "camera_pitch") camera_pitch_ = param.as_double();
                else if (param.get_name() == "camera_yaw") camera_yaw_ = param.as_double();
            }
            return result;
        });

    gimbalPublisher = create_publisher<robot_interfaces::msg::Gimbal>("/robot/gimbal", 10);
    modePublisher = create_publisher<robot_interfaces::msg::Mode>("/robot/mode", 10);
    aimSubscription = create_subscription<robot_interfaces::msg::Aim>(
        "/robot/aim", 10,
        std::bind(&RobotCommunication::aimCallback, this, std::placeholders::_1));
    gimbalTransformPublisher = create_publisher<TransformStamped>("/robot/gimbal_transform", 1);
    tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    try {
        serial = std::move(robot::RobotSerial(get_parameter("serial_name").as_string(), 4000000));
    } catch (const sp::SerialException &) {
        RCLCPP_FATAL_STREAM(get_logger(),
                            "Open cdc device Failed! Please check out robot connection or permission!");
        exit(-1);
    }
    RCLCPP_INFO_STREAM(get_logger(), "Open cdc device success!");

    serial.registerCallback(0x14, [this](const gimbal_and_config_data_t &gimbalAndConfigData) {
        auto current_time = now();

        TransformStamped gimbalInWorld;
        gimbalInWorld.header.stamp = current_time;
        gimbalInWorld.header.frame_id = "odom";
        gimbalInWorld.child_frame_id = "gimbal_link"; // Changed from "gimbal" to match solver
        tf2::Quaternion quaternion;
        quaternion.setRPY(robot_utils::deg_to_rad(gimbalAndConfigData.roll),
                          robot_utils::deg_to_rad(-gimbalAndConfigData.pitch),
                          robot_utils::deg_to_rad(gimbalAndConfigData.yaw));
        gimbalInWorld.transform.rotation.x = quaternion.x();
        gimbalInWorld.transform.rotation.y = quaternion.y();
        gimbalInWorld.transform.rotation.z = quaternion.z();
        gimbalInWorld.transform.rotation.w = quaternion.w();
        tfBroadcaster->sendTransform(gimbalInWorld);
        gimbalTransformPublisher->publish(gimbalInWorld);

        // Publish dynamic TF from gimbal_link to camera_optical_frame
        TransformStamped camera_tf;
        camera_tf.header.stamp = current_time;
        camera_tf.header.frame_id = "gimbal_link";
        camera_tf.child_frame_id = "camera_optical_frame";
        camera_tf.transform.translation.x = camera_x_;
        camera_tf.transform.translation.y = camera_y_;
        camera_tf.transform.translation.z = camera_z_;

        tf2::Quaternion q_param, q_opt, q_total;
        q_param.setRPY(robot_utils::deg_to_rad(camera_roll_),
                       robot_utils::deg_to_rad(camera_pitch_),
                       robot_utils::deg_to_rad(camera_yaw_));
        // Transform from ROS standard (X forward) to OpenCV standard (Z forward)
        q_opt.setRPY(-M_PI/2, 0.0, -M_PI/2);
        q_total = q_param * q_opt;

        camera_tf.transform.rotation.x = q_total.x();
        camera_tf.transform.rotation.y = q_total.y();
        camera_tf.transform.rotation.z = q_total.z();
        camera_tf.transform.rotation.w = q_total.w();
        tfBroadcaster->sendTransform(camera_tf);

        robot_interfaces::msg::Gimbal gimbal;
        gimbal.header.frame_id = "map";
        gimbal.header.stamp = current_time;
        gimbal.roll = robot_utils::deg_to_rad(gimbalAndConfigData.roll);
        gimbal.pitch = robot_utils::deg_to_rad(gimbalAndConfigData.pitch);
        gimbal.yaw = robot_utils::deg_to_rad(gimbalAndConfigData.yaw);
        gimbalPublisher->publish(gimbal);

        robot_interfaces::msg::Mode mode;
        mode.mode = gimbalAndConfigData.mode;
        mode.is_pressing = gimbalAndConfigData.is_pressing;
        modePublisher->publish(mode);
        return 0;
    });

    serial.spin(true);
}
#endif //ROBOT_COMMUNICATION_ROBOT_COMMUNICATION_H
