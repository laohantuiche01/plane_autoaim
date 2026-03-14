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
        TransformStamped gimbalInWorld;
        gimbalInWorld.header.stamp = now();
        gimbalInWorld.header.frame_id = "odom";
        gimbalInWorld.child_frame_id = "gimbal";
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

        robot_interfaces::msg::Gimbal gimbal;
        gimbal.header.frame_id = "map";
        gimbal.header.stamp = now();
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
