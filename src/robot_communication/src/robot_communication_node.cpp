//
// Created by mijiao on 24-4-4.
//
#include <rclcpp/rclcpp.hpp>
#include "robot_communication/robot_communication.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotCommunication>());
    rclcpp::shutdown();
}