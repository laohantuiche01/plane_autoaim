#include <rclcpp/rclcpp.hpp>
#include "hik_camera_driver.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HikCameraDriver>());
    rclcpp::shutdown();
    return 0;
}
