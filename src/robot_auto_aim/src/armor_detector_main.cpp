#include <rclcpp/rclcpp.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include "robot_auto_aim/armor_detector_node.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    
    // 初始化 Lifecycle Node
    auto node = std::make_shared<robot_auto_aim::ArmorDetectorNode>(options);
    
    // 直接触发 Configure 和 Activate 以便于单节点调试运行
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();

    rclcpp::shutdown();
    return 0;
}