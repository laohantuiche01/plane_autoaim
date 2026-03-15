#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "robot_auto_aim/armor_solver_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<robot_auto_aim::ArmorSolverNode>(options);
  
  // 对于生命周期节点，在普通 main 函数中通常需要手动触发配置和激活
  node->configure();
  node->activate();

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
