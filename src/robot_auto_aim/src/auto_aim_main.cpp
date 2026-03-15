#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "robot_auto_aim/armor_detector_node.hpp"
#include "robot_auto_aim/armor_solver_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::NodeOptions options;

  auto detector_node = std::make_shared<robot_auto_aim::ArmorDetectorNode>(options);
  auto solver_node = std::make_shared<robot_auto_aim::ArmorSolverNode>(options);

  // 手动触发生命周期转换
  detector_node->configure();
  detector_node->activate();
  
  solver_node->configure();
  solver_node->activate();

  executor.add_node(detector_node->get_node_base_interface());
  executor.add_node(solver_node->get_node_base_interface());

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
