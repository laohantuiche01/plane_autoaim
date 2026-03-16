#ifndef ROBOT_MANAGER__MANAGER_NODE_HPP_
#define ROBOT_MANAGER__MANAGER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/mode.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

namespace robot_manager {

class ManagerNode : public rclcpp::Node {
public:
    explicit ManagerNode(const rclcpp::NodeOptions& options);

private:
    void modeCallback(const robot_interfaces::msg::Mode::SharedPtr msg);

    void updateNodeState(const std::string& node_name, uint8_t transition);
    uint8_t getNodeState(const std::string& node_name);

    rclcpp::Subscription<robot_interfaces::msg::Mode>::SharedPtr mode_sub_;
    
    // Clients for lifecycle management
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr detector_change_state_client_;
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr detector_get_state_client_;
    
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr solver_change_state_client_;
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr solver_get_state_client_;

    std::string detector_name_;
    std::string solver_name_;
    
    uint8_t last_mode_;
};

} // namespace robot_manager

#endif // ROBOT_MANAGER__MANAGER_NODE_HPP_
