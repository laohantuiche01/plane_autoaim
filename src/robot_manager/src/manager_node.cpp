#include "robot_manager/manager_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

namespace robot_manager {

ManagerNode::ManagerNode(const rclcpp::NodeOptions& options)
    : Node("robot_manager", options), last_mode_(255) {
    
    detector_name_ = this->declare_parameter("detector_name", "armor_detector");
    solver_name_ = this->declare_parameter("solver_name", "armor_solver");

    mode_sub_ = this->create_subscription<robot_interfaces::msg::Mode>(
        "/robot/mode", 10, std::bind(&ManagerNode::modeCallback, this, std::placeholders::_1));

    detector_change_state_client_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
        detector_name_ + "/change_state");
    detector_get_state_client_ = this->create_client<lifecycle_msgs::srv::GetState>(
        detector_name_ + "/get_state");

    solver_change_state_client_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
        solver_name_ + "/change_state");
    solver_get_state_client_ = this->create_client<lifecycle_msgs::srv::GetState>(
        solver_name_ + "/get_state");

    RCLCPP_INFO(this->get_logger(), "Robot Manager Node started.");
}

void ManagerNode::modeCallback(const robot_interfaces::msg::Mode::SharedPtr msg) {
    if (msg->mode == last_mode_) {
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Mode changed to: %d", msg->mode);

    if (msg->mode == 0) {
        // Activate nodes
        updateNodeState(detector_name_, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
        updateNodeState(solver_name_, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    } else {
        // Deactivate nodes
        updateNodeState(detector_name_, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
        updateNodeState(solver_name_, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
    }

    last_mode_ = msg->mode;
}

uint8_t ManagerNode::getNodeState(const std::string& node_name) {
    auto client = (node_name == detector_name_) ? detector_get_state_client_ : solver_get_state_client_;
    
    if (!client->wait_for_service(std::chrono::milliseconds(100))) {
        return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    auto result_future = client->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future, std::chrono::milliseconds(500)) != 
        rclcpp::FutureReturnCode::SUCCESS) {
        return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    return result_future.get()->current_state.id;
}

void ManagerNode::updateNodeState(const std::string& node_name, uint8_t transition) {
    auto client = (node_name == detector_name_) ? detector_change_state_client_ : solver_change_state_client_;
    
    if (!client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(this->get_logger(), "Service %s not available", client->get_service_name());
        return;
    }

    // Check current state to see if transition is valid
    uint8_t current_state = getNodeState(node_name);
    
    if (transition == lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE) {
        if (current_state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) return;
        if (current_state == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
            // Need to configure first
            updateNodeState(node_name, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
            current_state = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
        }
    } else if (transition == lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE) {
        if (current_state != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) return;
    }

    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;

    RCLCPP_INFO(this->get_logger(), "Requesting transition %d for node %s", transition, node_name.c_str());
    
    auto result_future = client->async_send_request(request);
    // Note: async_send_request in a callback needs care if we use spin_until_future_complete.
    // However, since this is a simple manager and not high frequency, we'll wait briefly.
}

} // namespace robot_manager

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_manager::ManagerNode)
