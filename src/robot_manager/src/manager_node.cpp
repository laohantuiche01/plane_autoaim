#include "robot_manager/manager_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include <thread>
#include <future>

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

    RCLCPP_INFO(this->get_logger(), "Robot Manager Node started. Waiting to initialize armor nodes...");

    // Auto-start armor nodes asynchronously
    std::thread([this]() {
        // Wait for a short time to let the containers and other nodes spawn
        std::this_thread::sleep_for(std::chrono::seconds(2));
        RCLCPP_INFO(this->get_logger(), "Auto-activating armor nodes...");
        
        // Configure first
        updateNodeState(detector_name_, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
        updateNodeState(solver_name_, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
        
        // Then activate
        updateNodeState(detector_name_, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
        updateNodeState(solver_name_, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
        
        RCLCPP_INFO(this->get_logger(), "Armor nodes auto-activation complete.");
    }).detach();
}

void ManagerNode::modeCallback(const robot_interfaces::msg::Mode::SharedPtr msg) {
    if (msg->mode == last_mode_) {
        return;
    }

    uint8_t mode = msg->mode;
    last_mode_ = mode;

    std::thread([this, mode]() {
        RCLCPP_INFO(this->get_logger(), "Mode changed to: %d", mode);

        if (mode == 1) {
            // Activate nodes
            updateNodeState(detector_name_, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
            updateNodeState(solver_name_, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
        } else {
            // Deactivate nodes
            updateNodeState(detector_name_, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
            updateNodeState(solver_name_, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
        }
    }).detach();
}

uint8_t ManagerNode::getNodeState(const std::string& node_name) {
    auto client = (node_name == detector_name_) ? detector_get_state_client_ : solver_get_state_client_;
    
    if (!client->wait_for_service(std::chrono::milliseconds(100))) {
        return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    auto result_future = client->async_send_request(request);
    
    if (result_future.wait_for(std::chrono::milliseconds(500)) != std::future_status::ready) {
        return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    return result_future.get()->current_state.id;
}

void ManagerNode::updateNodeState(const std::string& node_name, uint8_t transition) {
    auto client = (node_name == detector_name_) ? detector_change_state_client_ : solver_change_state_client_;
    
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(this->get_logger(), "Service %s not available. Waiting...", client->get_service_name());
    }

    uint8_t current_state = getNodeState(node_name);
    
    if (transition == lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE) {
        if (current_state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) return;
        if (current_state == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
            updateNodeState(node_name, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
            current_state = getNodeState(node_name);
        }
    } else if (transition == lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE) {
        if (current_state != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) return;
    } else if (transition == lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE) {
        if (current_state != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) return;
    }

    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;

    RCLCPP_INFO(this->get_logger(), "Requesting transition %d for node %s", transition, node_name.c_str());
    
    auto result_future = client->async_send_request(request);
    
    if (result_future.wait_for(std::chrono::milliseconds(500)) != std::future_status::ready) {
        RCLCPP_ERROR(this->get_logger(), "Timeout waiting for state transition of %s", node_name.c_str());
    }
}

} // namespace robot_manager

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robot_manager::ManagerNode)