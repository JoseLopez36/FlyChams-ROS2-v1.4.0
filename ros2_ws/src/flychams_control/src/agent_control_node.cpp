#include "rclcpp/rclcpp.hpp"

// Control includes
#include "flychams_control/agent_control/uav_controller.hpp"
#include "flychams_control/agent_control/head_controller.hpp"

// Core includes
#include "flychams_core/base/base_discoverer_node.hpp"

using namespace flychams::core;
using namespace flychams::control;

/**
 * ════════════════════════════════════════════════════════════════
 * @brief Agent control node for controlling the different agents
 * in the simulation
 *
 * @details
 * This class implements the agent control node for controlling the
 * different agents in the simulation. It uses the discoverer node to
 * discover the different agents and then creates a controller for each
 * agent discovered.
 *
 * ════════════════════════════════════════════════════════════════
 * @author Jose Francisco Lopez Ruiz
 * @date 2025-02-28
 * ════════════════════════════════════════════════════════════════
 */
class AgentControlNode : public BaseDiscovererNode
{
public: // Constructor/Destructor
    AgentControlNode(const std::string& node_name, const rclcpp::NodeOptions& options)
        : BaseDiscovererNode(node_name, options)
    {
        // Nothing to do
    }

    void onInit() override
    {
        // Initialize agent controllers
        uav_controllers_.clear();
        head_controllers_.clear();
    }

    void onShutdown() override
    {
        // Destroy agent controllers
        uav_controllers_.clear();
        head_controllers_.clear();
    }

private: // Element management
    void onAddAgent(const ID& agent_id) override
    {
        // Create UAV controller
        auto uav_controller = std::make_shared<UAVController>(agent_id, node_, config_tools_, framework_tools_, topic_tools_, transform_tools_);
        uav_controllers_.insert(std::make_pair(agent_id, uav_controller));
        RCLCPP_INFO(node_->get_logger(), "UAV controller created for agent %s", agent_id.c_str());

        // Create head controller
        auto head_controller = std::make_shared<HeadController>(agent_id, node_, config_tools_, framework_tools_, topic_tools_, transform_tools_);
        head_controllers_.insert(std::make_pair(agent_id, head_controller));
        RCLCPP_INFO(node_->get_logger(), "Head controller created for agent %s", agent_id.c_str());
    }

    void onRemoveAgent(const ID& agent_id) override
    {
        // Destroy UAV controller
        uav_controllers_.erase(agent_id);
        RCLCPP_INFO(node_->get_logger(), "UAV controller destroyed for agent %s", agent_id.c_str());

        // Destroy head controller
        head_controllers_.erase(agent_id);
        RCLCPP_INFO(node_->get_logger(), "Head controller destroyed for agent %s", agent_id.c_str());
    }

private: // Components
    // UAV controllers
    std::unordered_map<ID, UAVController::SharedPtr> uav_controllers_;
    // Head controllers
    std::unordered_map<ID, HeadController::SharedPtr> head_controllers_;
};

int main(int argc, char** argv)
{
    // Initialize ROS
    rclcpp::init(argc, argv);
    // Create node options
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    // Create and initialize node
    auto node = std::make_shared<AgentControlNode>("agent_control_node", options);
    node->init();
    // Create executor and add node
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    // Spin node
    executor.spin();
    // Shutdown
    rclcpp::shutdown();
    return 0;
}