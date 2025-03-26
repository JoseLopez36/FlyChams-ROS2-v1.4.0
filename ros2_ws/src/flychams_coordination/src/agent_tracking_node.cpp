#include "rclcpp/rclcpp.hpp"

// Coordination includes
#include "flychams_coordination/tracking/agent_tracking.hpp"

// Core includes
#include "flychams_core/base/base_discoverer_node.hpp"

using namespace flychams::core;
using namespace flychams::coordination;

/**
 * ════════════════════════════════════════════════════════════════
 * @brief Agent tracking node for tracking targets (clusters) in the simulation
 *
 * @details
 * This class implements the agent tracking node for tracking
 * targets (clusters) in the simulation. It uses the discoverer node to discover
 * the different targets and then creates a tracking system for each
 * target.
 *
 * ════════════════════════════════════════════════════════════════
 * @author Jose Francisco Lopez Ruiz
 * @date 2025-02-28
 * ════════════════════════════════════════════════════════════════
 */
class AgentTrackingNode : public BaseDiscovererNode
{
public: // Constructor/Destructor
    AgentTrackingNode(const std::string& node_name, const rclcpp::NodeOptions& options)
        : BaseDiscovererNode(node_name, options)
    {
        // Nothing to do
    }

    void onInit() override
    {
        // Initialize agent tracking systems
        agent_trackings_.clear();
    }

    void onShutdown() override
    {
        // Destroy agent tracking systems
        agent_trackings_.clear();
    }

private: // Element management
    void onAddAgent(const ID& agent_id) override
    {
        // Create agent tracking system
        agent_trackings_.insert({ agent_id,
        std::make_shared<AgentTracking>(agent_id, node_, config_tools_, framework_tools_, topic_tools_, transform_tools_) });
    }

    void onRemoveAgent(const ID& agent_id) override
    {
        // Destroy agent tracking system
        agent_trackings_.erase(agent_id);
    }

private: // Components
    // Agent tracking systems
    std::unordered_map<ID, AgentTracking::SharedPtr> agent_trackings_;
};

int main(int argc, char** argv)
{
    // Initialize ROS
    rclcpp::init(argc, argv);
    // Create node options
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    // Create node
    auto node = std::make_shared<AgentTrackingNode>("agent_tracking_node", options);
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