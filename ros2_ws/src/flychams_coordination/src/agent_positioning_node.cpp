#include "rclcpp/rclcpp.hpp"

// Coordination includes
#include "flychams_coordination/positioning/agent_positioning.hpp"

// Core includes
#include "flychams_core/base/base_discoverer_node.hpp"

using namespace flychams::core;
using namespace flychams::coordination;

/**
 * ════════════════════════════════════════════════════════════════
 * @brief Agent positioning node for positioning agents in the 
 * simulation
 * ════════════════════════════════════════════════════════════════
 * @author Jose Francisco Lopez Ruiz
 * @date 2025-02-28
 * ════════════════════════════════════════════════════════════════
 */
class AgentPositioningNode : public BaseDiscovererNode
{
public: // Constructor/Destructor
    AgentPositioningNode(const std::string& node_name, const rclcpp::NodeOptions& options)
        : BaseDiscovererNode(node_name, options)
    {
        // Nothing to do
    }

    void onInit() override
    {
        // Initialize agent positioning systems
        agent_positionings_.clear();
    }

    void onShutdown() override
    {
        // Destroy agent positioning systems
        agent_positionings_.clear();
    }

private: // Element management
    void onAddAgent(const ID& agent_id) override
    {
        // Create callback group for each agent positioning system
        auto positioning_cb_group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // Create agent positioning system
        agent_positionings_.insert({ agent_id,
            std::make_shared<AgentPositioning>(agent_id, node_, config_tools_, framework_tools_, topic_tools_, transform_tools_, positioning_cb_group) });
    }

    void onRemoveAgent(const ID& agent_id) override
    {
        // Destroy agent positioning system
        agent_positionings_.erase(agent_id);
    }

private: // Components
    // Agent positioning system
    std::unordered_map<ID, AgentPositioning::SharedPtr> agent_positionings_;
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
    auto node = std::make_shared<AgentPositioningNode>("agent_positioning_node", options);
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