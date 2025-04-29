#include "rclcpp/rclcpp.hpp"

// Coordination includes
#include "flychams_coordination/analysis/agent_analysis.hpp"

// Core includes
#include "flychams_core/base/base_discoverer_node.hpp"

using namespace flychams::core;
using namespace flychams::coordination;

/**
 * ════════════════════════════════════════════════════════════════
 * @brief Agent analysis node for analyzing agent data
 * ════════════════════════════════════════════════════════════════
 * @author Jose Francisco Lopez Ruiz
 * @date 2025-03-28
 * ════════════════════════════════════════════════════════════════
 */
class AgentAnalysisNode : public BaseDiscovererNode
{
public: // Constructor/Destructor
    AgentAnalysisNode(const std::string& node_name, const rclcpp::NodeOptions& options)
        : BaseDiscovererNode(node_name, options)
    {
        // Nothing to do
    }

    void onInit() override
    {
        // Use callback group from discovery node (to avoid race conditions)
        // Initialize agent analysis system
        agent_analysis_ = std::make_shared<AgentAnalysis>(node_, config_tools_, framework_tools_, topic_tools_, transform_tools_, discovery_cb_group_);
    }

    void onShutdown() override
    {
        // Destroy agent analysis system
        agent_analysis_.reset();
    }

private: // Element management
    void onAddAgent(const ID& agent_id) override
    {
        // Add agent to analysis manager
        agent_analysis_->addAgent(agent_id);
    }

    void onRemoveAgent(const ID& agent_id) override
    {
        // Remove agent from analysis manager
        agent_analysis_->removeAgent(agent_id);
    }

    void onAddCluster(const ID& cluster_id) override
    {
        // Add cluster to analysis manager
        agent_analysis_->addCluster(cluster_id);
    }

    void onRemoveCluster(const ID& cluster_id) override
    {
        // Remove cluster from analysis manager
        agent_analysis_->removeCluster(cluster_id);
    }

private: // Components
    // Agent analysis system
    AgentAnalysis::SharedPtr agent_analysis_;
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
    auto node = std::make_shared<AgentAnalysisNode>("agent_analysis_node", options);
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