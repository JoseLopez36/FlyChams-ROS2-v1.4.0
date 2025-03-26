#include "rclcpp/rclcpp.hpp"

// Dashboard includes
#include "flychams_dashboard/visualization/visualization_factory.hpp"

// Core includes
#include "flychams_core/base/base_discoverer_node.hpp"

using namespace flychams::core;
using namespace flychams::dashboard;

/**
 * ════════════════════════════════════════════════════════════════
 * @brief Visualization node for the FlyingChameleons system
 *
 * @details
 * This class implements the visualization node for the FlyingChameleons
 * system. It uses the discoverer node to discover the different elements
 * and then creates a visualization factory for each element discovered.
 *
 * ════════════════════════════════════════════════════════════════
 * @author Jose Francisco Lopez Ruiz
 * @date 2025-03-01
 * ════════════════════════════════════════════════════════════════
 */
class VisualizationNode : public BaseDiscovererNode
{
public: // Constructor/Destructor
    VisualizationNode(const std::string& node_name, const rclcpp::NodeOptions& options)
        : BaseDiscovererNode(node_name, options)
    {
        // Nothing to do
    }

    void onInit() override
    {
        // Initialize visualization factory
        visualization_factory_ = std::make_shared<VisualizationFactory>(node_, config_tools_, framework_tools_, topic_tools_, transform_tools_);
    }

    void onShutdown() override
    {
        // Destroy visualization factory
        visualization_factory_.reset();
    }

private: // Element management
    void onAddAgent(const ID& agent_id) override
    {
        // Add agent to visualization factory
        visualization_factory_->addAgent(agent_id);
    }

    void onRemoveAgent(const ID& agent_id) override
    {
        // Remove agent from visualization factory
        visualization_factory_->removeAgent(agent_id);
    }

    void onAddTarget(const ID& target_id) override
    {
        // Add target to visualization factory
        visualization_factory_->addTarget(target_id);
    }

    void onRemoveTarget(const ID& target_id) override
    {
        // Remove target from visualization factory
        visualization_factory_->removeTarget(target_id);
    }

    void onAddCluster(const ID& cluster_id) override
    {
        // Add cluster to visualization factory
        visualization_factory_->addCluster(cluster_id);
    }

    void onRemoveCluster(const ID& cluster_id) override
    {
        // Remove cluster from visualization factory
        visualization_factory_->removeCluster(cluster_id);
    }

private: // Components
    // Visualization factory
    VisualizationFactory::SharedPtr visualization_factory_;
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
    auto node = std::make_shared<VisualizationNode>("visualization_node", options);
    node->init();
    // Spin node
    rclcpp::spin(node);
    // Shutdown
    rclcpp::shutdown();
    return 0;
}