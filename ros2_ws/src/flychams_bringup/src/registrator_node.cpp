#include "rclcpp/rclcpp.hpp"

// Registration includes
#include "flychams_bringup/registration/agent_registration.hpp"
#include "flychams_bringup/registration/target_registration.hpp"
#include "flychams_bringup/registration/cluster_registration.hpp"

// Core includes
#include "flychams_core/base/base_registrator_node.hpp"

using namespace flychams::core;
using namespace flychams::bringup;

/**
 * ════════════════════════════════════════════════════════════════
 * @brief Registrator node for registering the different elements
 * in the simulation
 * ════════════════════════════════════════════════════════════════
 * @author Jose Francisco Lopez Ruiz
 * @date 2025-02-28
 * ════════════════════════════════════════════════════════════════
 */
class RegistratorNode : public BaseRegistratorNode
{
public: // Constructor/Destructor
    RegistratorNode(const std::string& node_name, const rclcpp::NodeOptions& options)
        : BaseRegistratorNode(node_name, options)
    {
        // Nothing to do
    }

    void onInit() override
    {
        // Use callback group from registration node (to avoid race conditions)
        // Create registration instances for each element type
        agent_registration_ = std::make_shared<AgentRegistration>(node_, config_tools_, framework_tools_, topic_tools_, transform_tools_, registration_cb_group_);
        target_registration_ = std::make_shared<TargetRegistration>(node_, config_tools_, framework_tools_, topic_tools_, transform_tools_, registration_cb_group_);
        cluster_registration_ = std::make_shared<ClusterRegistration>(node_, config_tools_, framework_tools_, topic_tools_, transform_tools_, registration_cb_group_);

        // Get all elements
        agents_ = agent_registration_->getAgents();
        targets_ = target_registration_->getTargets();
        clusters_ = cluster_registration_->getClusters();

        // Check if every element type is correctly registered
        if (agents_.empty())
        {
            RCLCPP_ERROR(node_->get_logger(), "No agents registered. Cannot setup the simulation");
            rclcpp::shutdown();
            return;
        }
        if (targets_.empty())
        {
            RCLCPP_ERROR(node_->get_logger(), "No targets registered. Cannot setup the simulation");
            rclcpp::shutdown();
            return;
        }
        if (clusters_.empty())
        {
            RCLCPP_ERROR(node_->get_logger(), "No clusters registered. Cannot setup the simulation");
            rclcpp::shutdown();
            return;
        }

        // Register all agents, targets and clusters
        for (const auto& agent_id : agents_)
            registerElement(agent_id, ElementType::Agent);
        for (const auto& target_id : targets_)
            registerElement(target_id, ElementType::Target);
        for (const auto& cluster_id : clusters_)
            registerElement(cluster_id, ElementType::Cluster);
    }

    void onShutdown() override
    {
        // Unregister all elements
        for (const auto& agent_id : agents_)
            unregisterElement(agent_id, ElementType::Agent);
        for (const auto& target_id : targets_)
            unregisterElement(target_id, ElementType::Target);
        for (const auto& cluster_id : clusters_)
            unregisterElement(cluster_id, ElementType::Cluster);

        // Clear elements
        agents_.clear();
        targets_.clear();
        clusters_.clear();

        // Destroy registration instances
        agent_registration_.reset();
        target_registration_.reset();
        cluster_registration_.reset();
    }

private: // Components
    // Registration instances
    AgentRegistration::SharedPtr agent_registration_;
    TargetRegistration::SharedPtr target_registration_;
    ClusterRegistration::SharedPtr cluster_registration_;

    // Elements
    IDs agents_;
    IDs targets_;
    IDs clusters_;
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
    auto node = std::make_shared<RegistratorNode>("registrator_node", options);
    node->init();
    // Spin node
    rclcpp::spin(node);
    // Shutdown
    rclcpp::shutdown();
    return 0;
}