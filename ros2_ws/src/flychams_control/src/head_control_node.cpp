#include "rclcpp/rclcpp.hpp"

// Control includes
#include "flychams_control/head/head_control.hpp"

// Core includes
#include "flychams_core/base/base_discoverer_node.hpp"

using namespace flychams::core;
using namespace flychams::control;

/**
 * ════════════════════════════════════════════════════════════════
 * @brief Control node for controlling the agent's heads (gimbal/
 * camera)
 * ════════════════════════════════════════════════════════════════
 * @author Jose Francisco Lopez Ruiz
 * @date 2025-02-28
 * ════════════════════════════════════════════════════════════════
 */
class HeadControlNode : public BaseDiscovererNode
{
public: // Constructor/Destructor
    HeadControlNode(const std::string& node_name, const rclcpp::NodeOptions& options)
        : BaseDiscovererNode(node_name, options)
    {
        // Nothing to do
    }

    void onInit() override
    {
        // Initialize head controllers
        head_control_.clear();
    }

    void onShutdown() override
    {
        // Destroy head controllers
        head_control_.clear();
    }

private: // Element management
    void onAddAgent(const ID& agent_id) override
    {
        // Create callback group for head control unit
        auto control_cb_group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // Create head controller
        auto head_control = std::make_shared<HeadControl>(agent_id, node_, config_tools_, framework_tools_, topic_tools_, transform_tools_, control_cb_group);
        head_control_.insert(std::make_pair(agent_id, head_control));
        RCLCPP_INFO(node_->get_logger(), "Head controller created for agent %s", agent_id.c_str());
    }

    void onRemoveAgent(const ID& agent_id) override
    {
        // Destroy head controller
        head_control_.erase(agent_id);
        RCLCPP_INFO(node_->get_logger(), "Head controller destroyed for agent %s", agent_id.c_str());
    }

private: // Components
    // Head controllers
    std::unordered_map<ID, HeadControl::SharedPtr> head_control_;
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
    auto node = std::make_shared<HeadControlNode>("head_control_node", options);
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