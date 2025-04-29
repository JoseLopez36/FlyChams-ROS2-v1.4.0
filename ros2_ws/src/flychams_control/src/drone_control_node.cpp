#include "rclcpp/rclcpp.hpp"

// Control includes
#include "flychams_control/drone/drone_control.hpp"

// Core includes
#include "flychams_core/base/base_discoverer_node.hpp"

using namespace flychams::core;
using namespace flychams::control;

/**
 * ════════════════════════════════════════════════════════════════
 * @brief Control node for controlling each drone registered
 * ════════════════════════════════════════════════════════════════
 * @author Jose Francisco Lopez Ruiz
 * @date 2025-03-31
 * ════════════════════════════════════════════════════════════════
 */
class DroneControlNode : public BaseDiscovererNode
{
public: // Constructor/Destructor
    DroneControlNode(const std::string& node_name, const rclcpp::NodeOptions& options)
        : BaseDiscovererNode(node_name, options)
    {
        // Nothing to do
    }

    void onInit() override
    {
        // Initialize drone control
        drone_control_.clear();
    }

    void onShutdown() override
    {
        // Destroy drone control
        drone_control_.clear();
    }

private: // Element management
    void onAddAgent(const ID& agent_id) override
    {
        // Create callback group for each drone control
        auto control_cb_group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // Create drone control
        auto drone_control = std::make_shared<DroneControl>(agent_id, node_, config_tools_, framework_tools_, topic_tools_, transform_tools_, control_cb_group);
        drone_control_.insert(std::make_pair(agent_id, drone_control));
        RCLCPP_INFO(node_->get_logger(), "Drone control created for agent %s", agent_id.c_str());
    }

    void onRemoveAgent(const ID& agent_id) override
    {
        // Destroy drone control
        drone_control_.erase(agent_id);
        RCLCPP_INFO(node_->get_logger(), "Drone control destroyed for agent %s", agent_id.c_str());
    }

private: // Components
    // Drone control
    std::unordered_map<ID, DroneControl::SharedPtr> drone_control_;
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
    auto node = std::make_shared<DroneControlNode>("drone_control_node", options);
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