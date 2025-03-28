#include "rclcpp/rclcpp.hpp"

// Target includes
#include "flychams_targets/control/target_motion.hpp"

// Core includes
#include "flychams_core/base/base_discoverer_node.hpp"

using namespace flychams::core;
using namespace flychams::targets;

/**
 * ════════════════════════════════════════════════════════════════
 * @brief Target node for controlling the different targets in the
 * mission
 *
 * @details
 * This class implements the target node for controlling the different
 * targets in the mission. It uses the discoverer node to discover the
 * different targets and then creates controllers for each target
 * discovered.
 *
 * ════════════════════════════════════════════════════════════════
 * @author Jose Francisco Lopez Ruiz
 * @date 2025-03-16
 * ════════════════════════════════════════════════════════════════
 */
class TargetControlNode : public BaseDiscovererNode
{
public: // Constructor/Destructor
    TargetControlNode(const std::string& node_name, const rclcpp::NodeOptions& options)
        : BaseDiscovererNode(node_name, options)
    {
        // Nothing to do
    }

    void onInit() override
    {
        // Initialize target controllers
        target_motion_.clear();
    }

    void onShutdown() override
    {
        // Destroy target controllers
        target_motion_.clear();
    }

private: // Element management
    void onAddTarget(const ID& target_id) override
    {
        // Create callback group for target controllers
        auto motion_cb_group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // Create target controllers
        auto target_motion = std::make_shared<TargetMotion>(target_id, node_, config_tools_, framework_tools_, topic_tools_, transform_tools_, motion_cb_group);
        target_motion_.insert(std::make_pair(target_id, target_motion));
        RCLCPP_INFO(node_->get_logger(), "Target controllers created for target %s", target_id.c_str());
    }

    void onRemoveTarget(const ID& target_id) override
    {
        // Destroy target controllers
        target_motion_.erase(target_id);
        RCLCPP_INFO(node_->get_logger(), "Target controllers destroyed for target %s", target_id.c_str());
    }

private: // Components
    // Target controllers
    std::unordered_map<ID, TargetMotion::SharedPtr> target_motion_;
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
    auto node = std::make_shared<TargetControlNode>("target_control_node", options);
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