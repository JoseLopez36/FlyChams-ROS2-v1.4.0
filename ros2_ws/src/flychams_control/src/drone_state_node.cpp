#include "rclcpp/rclcpp.hpp"

// Control includes
#include "flychams_control/drone/drone_state.hpp"

// Core includes
#include "flychams_core/base/base_discoverer_node.hpp"

using namespace flychams::core;
using namespace flychams::control;

/**
 * ════════════════════════════════════════════════════════════════
 * @brief Control node for managing the state of the drones
 * ════════════════════════════════════════════════════════════════
 * @author Jose Francisco Lopez Ruiz
 * @date 2025-03-31
 * ════════════════════════════════════════════════════════════════
 */
class DroneStateNode : public BaseDiscovererNode
{
public: // Constructor/Destructor
    DroneStateNode(const std::string& node_name, const rclcpp::NodeOptions& options)
        : BaseDiscovererNode(node_name, options)
    {
        // Nothing to do
    }

    void onInit() override
    {
        // Initialize drone state
        drone_state_.clear();
    }

    void onShutdown() override
    {
        // Destroy drone state
        drone_state_.clear();
    }

private: // Element management
    void onAddAgent(const ID& agent_id) override
    {
        // Create callback group for each drone state
        auto state_cb_group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // Create drone state
        auto drone_state = std::make_shared<DroneState>(agent_id, node_, config_tools_, framework_tools_, topic_tools_, transform_tools_, state_cb_group);
        drone_state_.insert(std::make_pair(agent_id, drone_state));
        RCLCPP_INFO(node_->get_logger(), "Drone state created for agent %s", agent_id.c_str());

        // Arm drone. Try each 100ms until success
        while (drone_state->getStatus() != AgentStatus::ARMED)
        {
            drone_state->requestArm();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        // Takeoff drone. Try each 100ms until success
        while (drone_state->getStatus() != AgentStatus::TAKEN_OFF)
        {
            drone_state->requestTakeoff();
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        }
        // Hover drone. Try each 100ms until success
        while (drone_state->getStatus() != AgentStatus::HOVERED)
        {
            drone_state->requestHover();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        // Transition to tracking state. Try each 100ms until success
        while (drone_state->getStatus() != AgentStatus::TRACKING)
        {
            drone_state->requestTracking();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        // Now the drone is ready to move to the desired goal
    }

    void onRemoveAgent(const ID& agent_id) override
    {
        // Destroy drone state
        drone_state_.erase(agent_id);
        RCLCPP_INFO(node_->get_logger(), "Drone state destroyed for agent %s", agent_id.c_str());
    }

private: // Components
    // Drone state
    std::unordered_map<ID, DroneState::SharedPtr> drone_state_;
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
    auto node = std::make_shared<DroneStateNode>("drone_state_node", options);
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