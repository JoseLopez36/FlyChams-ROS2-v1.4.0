#include "rclcpp/rclcpp.hpp"

// Control includes
#include "flychams_control/drone/drone_state.hpp"
#include "flychams_control/drone/drone_motion.hpp"

// Core includes
#include "flychams_core/base/base_discoverer_node.hpp"

using namespace flychams::core;
using namespace flychams::control;

/**
 * ════════════════════════════════════════════════════════════════
 * @brief Control node for controlling the state and position
 * of the drones
 *
 * @details
 * This class implements the control node for controlling the state
 * and position of the drones. It uses the discoverer node to discover
 * each drone and then creates controllers for it.
 *
 * ════════════════════════════════════════════════════════════════
 * @author Jose Francisco Lopez Ruiz
 * @date 2025-02-28
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
        // Initialize drone controllers
        drone_state_.clear();
        drone_motion_.clear();
    }

    void onShutdown() override
    {
        // Destroy drone controllers
        drone_state_.clear();
        drone_motion_.clear();
    }

private: // Element management
    void onAddAgent(const ID& agent_id) override
    {
        // Create callback group for each drone control unit
        auto state_cb_group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto motion_cb_group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // Create drone controllers
        auto drone_state = std::make_shared<DroneState>(agent_id, node_, config_tools_, framework_tools_, topic_tools_, transform_tools_, state_cb_group);
        auto drone_motion = std::make_shared<DroneMotion>(agent_id, node_, config_tools_, framework_tools_, topic_tools_, transform_tools_, motion_cb_group);
        drone_state_.insert(std::make_pair(agent_id, drone_state));
        drone_motion_.insert(std::make_pair(agent_id, drone_motion));
        RCLCPP_INFO(node_->get_logger(), "Drone controllers created for agent %s", agent_id.c_str());

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
        // Destroy drone controllers
        drone_state_.erase(agent_id);
        drone_motion_.erase(agent_id);
        RCLCPP_INFO(node_->get_logger(), "Drone controllers destroyed for agent %s", agent_id.c_str());
    }

private: // Components
    // Drone controllers
    std::unordered_map<ID, DroneState::SharedPtr> drone_state_;
    std::unordered_map<ID, DroneMotion::SharedPtr> drone_motion_;
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