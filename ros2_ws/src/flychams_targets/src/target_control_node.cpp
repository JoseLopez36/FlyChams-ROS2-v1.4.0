#include "rclcpp/rclcpp.hpp"

// Standard includes
#include <mutex>

// Target includes
#include "flychams_targets/target_control/target_controller.hpp"

// Core includes
#include "flychams_core/base/base_discoverer_node.hpp"

using namespace flychams::core;
using namespace flychams::targets;

/**
 * ════════════════════════════════════════════════════════════════
 * @brief Target control node for controlling the different targets
 * in the mission
 *
 * @details
 * This class implements the target control node for controlling the
 * different targets in the mission. It uses the discoverer node to
 * discover the different targets and then creates a controller for each
 * target discovered.
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
        target_controllers_.clear();

        // Remove all targets from simulation
        ext_tools_->removeAllTargets();

        // Wait 2 seconds to ensure targets are removed
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // Set target update timer
        prev_time_ = RosUtils::getTimeNow(node_);
        const auto& update_rate = RosUtils::getParameter<float>(node_, "target_control.target_update_rate");
        update_timer_ = RosUtils::createTimerByRate(node_, update_rate,
            std::bind(&TargetControlNode::onUpdate, this));
    }

    void onShutdown() override
    {
        // Lock mutex
        std::lock_guard<std::mutex> lock(mutex_);

        // Destroy target controllers
        target_controllers_.clear();
        // Destroy update timer
        update_timer_.reset();
    }

private: // Element management
    void onAddTarget(const ID& target_id) override
    {
        // Lock mutex
        std::lock_guard<std::mutex> lock(mutex_);

        // Create target controller
        auto controller = std::make_shared<TargetController>(target_id, node_, config_tools_, ext_tools_, topic_tools_, tf_tools_);

        // Add target data to vectors
        target_ids_.push_back(target_id);
        target_controllers_.push_back(controller);
        target_positions_.push_back(controller->getPosition());

        RCLCPP_INFO(node_->get_logger(), "Target controller created for target %s", target_id.c_str());

        // Add target to simulation
        ColorMsg highlight_color;
        highlight_color.r = 1.0f;
        highlight_color.g = 0.0f;
        highlight_color.b = 0.0f;
        highlight_color.a = 0.015f;
        PointMsg position = controller->getPosition();
        position.z += 4.0f;
        ext_tools_->addTargetGroup({ target_id }, { config_tools_->getTarget(target_id)->target_type }, { position }, config_tools_->getSimulation()->draw_world_markers, { highlight_color }, config_tools_->getMap()->region_type);

        // Wait for 0.1 seconds to ensure target is added
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    void onRemoveTarget(const ID& target_id) override
    {
        // Lock mutex
        std::lock_guard<std::mutex> lock(mutex_);

        // Find target controller index
        const auto index = std::find(target_ids_.begin(), target_ids_.end(), target_id);
        if (index == target_ids_.end())
            return;

        // Calculate the index for erasure
        size_t erase_index = std::distance(target_ids_.begin(), index);

        // Remove target data from vectors
        target_ids_.erase(target_ids_.begin() + erase_index);
        target_controllers_.erase(target_controllers_.begin() + erase_index);
        target_positions_.erase(target_positions_.begin() + erase_index);

        RCLCPP_INFO(node_->get_logger(), "Target controller destroyed for target %s", target_id.c_str());
    }

private: // Update methods
    void onUpdate()
    {
        // Lock mutex
        std::lock_guard<std::mutex> lock(mutex_);

        // Get current time
        const auto current_time = RosUtils::getTimeNow(node_);
        const float dt = (current_time - prev_time_).seconds();
        prev_time_ = current_time;

        // Cycle through all target controllers
        for (int i = 0; i < target_ids_.size(); i++)
        {
            // Update target controller
            target_controllers_[i]->update(dt);

            // Update target position
            target_positions_[i] = target_controllers_[i]->getPosition();
        }

        // Update targets in simulation
        ext_tools_->updateTargetGroup(target_ids_, target_positions_);
    }

private: // Components
    // Target data
    std::vector<ID> target_ids_;
    std::vector<PointMsg> target_positions_;
    // Target controllers
    std::vector<TargetController::SharedPtr> target_controllers_;
    // Mutex
    std::mutex mutex_;
    // Time management
    Time prev_time_;
    TimerPtr update_timer_;
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