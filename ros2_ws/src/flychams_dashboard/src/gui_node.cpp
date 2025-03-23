#include "rclcpp/rclcpp.hpp"

// Standard includes
#include <mutex>

// Dashboard includes
#include "flychams_dashboard/gui/gui_controller.hpp"

// Core includes
#include "flychams_core/base/base_discoverer_node.hpp"

using namespace flychams::core;
using namespace flychams::dashboard;

/**
 * ════════════════════════════════════════════════════════════════
 * @brief GUI node for the FlyingChameleons system
 *
 * @details
 * This class implements the GUI node for the FlyingChameleons system.
 * It uses the discoverer node to discover the different targets and then
 * creates a GUI controller for each target discovered.
 *
 * ════════════════════════════════════════════════════════════════
 * @author Jose Francisco Lopez Ruiz
 * @date 2025-03-01
 * ════════════════════════════════════════════════════════════════
 */
class GuiNode : public BaseDiscovererNode
{
public: // Constructor/Destructor
    GuiNode(const std::string& node_name, const rclcpp::NodeOptions& options)
        : BaseDiscovererNode(node_name, options)
    {
        // Nothing to do
    }

    void onInit() override
    {
        // Initialize selected agent
        selected_agent_id_ = "NONE";

        // Set update timer
        float update_rate = RosUtils::getParameterOr<float>(node_, "gui.gui_update_rate", 20.0f);
        update_timer_ = RosUtils::createTimerByRate(node_, update_rate,
            std::bind(&GuiNode::onUpdate, this));
    }

    void onShutdown() override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // Destroy GUI controllers
        gui_controllers_.clear();
    }

private: // Update methods
    void onUpdate()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (selected_agent_id_ != "NONE")
        {
            // Update GUI controller
            auto controller = gui_controllers_[selected_agent_id_];
            controller->setTrackingWindows();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            controller->drawOnCentralWindow();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

private: // Agent management
    void onAddAgent(const ID& agent_id) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // Create and add GUI controller
        auto controller = std::make_shared<GuiController>(agent_id, node_, config_tools_, ext_tools_, topic_tools_, tf_tools_);
        gui_controllers_.insert({ agent_id, controller });
        // Select agent if no agent is selected
        if (selected_agent_id_ == "NONE")
        {
            selected_agent_id_ = agent_id;
            onAgentSwitched();
        }
    }

    void onRemoveAgent(const ID& agent_id) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // Remove agent from GUI controller
        gui_controllers_.erase(agent_id);
        // Select new agent if it is the one being removed
        if (selected_agent_id_ == agent_id)
        {
            // Get first agent
            auto it = gui_controllers_.begin();
            selected_agent_id_ = it->first;
            onAgentSwitched();
        }
    }

    void onAgentSwitched()
    {
        // Update tracking windows
        if (selected_agent_id_ != "NONE")
        {
            auto controller = gui_controllers_[selected_agent_id_];
            controller->setFixedWindows();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            controller->setCentralWindow();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

private: // Components
    // Selected agent to display
    ID selected_agent_id_;
    // GUI controller per agent
    std::unordered_map<ID, GuiController::SharedPtr> gui_controllers_;
    // Mutex for GUI controllers
    std::mutex mutex_;
    // Timer
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
    // Create node
    auto node = std::make_shared<GuiNode>("gui_node", options);
    node->init();
    // Spin node
    rclcpp::spin(node);
    // Shutdown
    rclcpp::shutdown();
    return 0;
}