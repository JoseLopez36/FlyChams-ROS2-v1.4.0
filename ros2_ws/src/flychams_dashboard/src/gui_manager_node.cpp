#include "rclcpp/rclcpp.hpp"

// Dashboard includes
#include "flychams_dashboard/gui/gui_manager.hpp"

// Core includes
#include "flychams_core/base/base_discoverer_node.hpp"

using namespace flychams::core;
using namespace flychams::dashboard;

/**
 * ════════════════════════════════════════════════════════════════
 * @brief GUI manager node for the FlyingChameleons system
 *
 * @details
 * This class implements the GUI node for the FlyingChameleons system.
 * It uses the discoverer node to discover the different targets and then
 * creates a GUI manager for each target discovered.
 *
 * ════════════════════════════════════════════════════════════════
 * @author Jose Francisco Lopez Ruiz
 * @date 2025-03-01
 * ════════════════════════════════════════════════════════════════
 */
class GuiManagerNode : public BaseDiscovererNode
{
public: // Constructor/Destructor
    GuiManagerNode(const std::string& node_name, const rclcpp::NodeOptions& options)
        : BaseDiscovererNode(node_name, options)
    {
        // Nothing to do
    }

    void onInit() override
    {
        // Initialize agent IDs
        agent_ids_.clear();

        // Initialize selected agent
        selected_agent_id_ = "NONE";

        // Initialize GUI managers
        gui_managers_.clear();
    }

    void onShutdown() override
    {
        // Destroy agent IDs
        agent_ids_.clear();
        // Destroy GUI managers
        gui_managers_.clear();
    }

private: // Agent management
    void onAddAgent(const ID& agent_id) override
    {
        // Use callback group from discovery node (to avoid race conditions)
        // Create and add GUI manager
        auto manager = std::make_shared<GuiManager>(agent_id, node_, config_tools_, framework_tools_, topic_tools_, transform_tools_, discovery_cb_group_);
        gui_managers_.insert({ agent_id, manager });

        // Deactivate GUI manager
        manager->deactivate();

        // Add agent ID to list
        agent_ids_.push_back(agent_id);

        // Select agent if no agent is selected
        if (selected_agent_id_ == "NONE")
        {
            selected_agent_id_ = agent_id;
            onAgentSelected();
        }
    }

    void onRemoveAgent(const ID& agent_id) override
    {
        // Remove agent from GUI manager
        gui_managers_.erase(agent_id);

        // Remove agent ID from list
        agent_ids_.erase(std::find(agent_ids_.begin(), agent_ids_.end(), agent_id));

        // Select new agent if it is the one being removed
        if (selected_agent_id_ == agent_id)
        {
            // Get first agent
            auto it = gui_managers_.begin();
            selected_agent_id_ = it->first;
            onAgentSelected();
        }
    }

    void onAgentSelected()
    {
        // Update tracking windows
        if (selected_agent_id_ != "NONE")
        {
            // Activate selected GUI manager and deactivate the rest
            for (const auto& id : agent_ids_)
            {
                if (id == selected_agent_id_)
                {
                    gui_managers_[id]->activate();
                }
                else
                {
                    gui_managers_[id]->deactivate();
                }
            }
        }
    }

private: // Parameters
    float update_rate_;

private: // Data
    // Agent IDs
    std::vector<ID> agent_ids_;
    // Selected agent
    ID selected_agent_id_;

private: // Components
    // GUI manager per agent
    std::unordered_map<ID, GuiManager::SharedPtr> gui_managers_;
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
    auto node = std::make_shared<GuiManagerNode>("gui_manager_node", options);
    node->init();
    // Spin node
    rclcpp::spin(node);
    // Shutdown
    rclcpp::shutdown();
    return 0;
}