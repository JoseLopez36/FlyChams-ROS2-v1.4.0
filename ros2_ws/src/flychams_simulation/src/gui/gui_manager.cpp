#include "flychams_simulation/gui/gui_manager.hpp"

using namespace flychams::core;

namespace flychams::simulation
{
    // ════════════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR: Constructor and destructor
    // ════════════════════════════════════════════════════════════════════════════

    void GuiManager::onInit()
    {
        // Get parameters from parameter server
        // Get update rate
        update_rate_ = RosUtils::getParameterOr<float>(node_, "gui_manager.update_rate", 20.0f);

        // Initialize GUI mode
        gui_mode_ = GuiMode::IDLE;

        // Initialize data
        agent_ = Agent();

        // Get head configuration
        const auto& [heads, n] = config_tools_->getHeads(agent_id_);

        // Get system config
        const auto& system_config = config_tools_->getSystem();

        // Initialize simulation window commands
        simulation_window_cmds_.clear();
        // Scenario window
        simulation_window_cmds_.push_back(WindowCmd(system_config.scenario_view_id, agent_id_, system_config.scenario_camera_id));
        // Agent window
        simulation_window_cmds_.push_back(WindowCmd(system_config.agent_view_id, agent_id_, system_config.agent_camera_id));
        // Payload window
        simulation_window_cmds_.push_back(WindowCmd(system_config.payload_view_id, agent_id_, system_config.payload_camera_id));
        // Map window
        simulation_window_cmds_.push_back(WindowCmd(system_config.map_view_id, agent_id_, system_config.map_camera_id));

        // Initialize operator window commands (empty cameras for now)
        operator_window_cmds_.clear();
        for (const auto& tracking_view_id : system_config.tracking_view_ids)
        {
            operator_window_cmds_.push_back(WindowCmd(tracking_view_id, agent_id_, ""));
        }

        // Initialize central head draw commands
        central_draw_cmd_ = DrawCmd();
        central_draw_cmd_.window_id = operator_window_cmds_[0].window_id;
        for (size_t i = 1; i < operator_window_cmds_.size(); i++)
        {
            // Draw parameters
            ColorMsg rectangle_color;
            rectangle_color.r = 0.0f;
            rectangle_color.g = 1.0f;
            rectangle_color.b = 1.0f;
            rectangle_color.a = 0.5f;
            const float rectangle_thickness = 2.0f;
            ColorMsg string_color;
            string_color.r = 0.0f;
            string_color.g = 1.0f;
            string_color.b = 1.0f;
            string_color.a = 0.5f;
            const float string_scale = 128.0f;
            PointMsg start_position;
            start_position.x = HUGE_VALF;
            start_position.y = HUGE_VALF;

            // Fill draw command
            central_draw_cmd_.rectangles.positions.push_back(start_position);
            central_draw_cmd_.rectangles.sizes.push_back(PointMsg());
            central_draw_cmd_.rectangles.color = rectangle_color;
            central_draw_cmd_.rectangles.thickness = rectangle_thickness;
            central_draw_cmd_.strings.positions.push_back(start_position);
            central_draw_cmd_.strings.texts.push_back("TW" + std::to_string(i));
            central_draw_cmd_.strings.color = string_color;
            central_draw_cmd_.strings.scale = string_scale;
        }

        // Create subscribers for agent status, head setpoints and window setpoints
        agent_.status_sub = topic_tools_->createAgentStatusSubscriber(agent_id_,
            std::bind(&GuiManager::statusCallback, this, std::placeholders::_1), sub_options_with_module_cb_group_);
        agent_.gui_setpoints_sub = topic_tools_->createGuiSetpointsSubscriber(agent_id_,
            std::bind(&GuiManager::setpointsCallback, this, std::placeholders::_1), sub_options_with_module_cb_group_);

        // Set update timer
        update_timer_ = RosUtils::createTimer(node_, update_rate_,
            std::bind(&GuiManager::update, this), module_cb_group_);
    }

    void GuiManager::onShutdown()
    {
        // Destroy agent data
        agent_.status_sub.reset();
        agent_.gui_setpoints_sub.reset();
        // Destroy update timer
        update_timer_.reset();
    }

    // ════════════════════════════════════════════════════════════════════════════
    // CALLBACKS: Callback functions
    // ════════════════════════════════════════════════════════════════════════════

    void GuiManager::statusCallback(const AgentStatusMsg::SharedPtr msg)
    {
        // Update agent status
        agent_.status = static_cast<AgentStatus>(msg->status);
        agent_.has_status = true;
    }

    void GuiManager::setpointsCallback(const GuiSetpointsMsg::SharedPtr msg)
    {
        // Get number of setpoints
        int n = static_cast<int>(msg->camera_ids.size());

        // Iterate through all setpoints
        for (int i = 0; i < n; i++)
        {
            // Get camera ID and crop
            const auto& camera_id = msg->camera_ids[i];
            const auto& crop = msg->crops[i];

            // Check if crop is out of bounds. If so, set empty image and skip this window
            if (crop.is_out_of_bounds)
            {
                operator_window_cmds_[i].camera_id = "";
                continue;
            }

            // Set camera ID
            operator_window_cmds_[i].camera_id = camera_id;

            // Update crop
            operator_window_cmds_[i].crop = crop;

            // Update rectangles (draw command)
            central_draw_cmd_.rectangles.positions[i].x = crop.x;
            central_draw_cmd_.rectangles.positions[i].y = crop.y;
            central_draw_cmd_.rectangles.sizes[i].x = crop.w;
            central_draw_cmd_.rectangles.sizes[i].y = crop.h;

            // Update strings (draw command)
            central_draw_cmd_.strings.positions[i].x = crop.x;
            central_draw_cmd_.strings.positions[i].y = crop.y - 200.0f;
        }

        // Update has_gui_setpoints flag
        agent_.has_gui_setpoints = true;
    }

    // ════════════════════════════════════════════════════════════════════════════
    // UPDATE: Update tracking
    // ════════════════════════════════════════════════════════════════════════════

    void GuiManager::update()
    {
        switch (gui_mode_)
        {
        case GuiMode::IDLE:
            // In this mode, we do nothing
            break;

        case GuiMode::RESET:
            // In this mode, we reset the windows
            // First, reset operator windows
            RCLCPP_INFO(node_->get_logger(), "GUI manager: Resetting windows for agent %s", agent_id_.c_str());
            resetWindows(operator_window_cmds_);
            // Then, set simulation windows
            setWindows(simulation_window_cmds_);
            // Finally, set mode to TRACKING
            gui_mode_ = GuiMode::TRACKING;
            break;

        case GuiMode::TRACKING:
            // In this mode, we update operator windows if the agent status is TRACKING
            RCLCPP_INFO(node_->get_logger(), "GUI manager: Updating operator windows for agent %s", agent_id_.c_str());
            // Check if we have a valid agent status
            if (!agent_.has_status)
            {
                RCLCPP_WARN(node_->get_logger(), "GUI manager: Agent %s has no status", agent_id_.c_str());
                return; // Skip updating operator windows if we don't have a valid agent status
            }

            // Check if we are in the correct state to track
            if (agent_.status != AgentStatus::TRACKING)
            {
                RCLCPP_WARN(node_->get_logger(), "GUI manager: Agent %s is not in the correct state to track",
                    agent_id_.c_str());
                return;
            }

            // Update GUI setpoints commands
            if (agent_.has_gui_setpoints)
            {
                RCLCPP_INFO(node_->get_logger(), "GUI manager: Updating GUI setpoints for agent %s", agent_id_.c_str());
                setWindows(operator_window_cmds_);
                drawWindow(central_draw_cmd_);
            }
            break;

        default:
            RCLCPP_ERROR(node_->get_logger(), "GUI manager: Invalid GUI mode: %d", static_cast<int>(gui_mode_));
            break;
        }
    }

    // ════════════════════════════════════════════════════════════════════════════
    // IMPLEMENTATION: Implementation methods for GUI management
    // ════════════════════════════════════════════════════════════════════════════

    void GuiManager::setWindows(const std::vector<WindowCmd>& window_cmds)
    {
        // Send commands to set window images
        framework_tools_->setWindows(window_cmds);
        // Delay to ensure GUI is updated
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    void GuiManager::resetWindows(std::vector<WindowCmd>& window_cmds)
    {
        // Empty source camera IDs
        for (auto& window_cmd : window_cmds)
        {
            window_cmd.camera_id = "";
        }

        // Send commands
        setWindows(window_cmds);
    }

    void GuiManager::drawWindow(const DrawCmd& draw_cmd)
    {
        // Send draw commands to the window
        framework_tools_->drawWindow(draw_cmd);
        // Delay to ensure GUI is updated
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

} // namespace flychams::simulation