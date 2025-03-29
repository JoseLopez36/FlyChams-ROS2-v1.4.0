#include "flychams_dashboard/gui/gui_manager.hpp"

using namespace flychams::core;

namespace flychams::dashboard
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

        // Get system config
        const auto& system_config = config_tools_->getSystem();

        // Initialize simulation window commands
        simulation_window_cmds_ = WindowCmds();
        // Scenario window
        simulation_window_cmds_.window_ids.push_back(system_config.scenario_view_id);
        simulation_window_cmds_.camera_ids.push_back(system_config.scenario_camera_id);
        simulation_window_cmds_.vehicle_ids.push_back(agent_id_);
        // Agent window
        simulation_window_cmds_.window_ids.push_back(system_config.agent_view_id);
        simulation_window_cmds_.camera_ids.push_back(system_config.agent_camera_id);
        simulation_window_cmds_.vehicle_ids.push_back(agent_id_);
        // Payload window
        simulation_window_cmds_.window_ids.push_back(system_config.payload_view_id);
        simulation_window_cmds_.camera_ids.push_back(system_config.payload_camera_id);
        simulation_window_cmds_.vehicle_ids.push_back(agent_id_);
        // Map window
        simulation_window_cmds_.window_ids.push_back(system_config.map_view_id);
        simulation_window_cmds_.camera_ids.push_back(system_config.map_camera_id);
        simulation_window_cmds_.vehicle_ids.push_back(agent_id_);

        // Initialize operator window commands
        operator_window_cmds_ = WindowCmds();
        // Central window
        operator_window_cmds_.window_ids.push_back(system_config.central_view_id);
        operator_window_cmds_.camera_ids.push_back(config_tools_->getCentralHead(agent_id_)->id);
        operator_window_cmds_.vehicle_ids.push_back(agent_id_);
        // Tracking windows
        for (const auto& tracking_view_id : system_config.tracking_view_ids)
        {
            operator_window_cmds_.window_ids.push_back(tracking_view_id);
            operator_window_cmds_.camera_ids.push_back("");
            operator_window_cmds_.vehicle_ids.push_back(agent_id_);
        }

        // Initialize central head draw commands
        central_draw_cmds_ = DrawCmds();
        central_draw_cmds_.window_id = operator_window_cmds_.window_ids[0];
        for (size_t i = 0; i < system_config.tracking_view_ids.size(); i++)
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

            central_draw_cmds_.rectangle_corners.push_back(PointMsg());
            central_draw_cmds_.rectangle_sizes.push_back(PointMsg());
            central_draw_cmds_.rectangle_color = rectangle_color;
            central_draw_cmds_.rectangle_thickness = rectangle_thickness;
            central_draw_cmds_.string_positions.push_back(PointMsg());
            central_draw_cmds_.string_texts.push_back("TW" + std::to_string(i));
            central_draw_cmds_.string_color = string_color;
            central_draw_cmds_.string_scale = string_scale;
        }

        // Create subscribers for agent status, head setpoints and window setpoints
        agent_.status_sub = topic_tools_->createAgentStatusSubscriber(agent_id_,
            std::bind(&GuiManager::statusCallback, this, std::placeholders::_1), sub_options_with_module_cb_group_);
        agent_.head_setpoints_sub = topic_tools_->createAgentHeadSetpointsSubscriber(agent_id_,
            std::bind(&GuiManager::headSetpointsCallback, this, std::placeholders::_1), sub_options_with_module_cb_group_);
        agent_.window_setpoints_sub = topic_tools_->createAgentWindowSetpointsSubscriber(agent_id_,
            std::bind(&GuiManager::windowSetpointsCallback, this, std::placeholders::_1), sub_options_with_module_cb_group_);

        // Set update timer
        update_timer_ = RosUtils::createTimer(node_, update_rate_,
            std::bind(&GuiManager::update, this), module_cb_group_);
    }

    void GuiManager::onShutdown()
    {
        // Destroy agent data
        agent_.status_sub.reset();
        agent_.head_setpoints_sub.reset();
        agent_.window_setpoints_sub.reset();
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

    void GuiManager::headSetpointsCallback(const AgentHeadSetpointsMsg::SharedPtr msg)
    {
        // Update agent head setpoints
        agent_.head_setpoints = *msg;
        agent_.has_head_setpoints = true;
    }

    void GuiManager::windowSetpointsCallback(const AgentWindowSetpointsMsg::SharedPtr msg)
    {
        // Update agent window setpoints
        agent_.window_setpoints = *msg;
        agent_.has_window_setpoints = true;
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
            resetWindows(operator_window_cmds_);
            // Then, set simulation windows
            setWindows(simulation_window_cmds_);
            // Finally, set mode to TRACKING
            gui_mode_ = GuiMode::TRACKING;
            break;

        case GuiMode::TRACKING:
            // In this mode, we update operator windows if the agent status is TRACKING
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

            // Update head setpoints commands
            if (agent_.has_head_setpoints)
            {
                updateHeadSetpoints(agent_.head_setpoints);
            }

            // Update window setpoints commands
            if (agent_.has_window_setpoints)
            {
                updateWindowSetpoints(agent_.window_setpoints);
            }

            // Set operator window images
            setWindows(operator_window_cmds_);

            // Send draw commands to the central window
            drawWindow(central_draw_cmds_);
            break;

        default:
            RCLCPP_ERROR(node_->get_logger(), "GUI manager: Invalid GUI mode: %d", gui_mode_);
            break;
        }
    }

    // ════════════════════════════════════════════════════════════════════════════
    // IMPLEMENTATION: Implementation methods for GUI management
    // ════════════════════════════════════════════════════════════════════════════

    void GuiManager::setWindows(const WindowCmds& window_cmds)
    {
        // Send commands to set window images
        framework_tools_->setWindowImageGroup(window_cmds.window_ids,
            window_cmds.vehicle_ids, window_cmds.camera_ids,
            window_cmds.crop_x, window_cmds.crop_y,
            window_cmds.crop_w, window_cmds.crop_h);
        // Delay to ensure GUI is updated
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    void GuiManager::resetWindows(const WindowCmds& window_cmds)
    {
        // Send empty commands to windows
        int n = window_cmds.window_ids.size();
        framework_tools_->setWindowImageGroup(window_cmds.window_ids,
            std::vector<ID>(n, ""),
            std::vector<ID>(n, ""),
            std::vector<int>(n, 0),
            std::vector<int>(n, 0),
            std::vector<int>(n, 0),
            std::vector<int>(n, 0));
        // Delay to ensure GUI is updated
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    void GuiManager::drawWindow(const DrawCmds& draw_cmds)
    {
        // Send rectangle draw commands to the window
        framework_tools_->setWindowRectangles(draw_cmds.window_id,
            draw_cmds.rectangle_corners, draw_cmds.rectangle_sizes,
            draw_cmds.rectangle_color, draw_cmds.rectangle_thickness);
        // Delay to ensure GUI is updated
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        // Send string draw commands to the window
        framework_tools_->setWindowStrings(draw_cmds.window_id,
            draw_cmds.string_texts, draw_cmds.string_positions,
            draw_cmds.string_color, draw_cmds.string_scale);
        // Delay to ensure GUI is updated
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    void GuiManager::updateHeadSetpoints(const AgentHeadSetpointsMsg& setpoints)
    {
        for (int i = 0; i < setpoints.head_ids.size(); i++)
        {
            // Set camera ID as head ID
            operator_window_cmds_.camera_ids[i] = setpoints.head_ids[i];
            // No crop
            operator_window_cmds_.crop_x[i] = 0;
            operator_window_cmds_.crop_y[i] = 0;
            operator_window_cmds_.crop_w[i] = 0;
            operator_window_cmds_.crop_h[i] = 0;
        }
    }

    void GuiManager::updateWindowSetpoints(const AgentWindowSetpointsMsg& setpoints)
    {
        for (int i = 0; i < setpoints.crop_setpoints.size(); i++)
        {
            // Get setpoint crop
            const CropMsg crop = setpoints.crop_setpoints[i];
            // Check if crop is out of bounds. If so, set empty image and skip this window
            if (crop.is_out_of_bounds)
            {
                operator_window_cmds_.camera_ids[i] = "";
                continue;
            }
            // Set camera ID as central head ID
            operator_window_cmds_.camera_ids[i] = operator_window_cmds_.camera_ids[0];
            // Update crop
            operator_window_cmds_.crop_x[i] = crop.x;
            operator_window_cmds_.crop_y[i] = crop.y;
            operator_window_cmds_.crop_w[i] = crop.w;
            operator_window_cmds_.crop_h[i] = crop.h;
            // Update draw rectangles
            central_draw_cmds_.rectangle_corners[i].x = crop.x;
            central_draw_cmds_.rectangle_corners[i].y = crop.y;
            central_draw_cmds_.rectangle_sizes[i].x = crop.w;
            central_draw_cmds_.rectangle_sizes[i].y = crop.h;
            // Update draw strings
            central_draw_cmds_.string_positions[i].x = crop.x;
            central_draw_cmds_.string_positions[i].y = crop.y - 200.0f;
        }
    }

} // namespace flychams::dashboard