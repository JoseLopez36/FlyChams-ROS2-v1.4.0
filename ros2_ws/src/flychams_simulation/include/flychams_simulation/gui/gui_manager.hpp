#pragma once

// Base module include
#include "flychams_core/base/base_module.hpp"

namespace flychams::simulation
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Manager for GUI
     *
     * @details
     * This class is responsible for controlling the different windows
     * in the GUI. It manages a single agent and its respective parameters.
     *
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-02-27
     * ════════════════════════════════════════════════════════════════
     */
    class GuiManager : public core::BaseModule
    {
    public: // Constructor/Destructor
        GuiManager(const core::ID& agent_id, core::NodePtr node, core::ConfigTools::SharedPtr config_tools, core::FrameworkTools::SharedPtr framework_tools, core::TopicTools::SharedPtr topic_tools, core::TransformTools::SharedPtr transform_tools, core::CallbackGroupPtr module_cb_group)
            : BaseModule(node, config_tools, framework_tools, topic_tools, transform_tools, module_cb_group), agent_id_(agent_id)
        {
            init();
        }

    protected: // Overrides
        void onInit() override;
        void onShutdown() override;

    public: // Types
        using SharedPtr = std::shared_ptr<GuiManager>;
        using WindowCmd = core::FrameworkTools::WindowCmd;
        using DrawCmd = core::FrameworkTools::DrawCmd;
        enum class GuiMode
        {
            IDLE,
            RESET,
            TRACKING
        };
        struct Agent
        {
            // Status data
            core::AgentStatus status;
            bool has_status;
            // Setpoints data
            bool has_gui_setpoints;
            // Subscribers
            core::SubscriberPtr<core::AgentStatusMsg> status_sub;
            core::SubscriberPtr<core::GuiSetpointsMsg> gui_setpoints_sub;
            // Constructor
            Agent()
                : status(), has_status(false), has_gui_setpoints(false),
                status_sub(), gui_setpoints_sub()
            {
            }
        };

    private: // Parameters
        core::ID agent_id_;
        float update_rate_;

    private: // Data
        // GUI mode
        GuiMode gui_mode_;
        // Agent
        Agent agent_;
        // Window commands
        std::vector<WindowCmd> simulation_window_cmds_; // Simulation window commands
        std::vector<WindowCmd> operator_window_cmds_;   // Operator window commands
        DrawCmd central_draw_cmd_;                      // Central draw command

    public: // Public methods
        void activate() { gui_mode_ = GuiMode::RESET; }
        void deactivate() { gui_mode_ = GuiMode::IDLE; }

    private: // Callbacks
        void statusCallback(const core::AgentStatusMsg::SharedPtr msg);
        void setpointsCallback(const core::GuiSetpointsMsg::SharedPtr msg);

    private: // GUI management
        void update();

    private: // GUI methods
        void setWindows(const std::vector<WindowCmd>& window_cmds);
        void resetWindows(std::vector<WindowCmd>& window_cmds);
        void drawWindow(const DrawCmd& draw_cmd);

    private: // ROS components
        // Timer
        core::TimerPtr update_timer_;
    };

} // namespace flychams::simulation