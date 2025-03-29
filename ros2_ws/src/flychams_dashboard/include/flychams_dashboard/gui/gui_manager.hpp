#pragma once

// Base module include
#include "flychams_core/base/base_module.hpp"

namespace flychams::dashboard
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
            // Head setpoints messages
            core::AgentHeadSetpointsMsg head_setpoints;
            bool has_head_setpoints;
            // Window setpoints messages
            core::AgentWindowSetpointsMsg window_setpoints;
            bool has_window_setpoints;
            // Subscribers
            core::SubscriberPtr<core::AgentStatusMsg> status_sub;
            core::SubscriberPtr<core::AgentHeadSetpointsMsg> head_setpoints_sub;
            core::SubscriberPtr<core::AgentWindowSetpointsMsg> window_setpoints_sub;
            // Constructor
            Agent()
                : status(), has_status(false), head_setpoints(), has_head_setpoints(false),
                window_setpoints(), has_window_setpoints(false), status_sub(), head_setpoints_sub(),
                window_setpoints_sub()
            {
            }
        };
        struct WindowCmds
        {
            core::IDs window_ids;
            core::IDs vehicle_ids;
            core::IDs camera_ids;
            std::vector<int> crop_x;
            std::vector<int> crop_y;
            std::vector<int> crop_w;
            std::vector<int> crop_h;
        };
        struct DrawCmds
        {
            core::ID window_id;
            // Rectangles
            std::vector<core::PointMsg> rectangle_corners;
            std::vector<core::PointMsg> rectangle_sizes;
            core::ColorMsg rectangle_color;
            float rectangle_thickness;
            // Strings
            std::vector<core::PointMsg> string_positions;
            std::vector<std::string> string_texts;
            core::ColorMsg string_color;
            float string_scale;

            // Constructor
            DrawCmds()
                : window_id(), rectangle_corners(), rectangle_sizes(), rectangle_color(),
                rectangle_thickness(), string_positions(), string_texts(), string_color(),
                string_scale()
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
        WindowCmds simulation_window_cmds_; // Simulation window commands
        WindowCmds operator_window_cmds_;   // Operator window commands
        DrawCmds central_draw_cmds_;        // Central draw commands

    public: // Public methods
        void activate() { gui_mode_ = GuiMode::RESET; }
        void deactivate() { gui_mode_ = GuiMode::IDLE; }

    private: // Callbacks
        void statusCallback(const core::AgentStatusMsg::SharedPtr msg);
        void headSetpointsCallback(const core::AgentHeadSetpointsMsg::SharedPtr msg);
        void windowSetpointsCallback(const core::AgentWindowSetpointsMsg::SharedPtr msg);

    private: // GUI management
        void update();

    private: // GUI methods
        void setWindows(const WindowCmds& window_cmds);
        void resetWindows(const WindowCmds& window_cmds);
        void drawWindow(const DrawCmds& draw_cmds);
        void updateHeadSetpoints(const core::AgentHeadSetpointsMsg& setpoints);
        void updateWindowSetpoints(const core::AgentWindowSetpointsMsg& setpoints);

    private: // ROS components
        // Timer
        core::TimerPtr update_timer_;
    };

} // namespace flychams::dashboard