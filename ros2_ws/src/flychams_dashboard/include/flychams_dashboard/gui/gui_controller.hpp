#pragma once

// Standard includes
#include <mutex>
#include <set>

// Base module include
#include "flychams_core/base/base_module.hpp"

namespace flychams::dashboard
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Controller for GUI
     *
     * @details
     * This class is responsible for controlling the GUI. It manages
     * multiple agents and their respective control parameters.
     *
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-02-27
     * ════════════════════════════════════════════════════════════════
     */
    class GuiController : public core::BaseModule
    {
    public: // Constructor/Destructor
        GuiController(const core::ID& agent_id, core::NodePtr node, core::ConfigTools::SharedPtr config_tools, core::FrameworkTools::SharedPtr framework_tools, core::TopicTools::SharedPtr topic_tools, core::TransformTools::SharedPtr transform_tools)
            : BaseModule(node, config_tools, framework_tools, topic_tools, transform_tools), agent_id_(agent_id)
        {
            init();
        }

    protected: // Overrides
        void onInit() override;
        void onShutdown() override;

    public: // Types
        using SharedPtr = std::shared_ptr<GuiController>;

    private: // Parameters
        // Agent parameters
        core::ID agent_id_;
        // Window IDs
        core::IDs fixed_window_ids_;
        core::ID central_window_id_;
        core::IDs tracking_window_ids_;
        int num_fixed_windows_;
        int num_tracking_windows_;
        int num_windows_;
        // Camera IDs
        core::IDs fixed_camera_ids_;
        core::ID central_camera_id_;

    private: // Data
        // Agent tracking goal
        core::TrackingGoalMsg goal_;
        bool has_goal_;
        // Tracking goal mutex
        std::mutex mutex_;
        // Tracking command vectors
        core::IDs tracking_vehicle_id_cmds_;
        core::IDs tracking_camera_id_cmds_;
        std::vector<int> tracking_crop_x_cmds_;
        std::vector<int> tracking_crop_y_cmds_;
        std::vector<int> tracking_crop_w_cmds_;
        std::vector<int> tracking_crop_h_cmds_;
        // Digital tracking draw data
        std::vector<core::PointMsg> dig_rect_corners_;
        std::vector<core::PointMsg> dig_rect_sizes_;
        core::ColorMsg dig_rect_color_;
        float dig_rect_thickness_;
        std::vector<core::PointMsg> dig_string_pos_;
        std::vector<std::string> dig_strings_;
        core::ColorMsg dig_string_color_;
        float dig_string_scale_;

    private: // Safe callbacks
        void trackingCallback(const core::TrackingGoalMsg::SharedPtr msg);

    public: // Public methods
        // Update
        void setFixedWindows();
        void setCentralWindow();
        void setTrackingWindows();
        void drawOnCentralWindow();

    private:
        // Subscribers
        core::SubscriberPtr<core::TrackingGoalMsg> tracking_sub_;
    };

} // namespace flychams::dashboard