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
        GuiController(const core::ID& agent_id, core::NodePtr node, core::ConfigTools::SharedPtr config_tools, core::ExternalTools::SharedPtr ext_tools, core::TopicTools::SharedPtr topic_tools, core::TfTools::SharedPtr tf_tools)
            : BaseModule(node, config_tools, ext_tools, topic_tools, tf_tools), agent_id_(agent_id)
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
        core::ID scene_window_id_;
        core::ID agent_1_window_id_;
        core::ID agent_2_window_id_;
        core::ID map_window_id_;
        core::ID central_window_id_;
        std::vector<core::ID> tracking_window_ids_;
        // Camera IDs (for fixed windows)
        core::ID scene_camera_id_;
        core::ID agent_1_camera_id_;
        core::ID agent_2_camera_id_;
        core::ID central_camera_id_;
        // Number of windows
        int num_windows_;

    private: // Data
        // Agent tracking goal
        core::TrackingGoalMsg goal_;
        bool has_goal_;
        // Tracking goal mutex
        std::mutex mutex_;
        // Command vectors
        core::IDs window_ids_;
        core::IDs vehicle_ids_;
        core::IDs camera_ids_;
        std::vector<int> crop_x_;
        std::vector<int> crop_y_;
        std::vector<int> crop_w_;
        std::vector<int> crop_h_;
        // Rectangle vectors (for drawing on central window)
        std::vector<core::PointMsg> rectangle_corners_;
        std::vector<core::PointMsg> rectangle_sizes_;
        core::ColorMsg rectangle_color_;
        float rectangle_thickness_;
        // String vectors (for drawing on central window)
        std::vector<std::string> strings_;
        std::vector<core::PointMsg> string_positions_;
        core::ColorMsg string_color_;
        float string_scale_;

    private: // Safe callbacks
        void trackingCallback(const core::TrackingGoalMsg::SharedPtr msg);

    public: // Public methods
        // Update
        void updateAgent();
        void updateTracking();

    private:
        // Subscribers
        core::SubscriberPtr<core::TrackingGoalMsg> tracking_sub_;
    };

} // namespace flychams::dashboard