#pragma once

// Tracking includes
#include "flychams_coordination/tracking/head_solver.hpp"
#include "flychams_coordination/tracking/window_solver.hpp"

// Base module include
#include "flychams_core/base/base_module.hpp"

namespace flychams::coordination
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Manager to ensure constant tracking of targets
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-01-29
     * ════════════════════════════════════════════════════════════════
     */
    class AgentTracking : public core::BaseModule
    {
    public: // Constructor/Destructor
        AgentTracking(const core::ID& agent_id, core::NodePtr node, core::ConfigTools::SharedPtr config_tools, core::FrameworkTools::SharedPtr framework_tools, core::TopicTools::SharedPtr topic_tools, core::TransformTools::SharedPtr transform_tools, core::CallbackGroupPtr module_cb_group)
            : BaseModule(node, config_tools, framework_tools, topic_tools, transform_tools, module_cb_group), agent_id_(agent_id)
        {
            init();
        }

    protected: // Overrides
        void onInit() override;
        void onShutdown() override;

    public: // Types
        using SharedPtr = std::shared_ptr<AgentTracking>;
        struct Agent
        {
            // Status data
            core::AgentStatus status;
            bool has_status;
            // Clusters data
            core::AgentClustersMsg clusters;
            bool has_clusters;
            // Tracking setpoint messages
            core::AgentTrackingSetpointsMsg tracking_setpoints;
            core::GuiSetpointsMsg gui_setpoints;
            // Subscribers
            core::SubscriberPtr<core::AgentStatusMsg> status_sub;
            core::SubscriberPtr<core::AgentClustersMsg> clusters_sub;
            // Publisher
            core::PublisherPtr<core::AgentTrackingSetpointsMsg> tracking_setpoints_pub;
            core::PublisherPtr<core::GuiSetpointsMsg> gui_setpoints_pub;
            // Constructor
            Agent()
                : status(), has_status(false), clusters(), has_clusters(false), tracking_setpoints(),
                gui_setpoints(), status_sub(), clusters_sub(), tracking_setpoints_pub(), gui_setpoints_pub()
            {
            }
        };

    private: // Parameters
        core::ID agent_id_;
        float update_rate_;
        // Tracking parameters
        core::TrackingMode mode_;
        int n_heads_;
        int n_windows_;
        std::vector<core::HeadParameters> head_params_;
        std::vector<core::WindowParameters> window_params_;
        // Transform parameters
        std::string world_frame_;
        std::vector<std::string> optical_frames_;

    private: // Data
        // Agent
        Agent agent_;
        // Solvers
        std::vector<HeadSolver> head_solvers_;
        std::vector<WindowSolver> window_solvers_;

    private: // Callbacks
        void statusCallback(const core::AgentStatusMsg::SharedPtr msg);
        void positionCallback(const core::PointStampedMsg::SharedPtr msg);
        void clustersCallback(const core::AgentClustersMsg::SharedPtr msg);

    private: // Tracking management
        void update();

    private: // Tracking methods
        void initializeMultiCamera();
        void initializeMultiWindow();
        void updateMultiCamera(const core::Matrix3Xr& tab_P, const core::RowVectorXr& tab_r, const std::vector<core::Matrix4r>& tab_T);
        void updateMultiWindow(const core::Matrix3Xr& tab_P, const core::RowVectorXr& tab_r, const core::Matrix4r& central_T);

    private: // ROS components
        // Timer
        core::TimerPtr update_timer_;
    };

} // namespace flychams::coordination