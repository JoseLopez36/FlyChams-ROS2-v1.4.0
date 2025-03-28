#pragma once

// Tracking includes
#include "flychams_coordination/tracking/tracking_solver.hpp"

// Base module include
#include "flychams_core/base/base_module.hpp"

namespace flychams::coordination
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Controller for UAV agent position and yaw
     *
     * @details
     * This class is responsible for controlling the position and yaw
     * of UAV agents using PID controllers. It manages multiple agents
     * and their respective control parameters.
     *
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
            // Position data
            core::PointMsg position;
            bool has_position;
            // Clusters data
            core::AgentClustersMsg clusters;
            bool has_clusters;
            // Tracking setpoint messages
            core::AgentHeadSetpointsMsg head_setpoints;
            core::AgentWindowSetpointsMsg window_setpoints;
            // Subscribers
            core::SubscriberPtr<core::AgentStatusMsg> status_sub;
            core::SubscriberPtr<core::PointStampedMsg> position_sub;
            core::SubscriberPtr<core::AgentClustersMsg> clusters_sub;
            // Publisher
            core::PublisherPtr<core::AgentHeadSetpointsMsg> head_setpoints_pub;
            core::PublisherPtr<core::AgentWindowSetpointsMsg> window_setpoints_pub;
            // Constructor
            Agent()
                : status(), has_status(false), position(), has_position(false), clusters(),
                has_clusters(false), head_setpoints(), window_setpoints(), status_sub(),
                position_sub(), clusters_sub(), head_setpoints_pub(), window_setpoints_pub()
            {
            }
        };

    private: // Parameters
        core::ID agent_id_;
        float update_rate_;
        // Tracking parameters
        core::TrackingParameters tracking_params_;

    private: // Data
        // Agent
        Agent agent_;
        // Solvers
        std::vector<TrackingSolver> solvers_;

    private: // Callbacks
        void statusCallback(const core::AgentStatusMsg::SharedPtr msg);
        void positionCallback(const core::PointStampedMsg::SharedPtr msg);
        void clustersCallback(const core::AgentClustersMsg::SharedPtr msg);

    private: // Tracking management
        void update();

    private: // Tracking methods
        void computeMultiCamera(const core::Matrix3Xr& tab_P, const core::RowVectorXr& tab_r, core::AgentHeadSetpointsMsg& setpoints);
        void computeMultiWindow(const core::Matrix3Xr& tab_P, const core::RowVectorXr& tab_r, core::AgentWindowSetpointsMsg& setpoints);

    private: // ROS components
        // Timer
        core::TimerPtr update_timer_;
    };

} // namespace flychams::coordination