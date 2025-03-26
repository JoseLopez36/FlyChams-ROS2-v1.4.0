#pragma once

// Standard includes
#include <mutex>

// Tracking includes
#include "flychams_coordination/tracking/tracking_utils.hpp"

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
        AgentTracking(const core::ID& agent_id, core::NodePtr node, core::ConfigTools::SharedPtr config_tools, core::FrameworkTools::SharedPtr framework_tools, core::TopicTools::SharedPtr topic_tools, core::TransformTools::SharedPtr transform_tools)
            : BaseModule(node, config_tools, framework_tools, topic_tools, transform_tools), agent_id_(agent_id)
        {
            init();
        }

    protected: // Overrides
        void onInit() override;
        void onShutdown() override;

    public: // Types
        using SharedPtr = std::shared_ptr<AgentTracking>;

    private: // Parameters
        // Agent parameters
        core::ID agent_id_;
        core::ID central_head_id_;
        // Module parameters
        core::TrackingParameters tracking_params_;

    private: // Data
        // Odom
        core::Vector3r curr_pos_; 	// Current position (x, y, z)
        bool has_odom_;
        // Clusters
        std::pair<core::Matrix3Xr, core::RowVectorXr> clusters_;
        bool has_clusters_;
        // Computed tracking goal
        core::TrackingGoalMsg goal_;
        std::vector<core::Vector3r> prev_angles_;
        bool is_first_update_;
        // Odom and goal mutex
        std::mutex mutex_;

    private: // Methods
        // Callbacks
        void odomCallback(const core::OdometryMsg::SharedPtr msg);
        void infoCallback(const core::TrackingInfoMsg::SharedPtr msg);
        // Update
        void updateTracking();
        // Implementation
        void computeMultiCameraTracking(const core::Matrix3Xr& tab_P, const core::RowVectorXr& tab_r, core::TrackingGoalMsg& goal);
        void computeMultiWindowTracking(const core::Matrix3Xr& tab_P, const core::RowVectorXr& tab_r, core::TrackingGoalMsg& goal);

    private:
        // Subscribers
        core::SubscriberPtr<core::OdometryMsg> odom_sub_;
        core::SubscriberPtr<core::TrackingInfoMsg> info_sub_;
        // Publishers
        core::PublisherPtr<core::TrackingGoalMsg> goal_pub_;
        // Timers
        core::TimerPtr tracking_timer_;
    };

} // namespace flychams::coordination