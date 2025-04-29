#pragma once

// Base module include
#include "flychams_core/base/base_module.hpp"

namespace flychams::control
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief State manager for UAV drones
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-03-26
     * ════════════════════════════════════════════════════════════════
     */
    class DroneState : public core::BaseModule
    {
    public: // Constructor/Destructor
        DroneState(const core::ID& agent_id, core::NodePtr node, core::ConfigTools::SharedPtr config_tools, core::FrameworkTools::SharedPtr framework_tools, core::TopicTools::SharedPtr topic_tools, core::TransformTools::SharedPtr transform_tools, core::CallbackGroupPtr module_cb_group)
            : BaseModule(node, config_tools, framework_tools, topic_tools, transform_tools, module_cb_group), agent_id_(agent_id)
        {
            init();
        }

    protected: // Overrides
        void onInit() override;
        void onShutdown() override;

    public: // Types
        using SharedPtr = std::shared_ptr<DroneState>;
        struct Agent
        {
            // Odometry data
            core::OdometryMsg odom;
            bool has_odom;
            // Status data
            core::AgentStatus status;
            // Status message
            core::AgentStatusMsg status_msg;
            // Position message
            core::PointStampedMsg position_msg;
            // Subscriber
            core::SubscriberPtr<core::OdometryMsg> odom_sub;
            // Publishers
            core::PublisherPtr<core::AgentStatusMsg> status_pub;
            core::PublisherPtr<core::PointStampedMsg> position_pub;
            // Constructor
            Agent()
                : odom(), has_odom(false), status(), status_msg(), position_msg(), odom_sub(),
                status_pub(), position_pub()
            {
            }
        };

    private: // Parameters
        core::ID agent_id_;
        float update_rate_;
        // Takeoff parameters
        float takeoff_altitude_;
        float takeoff_timeout_;
        // Landing parameters
        float landing_altitude_;
        float landing_timeout_;
        // Hovering parameters
        float hover_altitude_;
        float hover_timeout_;
        // Command timeout
        float cmd_timeout_;

    private: // Data
        // Agent
        Agent agent_;
        // Time step
        float status_duration_;
        core::Time last_update_time_;

    public: // Public methods
        // Status getter
        core::AgentStatus getStatus() const { return agent_.status; }
        // Status transition requests
        bool requestDisarm();
        bool requestArm();
        bool requestTakeoff();
        bool requestHover();
        bool requestTracking();
        bool requestLand();

    private: // Callbacks
        void odomCallback(const core::OdometryMsg::SharedPtr msg);

    private: // State management
        void update();

    private: // State methods
        void setStatus(const core::AgentStatus& new_status);
        bool isValid(const core::AgentStatus& from, const core::AgentStatus& to) const;
        void handleIdle();
        void handleDisarmed();
        void handleArmed();
        void handleTakingOff();
        void handleTakenOff();
        void handleHovering();
        void handleHovered();
        void handleTracking();
        void handleLanding();
        void handleLanded();
        void handleError();

    private: // ROS components
        // Timer
        core::TimerPtr update_timer_;
    };

} // namespace flychams::control