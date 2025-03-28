#pragma once

// Base module include
#include "flychams_core/base/base_module.hpp"

namespace flychams::control
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief State manager for UAV drones
     *
     * @details
     * This class implements a state manager for UAV drones.
     * It handles the state transitions and the communication with the drone.
     *
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
        // Current status
        core::AgentStatus curr_status_;
        float status_duration_;
        // Current position
        core::PointMsg curr_position_;
        bool has_position_;
        // Messages
        core::AgentStatusMsg status_msg_;
        core::PointStampedMsg position_msg_;
        // Time step
        core::Time last_update_time_;

    public: // Public methods
        // Status getter
        const core::AgentStatus getStatus() const { return curr_status_; }
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
        void setStatus(const core::AgentStatus& new_status);
        bool isValid(const core::AgentStatus& from, const core::AgentStatus& to) const;

    private: // State handlers
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

    private:
        // Callback group
        core::CallbackGroupPtr callback_group_;
        // Subscriber
        core::SubscriberPtr<core::OdometryMsg> odom_sub_;
        // Publishers
        core::PublisherPtr<core::AgentStatusMsg> status_pub_;
        core::PublisherPtr<core::PointStampedMsg> position_pub_;
        // Timer
        core::TimerPtr update_timer_;
    };

} // namespace flychams::control