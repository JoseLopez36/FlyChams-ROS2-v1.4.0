#pragma once

// Standard includes
#include <mutex>

// Position solver include
#include "flychams_coordination/positioning/position_solver.hpp"

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
    class AgentPositioning : public core::BaseModule
    {
    public: // Constructor/Destructor
        AgentPositioning(const core::ID& agent_id, core::NodePtr node, core::ConfigTools::SharedPtr config_tools, core::ExternalTools::SharedPtr ext_tools, core::TopicTools::SharedPtr topic_tools, core::TfTools::SharedPtr tf_tools)
            : BaseModule(node, config_tools, ext_tools, topic_tools, tf_tools), agent_id_(agent_id)
        {
            init();
        }

    protected: // Overrides
        void onInit() override;
        void onShutdown() override;

    public: // Types
        using SharedPtr = std::shared_ptr<AgentPositioning>;

    private: // Parameters
        core::ID agent_id_;

    private: // Data
        // Odom
        core::Vector3r curr_pos_; 	            // Current position (x, y, z)
        bool has_odom_;
        // Clusters
        std::pair<core::Matrix3Xr, core::RowVectorXr> clusters_;
        bool has_clusters_;
        // Odom and goal mutex
        std::mutex mutex_;
        // Position solver
        PositionSolver::SharedPtr solver_;

    private: // Methods
        // Callbacks
        void odomCallback(const core::OdometryMsg::SharedPtr msg);
        void infoCallback(const core::AgentInfoMsg::SharedPtr msg);
        // Update
        void updatePosition();

    private:
        // Subscribers
        core::SubscriberPtr<core::OdometryMsg> odom_sub_;
        core::SubscriberPtr<core::AgentInfoMsg> info_sub_;
        // Publishers
        core::PublisherPtr<core::AgentGoalMsg> goal_pub_;
        // Timers
        core::TimerPtr positioning_timer_;
    };

} // namespace flychams::coordination