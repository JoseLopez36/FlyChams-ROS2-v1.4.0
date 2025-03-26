#pragma once

// Standard includes
#include <mutex>

// Assignment solver include
#include "flychams_coordination/assignment/assignment_solver.hpp"

// Base module include
#include "flychams_core/base/base_module.hpp"

namespace flychams::coordination
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Assignment manager for UAV agents
     *
     * @details
     * This class is responsible for controlling the assignment
     * of UAV agents to clusters. It manages multiple agents
     * and their respective assignment parameters.
     *
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-01-29
     * ════════════════════════════════════════════════════════════════
     */
    class AgentAssignment : public core::BaseModule
    {
    public: // Constructor/Destructor
        AgentAssignment(core::NodePtr node, core::ConfigTools::SharedPtr config_tools, core::FrameworkTools::SharedPtr framework_tools, core::TopicTools::SharedPtr topic_tools, core::TransformTools::SharedPtr transform_tools)
            : BaseModule(node, config_tools, framework_tools, topic_tools, transform_tools)
        {
            init();
        }

    protected: // Overrides
        void onInit() override;
        void onShutdown() override;

    public: // Types
        using SharedPtr = std::shared_ptr<AgentAssignment>;

    private: // Data
        // Agent data
        std::unordered_set<core::ID> agent_ids_;
        AssignmentSolver::Agents agents_;
        // Cluster data
        std::unordered_set<core::ID> cluster_ids_;
        AssignmentSolver::Clusters clusters_;
        // Assignments
        AssignmentSolver::Assignments assignments_;
        std::unordered_map<core::ID, std::set<core::ID>> agent_clusters_;
        bool has_assignment_;

        // Thread-safety
        std::mutex mutex_;

        // Assignment solver
        AssignmentSolver solver_;

    public: // Public methods
        void addAgent(const core::ID& agent_id);
        void removeAgent(const core::ID& agent_id);
        void addCluster(const core::ID& cluster_id);
        void removeCluster(const core::ID& cluster_id);

    private: // Methods
        // Callbacks
        void agentOdomCallback(const core::ID& agent_id, const core::OdometryMsg::SharedPtr msg);
        void clusterInfoCallback(const core::ID& cluster_id, const core::ClusterInfoMsg::SharedPtr msg);
        // Update
        void updateAssignment();
        void publishAssignment();

    private:
        // Subscribers
        std::unordered_map<core::ID, core::SubscriberPtr<core::OdometryMsg>> agent_odom_subs_;
        std::unordered_map<core::ID, core::SubscriberPtr<core::ClusterInfoMsg>> cluster_info_subs_;
        // Publishers
        std::unordered_map<core::ID, core::PublisherPtr<core::TrackingInfoMsg>> agent_info_pubs_;
        // Timers
        core::TimerPtr assignment_timer_;
        core::TimerPtr publish_timer_;
    };

} // namespace flychams::coordination