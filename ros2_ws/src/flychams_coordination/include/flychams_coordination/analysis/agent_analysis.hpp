#pragma once

// Base module include
#include "flychams_core/base/base_module.hpp"

namespace flychams::coordination
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Agent analysis manager
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-03-28
     * ════════════════════════════════════════════════════════════════
     */
    class AgentAnalysis : public core::BaseModule
    {
    public: // Constructor/Destructor
        AgentAnalysis(core::NodePtr node, core::ConfigTools::SharedPtr config_tools, core::FrameworkTools::SharedPtr framework_tools, core::TopicTools::SharedPtr topic_tools, core::TransformTools::SharedPtr transform_tools, core::CallbackGroupPtr module_cb_group)
            : BaseModule(node, config_tools, framework_tools, topic_tools, transform_tools, module_cb_group)
        {
            init();
        }

    protected: // Overrides
        void onInit() override;
        void onShutdown() override;

    public: // Types
        using SharedPtr = std::shared_ptr<AgentAnalysis>;
        struct Agent
        {
            // Status data
            core::AgentStatus status;
            bool has_status;
            // Assignment data
            std::vector<core::ID> assignment;
            bool has_assignment;
            // Subscriber
            core::SubscriberPtr<core::AgentStatusMsg> status_sub;
            core::SubscriberPtr<core::AgentAssignmentMsg> assignment_sub;
            // Publisher
            core::PublisherPtr<core::AgentClustersMsg> clusters_pub;
            // Constructor
            Agent()
                : status(), has_status(false), assignment(), has_assignment(false), status_sub(), assignment_sub(), clusters_pub()
            {
            }
        };
        struct Cluster
        {
            // Geometric data
            core::PointMsg center;
            float radius;
            bool has_geometry;
            // Subscribers
            core::SubscriberPtr<core::ClusterGeometryMsg> geometry_sub;
            // Constructor
            Cluster()
                : center(), radius(), has_geometry(false), geometry_sub()
            {
            }
        };

    private: // Parameters
        float update_rate_;

    private: // Data
        // Agents
        std::unordered_map<core::ID, Agent> agents_;
        // Clusters
        std::unordered_map<core::ID, Cluster> clusters_;

    public: // Public methods
        void addAgent(const core::ID& agent_id);
        void addCluster(const core::ID& cluster_id);
        void removeAgent(const core::ID& agent_id);
        void removeCluster(const core::ID& cluster_id);

    private: // Callbacks
        void agentStatusCallback(const core::ID& agent_id, const core::AgentStatusMsg::SharedPtr msg);
        void agentAssignmentCallback(const core::ID& agent_id, const core::AgentAssignmentMsg::SharedPtr msg);
        void clusterGeometryCallback(const core::ID& cluster_id, const core::ClusterGeometryMsg::SharedPtr msg);

    private: // Analysis management
        void update();

    private: // Analysis methods
        std::pair<core::PointMsg, float> computeCentralCluster(const std::vector<core::PointMsg>& centers, const std::vector<float>& radii);

    private: // ROS components
        // Timer
        core::TimerPtr update_timer_;
    };

} // namespace flychams::coordination