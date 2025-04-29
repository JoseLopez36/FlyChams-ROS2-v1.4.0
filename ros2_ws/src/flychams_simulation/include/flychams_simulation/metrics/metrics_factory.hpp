#pragma once

// Base module include
#include "flychams_core/base/base_module.hpp"

namespace flychams::simulation
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Factory for creating and updating metrics
     *
     * @details
     * This class is responsible for creating and updating various
     * metrics.
     *
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-02-27
     * ════════════════════════════════════════════════════════════════
     */
    class MetricsFactory : public core::BaseModule
    {
    public: // Constructor/Destructor
        MetricsFactory(core::NodePtr node, core::ConfigTools::SharedPtr config_tools, core::FrameworkTools::SharedPtr framework_tools, core::TopicTools::SharedPtr topic_tools, core::TransformTools::SharedPtr transform_tools, core::CallbackGroupPtr module_cb_group)
            : BaseModule(node, config_tools, framework_tools, topic_tools, transform_tools, module_cb_group)
        {
            init();
        }

    protected: // Overrides
        void onInit() override;
        void onShutdown() override;

    public: // Types
        using SharedPtr = std::shared_ptr<MetricsFactory>;
        struct Global
        {
            // Metrics message
            core::GlobalMetricsMsg metrics;
            // Publisher
            core::PublisherPtr<core::GlobalMetricsMsg> metrics_pub;
            // Constructor
            Global() : metrics(), metrics_pub() {}
        };
        struct Agent
        {
            // Metrics message
            core::AgentMetricsMsg metrics;
            // Previous metrics
            core::AgentMetricsMsg prev_metrics;
            // Subscribers
            core::SubscriberPtr<core::PointStampedMsg> position_sub;
            core::SubscriberPtr<core::PointStampedMsg> position_setpoint_sub;
            core::SubscriberPtr<core::AgentTrackingSetpointsMsg> tracking_setpoints_sub;
            // Publisher
            core::PublisherPtr<core::AgentMetricsMsg> metrics_pub;
            // Constructor
            Agent()
                : metrics(), prev_metrics(), position_sub(), position_setpoint_sub(), 
                tracking_setpoints_sub(), metrics_pub()
            {
            }
        };
        struct Target
        {
            // Metrics message
            core::TargetMetricsMsg metrics;
            // Previous metrics
            core::TargetMetricsMsg prev_metrics;
            // Subscribers
            core::SubscriberPtr<core::PointStampedMsg> position_sub;
            // Publisher
            core::PublisherPtr<core::TargetMetricsMsg> metrics_pub;
            // Constructor
            Target()
                : metrics(), prev_metrics(), position_sub(), metrics_pub()
            {
            }
        };
        struct Cluster
        {
            // Metrics message
            core::ClusterMetricsMsg metrics;
            // Previous metrics
            core::ClusterMetricsMsg prev_metrics;
            // Subscriber
            core::SubscriberPtr<core::ClusterGeometryMsg> geometry_sub;
            // Publisher
            core::PublisherPtr<core::ClusterMetricsMsg> metrics_pub;
            // Constructor
            Cluster()
                : metrics(), prev_metrics(), geometry_sub(), metrics_pub()
            {
            }
        };


    private: // Parameters
        float update_rate_;

    private: // Data
        // Global
        Global global_;
        // Agents
        std::unordered_map<core::ID, Agent> agents_;
        // Targets
        std::unordered_map<core::ID, Target> targets_;
        // Clusters
        std::unordered_map<core::ID, Cluster> clusters_;
        // Time step
        core::Time last_update_time_;

    public: // Public methods
        void addAgent(const core::ID& agent_id);
        void addTarget(const core::ID& target_id);
        void addCluster(const core::ID& cluster_id);
        void removeAgent(const core::ID& agent_id);
        void removeTarget(const core::ID& target_id);
        void removeCluster(const core::ID& cluster_id);

    private: // Callbacks
        void agentPositionCallback(const core::ID& agent_id, const core::PointStampedMsg::SharedPtr msg);
        void agentPositionSetpointCallback(const core::ID& agent_id, const core::PointStampedMsg::SharedPtr msg);
        void agentTrackingSetpointsCallback(const core::ID& agent_id, const core::AgentTrackingSetpointsMsg::SharedPtr msg);
        void targetPositionCallback(const core::ID& target_id, const core::PointStampedMsg::SharedPtr msg);
        void clusterGeometryCallback(const core::ID& cluster_id, const core::ClusterGeometryMsg::SharedPtr msg);

    private: // Metrics management
        void update();

    private: // Metrics methods
        void createGlobalMetrics(core::GlobalMetricsMsg& metrics);
        void createAgentMetrics(core::AgentMetricsMsg& metrics);
        void createTargetMetrics(core::TargetMetricsMsg& metrics);
        void createClusterMetrics(core::ClusterMetricsMsg& metrics);
        void updateGlobalMetrics(core::GlobalMetricsMsg& metrics, float dt);
        void updateAgentMetrics(core::AgentMetricsMsg& prev_metrics, core::AgentMetricsMsg& curr_metrics, float dt);
        void updateTargetMetrics(core::TargetMetricsMsg& prev_metrics, core::TargetMetricsMsg& curr_metrics, float dt);
        void updateClusterMetrics(core::ClusterMetricsMsg& prev_metrics, core::ClusterMetricsMsg& curr_metrics, float dt);

    private: // ROS components
        // Timer
        core::TimerPtr update_timer_;
    };

} // namespace flychams::simulation