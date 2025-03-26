#pragma once

// Standard includes
#include <mutex>

// Metrics factory include
#include "flychams_dashboard/visualization/metrics_factory.hpp"

// Markers factory include
#include "flychams_dashboard/visualization/markers_factory.hpp"

// Base module include
#include "flychams_core/base/base_module.hpp"

namespace flychams::dashboard
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Factory for creating and updating visualization components
     *
     * @details
     * This class is responsible for creating and updating various
     * visualization components, including RViz markers and standard
     * metrics.
     *
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-02-27
     * ════════════════════════════════════════════════════════════════
     */
    class VisualizationFactory : public core::BaseModule
    {
    public: // Constructor/Destructor
        VisualizationFactory(core::NodePtr node, core::ConfigTools::SharedPtr config_tools, core::FrameworkTools::SharedPtr framework_tools, core::TopicTools::SharedPtr topic_tools, core::TransformTools::SharedPtr transform_tools)
            : BaseModule(node, config_tools, framework_tools, topic_tools, transform_tools)
        {
            init();
        }

    protected: // Overrides
        void onInit() override;
        void onShutdown() override;

    public: // Types
        using SharedPtr = std::shared_ptr<VisualizationFactory>;

    private: // Parameters
        float metrics_update_rate_;
        float markers_update_rate_;

    private: // Data
        // Element metrics
        std::unordered_map<core::ID, core::AgentMetrics> curr_agent_metrics_;
        std::unordered_map<core::ID, core::AgentMetrics> prev_agent_metrics_;
        std::unordered_map<core::ID, core::TargetMetrics> curr_target_metrics_;
        std::unordered_map<core::ID, core::TargetMetrics> prev_target_metrics_;
        std::unordered_map<core::ID, core::ClusterMetrics> curr_cluster_metrics_;
        std::unordered_map<core::ID, core::ClusterMetrics> prev_cluster_metrics_;
        core::GlobalMetrics curr_global_metrics_;
        core::GlobalMetrics prev_global_metrics_;
        // Element markers
        std::unordered_map<core::ID, core::MarkerArrayMsg> agent_markers_;
        std::unordered_map<core::ID, core::MarkerArrayMsg> target_markers_;
        std::unordered_map<core::ID, core::MarkerArrayMsg> cluster_markers_;
        // Element IDs
        std::unordered_set<core::ID> agent_ids_;
        std::unordered_set<core::ID> target_ids_;
        std::unordered_set<core::ID> cluster_ids_;
        // Time data
        core::Time prev_time_;

    public: // Public methods
        void addAgent(const core::ID& agent_id);
        void removeAgent(const core::ID& agent_id);
        void addTarget(const core::ID& target_id);
        void removeTarget(const core::ID& target_id);
        void addCluster(const core::ID& cluster_id);
        void removeCluster(const core::ID& cluster_id);

    private: // Methods
        // Callbacks
        void agentOdomCallback(const core::ID& agent_id, const core::OdometryMsg::SharedPtr msg);
        void agentGoalCallback(const core::ID& agent_id, const core::PositionGoalMsg::SharedPtr msg);
        void targetInfoCallback(const core::ID& target_id, const core::TargetInfoMsg::SharedPtr msg);
        void clusterInfoCallback(const core::ID& cluster_id, const core::ClusterInfoMsg::SharedPtr msg);
        // Update
        void updateMetrics();
        void updateRvizMarkers();

    private:
        // Subscriber callback group
        core::CallbackGroupPtr callback_group_;
        // Subscribers
        std::unordered_map<std::string, core::SubscriberPtr<core::OdometryMsg>> agent_odom_subs_;
        std::unordered_map<std::string, core::SubscriberPtr<core::PositionGoalMsg>> agent_goal_subs_;
        std::unordered_map<std::string, core::SubscriberPtr<core::TargetInfoMsg>> target_info_subs_;
        std::unordered_map<std::string, core::SubscriberPtr<core::ClusterInfoMsg>> cluster_info_subs_;
        // Publishers
        std::unordered_map<std::string, core::PublisherPtr<core::AgentMetricsMsg>> agent_metrics_pubs_;
        std::unordered_map<std::string, core::PublisherPtr<core::TargetMetricsMsg>> target_metrics_pubs_;
        std::unordered_map<std::string, core::PublisherPtr<core::ClusterMetricsMsg>> cluster_metrics_pubs_;
        core::PublisherPtr<core::GlobalMetricsMsg> global_metrics_pubs_;
        std::unordered_map<std::string, core::PublisherPtr<core::MarkerArrayMsg>> agent_markers_pubs_;
        std::unordered_map<std::string, core::PublisherPtr<core::MarkerArrayMsg>> target_markers_pubs_;
        std::unordered_map<std::string, core::PublisherPtr<core::MarkerArrayMsg>> cluster_markers_pubs_;
        // Timers
        core::TimerPtr metrics_timer_;
        core::TimerPtr rviz_markers_timer_;
    };

} // namespace flychams::dashboard