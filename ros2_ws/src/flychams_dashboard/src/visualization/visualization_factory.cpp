#include "flychams_dashboard/visualization/visualization_factory.hpp"

using namespace flychams::core;

namespace flychams::dashboard
{
    // ════════════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR: Constructor and destructor
    // ════════════════════════════════════════════════════════════════════════════

    void VisualizationFactory::onInit()
    {
        // Get parameters from parameter server
        // Get update rates
        metrics_update_rate_ = RosUtils::getParameterOr<float>(node_, "visualization.metrics_update_rate", 10.0f);
        markers_update_rate_ = RosUtils::getParameterOr<float>(node_, "visualization.markers_update_rate", 10.0f);
        // Get recording flags from config
        bool record_metrics = config_tools_->getSimulation()->record_metrics;
        bool draw_rviz_markers = config_tools_->getSimulation()->draw_rviz_markers;

        // Initialize data
        curr_agent_metrics_.clear();
        prev_agent_metrics_.clear();
        curr_target_metrics_.clear();
        prev_target_metrics_.clear();
        curr_cluster_metrics_.clear();
        prev_cluster_metrics_.clear();
        curr_global_metrics_ = MetricsFactory::createDefaultGlobal();
        prev_global_metrics_ = MetricsFactory::createDefaultGlobal();
        agent_markers_.clear();
        target_markers_.clear();
        cluster_markers_.clear();

        // Initialize global metrics publisher
        global_metrics_pubs_ = topic_tools_->createGlobalMetricsPublisher();

        // Initialize callback group
        callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // Set update timers
        prev_time_ = RosUtils::now(node_);
        if (record_metrics)
        {
            metrics_timer_ = RosUtils::createWallTimerByRate(node_, metrics_update_rate_,
                std::bind(&VisualizationFactory::updateMetrics, this), callback_group_);
        }
        if (draw_rviz_markers)
        {
            rviz_markers_timer_ = RosUtils::createWallTimerByRate(node_, markers_update_rate_,
                std::bind(&VisualizationFactory::updateRvizMarkers, this), callback_group_);
        }
    }

    void VisualizationFactory::onShutdown()
    {
        // Destroy data
        curr_agent_metrics_.clear();
        prev_agent_metrics_.clear();
        curr_target_metrics_.clear();
        prev_target_metrics_.clear();
        curr_cluster_metrics_.clear();
        prev_cluster_metrics_.clear();
        agent_markers_.clear();
        target_markers_.clear();
        cluster_markers_.clear();
        // Destroy subscribers
        agent_odom_subs_.clear();
        agent_goal_subs_.clear();
        target_info_subs_.clear();
        cluster_info_subs_.clear();
        // Destroy publishers
        agent_metrics_pubs_.clear();
        agent_markers_pubs_.clear();
        target_metrics_pubs_.clear();
        target_markers_pubs_.clear();
        cluster_metrics_pubs_.clear();
        cluster_markers_pubs_.clear();
        global_metrics_pubs_.reset();
        // Destroy update timers
        metrics_timer_.reset();
        rviz_markers_timer_.reset();
    }

    // ════════════════════════════════════════════════════════════════════════════
    // CALLBACKS: Callback functions
    // ════════════════════════════════════════════════════════════════════════════

    void VisualizationFactory::agentOdomCallback(const core::ID& agent_id, const core::OdometryMsg::SharedPtr msg)
    {
        // Update agent metrics under lock
        curr_agent_metrics_[agent_id].curr_x = msg->pose.position.x;
        curr_agent_metrics_[agent_id].curr_y = msg->pose.position.y;
        curr_agent_metrics_[agent_id].curr_z = msg->pose.position.z;
        curr_agent_metrics_[agent_id].vel_x = msg->twist.linear.x;
        curr_agent_metrics_[agent_id].vel_y = msg->twist.linear.y;
        curr_agent_metrics_[agent_id].vel_z = msg->twist.linear.z;
    }

    void VisualizationFactory::agentGoalCallback(const core::ID& agent_id, const core::PositionGoalMsg::SharedPtr msg)
    {
        // Get target position
        curr_agent_metrics_[agent_id].goal_x = msg->position.x;
        curr_agent_metrics_[agent_id].goal_y = msg->position.y;
        curr_agent_metrics_[agent_id].goal_z = msg->position.z;
    }


    void VisualizationFactory::targetInfoCallback(const core::ID& target_id, const core::TargetInfoMsg::SharedPtr msg)
    {
        // Update target metrics under lock
        curr_target_metrics_[target_id].curr_x = msg->position.x;
        curr_target_metrics_[target_id].curr_y = msg->position.y;
        curr_target_metrics_[target_id].curr_z = msg->position.z;
    }

    void VisualizationFactory::clusterInfoCallback(const core::ID& cluster_id, const core::ClusterInfoMsg::SharedPtr msg)
    {
        // Update cluster metrics under lock
        curr_cluster_metrics_[cluster_id].curr_center_x = msg->center.x;
        curr_cluster_metrics_[cluster_id].curr_center_y = msg->center.y;
        curr_cluster_metrics_[cluster_id].curr_center_z = msg->center.z;
        curr_cluster_metrics_[cluster_id].curr_radius = msg->radius;
    }

    // ════════════════════════════════════════════════════════════════════════════
    // UPDATE: Update metrics and markers
    // ════════════════════════════════════════════════════════════════════════════

    void VisualizationFactory::updateMetrics()
    {
        // Get current time
        auto curr_time = RosUtils::now(node_);
        float dt = (curr_time - prev_time_).seconds();
        prev_time_ = curr_time;

        // Update agent metrics
        for (const auto& agent_id : agent_ids_)
        {
            MetricsFactory::updateAgentMetrics(prev_agent_metrics_[agent_id], curr_agent_metrics_[agent_id], dt);
            prev_agent_metrics_[agent_id] = curr_agent_metrics_[agent_id];
            // Publish agent metrics
            AgentMetricsMsg agent_metrics_msg;
            agent_metrics_msg.header.stamp = curr_time;
            agent_metrics_msg.header.frame_id = transform_tools_->getGlobalFrame();
            RosUtils::toMsg(curr_agent_metrics_[agent_id], agent_metrics_msg);
            agent_metrics_pubs_[agent_id]->publish(agent_metrics_msg);
        }

        // Update target metrics
        for (const auto& target_id : target_ids_)
        {
            MetricsFactory::updateTargetMetrics(prev_target_metrics_[target_id], curr_target_metrics_[target_id], dt);
            prev_target_metrics_[target_id] = curr_target_metrics_[target_id];
            // Publish target metrics
            TargetMetricsMsg target_metrics_msg;
            target_metrics_msg.header.stamp = curr_time;
            target_metrics_msg.header.frame_id = transform_tools_->getGlobalFrame();
            RosUtils::toMsg(curr_target_metrics_[target_id], target_metrics_msg);
            target_metrics_pubs_[target_id]->publish(target_metrics_msg);
        }

        // Update cluster metrics
        for (const auto& cluster_id : cluster_ids_)
        {
            MetricsFactory::updateClusterMetrics(prev_cluster_metrics_[cluster_id], curr_cluster_metrics_[cluster_id], dt);
            prev_cluster_metrics_[cluster_id] = curr_cluster_metrics_[cluster_id];
            // Publish cluster metrics
            ClusterMetricsMsg cluster_metrics_msg;
            cluster_metrics_msg.header.stamp = curr_time;
            cluster_metrics_msg.header.frame_id = transform_tools_->getGlobalFrame();
            RosUtils::toMsg(curr_cluster_metrics_[cluster_id], cluster_metrics_msg);
            cluster_metrics_pubs_[cluster_id]->publish(cluster_metrics_msg);
        }

        // Update global metrics
        MetricsFactory::updateGlobalMetrics(prev_global_metrics_, curr_global_metrics_, dt);
        prev_global_metrics_ = curr_global_metrics_;
        // Publish global metrics
        GlobalMetricsMsg global_metrics_msg;
        global_metrics_msg.header.stamp = curr_time;
        global_metrics_msg.header.frame_id = transform_tools_->getGlobalFrame();
        RosUtils::toMsg(curr_global_metrics_, global_metrics_msg);
        global_metrics_pubs_->publish(global_metrics_msg);
    }

    void VisualizationFactory::updateRvizMarkers()
    {
        // Get current time
        auto curr_time = RosUtils::now(node_);

        // Update agent markers
        for (const auto& agent_id : agent_ids_)
        {
            // Get agent metrics
            const auto& metrics = curr_agent_metrics_[agent_id];
            // Get agent markers
            auto& markers = agent_markers_[agent_id];
            // Update agent markers
            MarkersFactory::updateAgentPoint(metrics.curr_x, metrics.curr_y, metrics.curr_z, curr_time, markers.markers[0]);
            MarkersFactory::updateAgentGoalPoint(metrics.goal_x, metrics.goal_y, metrics.goal_z, curr_time, markers.markers[1]);
            // Publish updated agent markers
            agent_markers_pubs_[agent_id]->publish(markers);
        }

        // Update target markers
        for (const auto& target_id : target_ids_)
        {
            // Get target metrics
            const auto& metrics = curr_target_metrics_[target_id];
            // Get target markers
            auto& markers = target_markers_[target_id];
            // Update target markers
            MarkersFactory::updateTargetPoint(metrics.curr_x, metrics.curr_y, metrics.curr_z, curr_time, markers.markers[0]);
            // Publish updated target markers
            target_markers_pubs_[target_id]->publish(markers);
        }

        // Update cluster markers
        for (const auto& cluster_id : cluster_ids_)
        {
            // Get cluster metrics
            const auto& metrics = curr_cluster_metrics_[cluster_id];
            // Get cluster markers
            auto& markers = cluster_markers_[cluster_id];
            // Update cluster markers
            MarkersFactory::updateClusterCenter(metrics.curr_center_x, metrics.curr_center_y, metrics.curr_center_z, curr_time, markers.markers[0]);
            MarkersFactory::updateClusterBoundary(metrics.curr_center_x, metrics.curr_center_y, metrics.curr_center_z, metrics.curr_radius, curr_time, markers.markers[1]);
            // Publish cluster markers
            cluster_markers_pubs_[cluster_id]->publish(markers);
        }
    }

    // ════════════════════════════════════════════════════════════════════════════
    // HELPER METHODS: Helper methods for adding and removing elements
    // ════════════════════════════════════════════════════════════════════════════

    void VisualizationFactory::addAgent(const core::ID& agent_id)
    {
        // Add agent to set
        agent_ids_.insert(agent_id);
        // Add metrics
        curr_agent_metrics_.insert({ agent_id, MetricsFactory::createDefaultAgent() });
        prev_agent_metrics_.insert({ agent_id, MetricsFactory::createDefaultAgent() });
        // Add markers
        agent_markers_.insert({ agent_id, MarkersFactory::createAgentMarkers(transform_tools_->getGlobalFrame(), markers_update_rate_ * 1.1f) });
        // Add subscribers
        auto options = rclcpp::SubscriptionOptions();
        options.callback_group = callback_group_;
        agent_odom_subs_.insert({ agent_id, topic_tools_->createAgentOdomSubscriber(agent_id,
            [this, agent_id](const OdometryMsg::SharedPtr msg)
            {
                this->agentOdomCallback(agent_id, msg);
            }, options) });
        agent_goal_subs_.insert({ agent_id, topic_tools_->createAgentPositionGoalSubscriber(agent_id,
            [this, agent_id](const PositionGoalMsg::SharedPtr msg)
            {
                this->agentGoalCallback(agent_id, msg);
            }, options) });
        // Add publishers
        agent_metrics_pubs_.insert({ agent_id, topic_tools_->createAgentMetricsPublisher(agent_id) });
        agent_markers_pubs_.insert({ agent_id, topic_tools_->createAgentMarkersPublisher(agent_id) });

    }

    void VisualizationFactory::removeAgent(const core::ID& agent_id)
    {
        // Remove agent from set
        agent_ids_.erase(agent_id);
        // Remove metrics
        curr_agent_metrics_.erase(agent_id);
        prev_agent_metrics_.erase(agent_id);
        // Remove markers
        agent_markers_.erase(agent_id);
        // Remove subscriber
        agent_odom_subs_.erase(agent_id);
        agent_goal_subs_.erase(agent_id);
        // Remove publishers
        agent_metrics_pubs_.erase(agent_id);
        agent_markers_pubs_.erase(agent_id);
    }

    void VisualizationFactory::addTarget(const core::ID& target_id)
    {
        // Add target to set
        target_ids_.insert(target_id);
        // Add metrics
        curr_target_metrics_.insert({ target_id, MetricsFactory::createDefaultTarget() });
        prev_target_metrics_.insert({ target_id, MetricsFactory::createDefaultTarget() });
        // Add markers
        target_markers_.insert({ target_id, MarkersFactory::createTargetMarkers(transform_tools_->getGlobalFrame(), markers_update_rate_ * 1.1f) });
        // Add subscriber
        auto options = rclcpp::SubscriptionOptions();
        options.callback_group = callback_group_;
        target_info_subs_.insert({ target_id, topic_tools_->createTargetInfoSubscriber(target_id,
            [this, target_id](const TargetInfoMsg::SharedPtr msg)
            {
                this->targetInfoCallback(target_id, msg);
            }, options) });
        // Add publishers   
        target_metrics_pubs_.insert({ target_id, topic_tools_->createTargetMetricsPublisher(target_id) });
        target_markers_pubs_.insert({ target_id, topic_tools_->createTargetMarkersPublisher(target_id) });
    }

    void VisualizationFactory::removeTarget(const core::ID& target_id)
    {
        // Remove target from set
        target_ids_.erase(target_id);
        // Remove metrics
        curr_target_metrics_.erase(target_id);
        prev_target_metrics_.erase(target_id);
        // Remove markers
        target_markers_.erase(target_id);
        // Remove subscriber
        target_info_subs_.erase(target_id);
        // Remove publishers
        target_metrics_pubs_.erase(target_id);
        target_markers_pubs_.erase(target_id);
    }

    void VisualizationFactory::addCluster(const core::ID& cluster_id)
    {
        // Add cluster to set
        cluster_ids_.insert(cluster_id);
        // Add metrics
        curr_cluster_metrics_.insert({ cluster_id, MetricsFactory::createDefaultCluster() });
        prev_cluster_metrics_.insert({ cluster_id, MetricsFactory::createDefaultCluster() });
        // Add markers
        cluster_markers_.insert({ cluster_id, MarkersFactory::createClusterMarkers(transform_tools_->getGlobalFrame(), markers_update_rate_ * 1.1f) });
        // Add subscriber
        auto options = rclcpp::SubscriptionOptions();
        options.callback_group = callback_group_;
        cluster_info_subs_.insert({ cluster_id, topic_tools_->createClusterInfoSubscriber(cluster_id,
            [this, cluster_id](const ClusterInfoMsg::SharedPtr msg)
            {
                this->clusterInfoCallback(cluster_id, msg);
            }, options) });
        // Add publishers
        cluster_metrics_pubs_.insert({ cluster_id, topic_tools_->createClusterMetricsPublisher(cluster_id) });
        cluster_markers_pubs_.insert({ cluster_id, topic_tools_->createClusterMarkersPublisher(cluster_id) });
    }

    void VisualizationFactory::removeCluster(const core::ID& cluster_id)
    {
        // Remove cluster from set
        cluster_ids_.erase(cluster_id);
        // Remove metrics
        curr_cluster_metrics_.erase(cluster_id);
        prev_cluster_metrics_.erase(cluster_id);
        // Remove markers
        cluster_markers_.erase(cluster_id);
        // Remove subscriber
        cluster_info_subs_.erase(cluster_id);
        // Remove publishers
        cluster_metrics_pubs_.erase(cluster_id);
        cluster_markers_pubs_.erase(cluster_id);
    }

} // namespace flychams::dashboard