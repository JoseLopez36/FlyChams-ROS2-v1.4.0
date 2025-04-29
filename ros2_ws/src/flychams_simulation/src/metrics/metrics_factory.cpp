#include "flychams_simulation/metrics/metrics_factory.hpp"

using namespace flychams::core;

namespace flychams::simulation
{
    // ════════════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR: Constructor and destructor
    // ════════════════════════════════════════════════════════════════════════════

    void MetricsFactory::onInit()
    {
        // Get parameters from parameter server
        // Get update rate
        update_rate_ = RosUtils::getParameterOr<float>(node_, "metrics_factory.update_rate", 10.0f);

        // Initialize data
        agents_.clear();
        targets_.clear();
        clusters_.clear();

        // Initialize global metrics
        createGlobalMetrics(global_.metrics);

        // Create global metrics publisher
        global_.metrics_pub = topic_tools_->createGlobalMetricsPublisher();

        // Set update timer
        last_update_time_ = RosUtils::now(node_);
        update_timer_ = RosUtils::createTimer(node_, update_rate_,
            std::bind(&MetricsFactory::update, this), module_cb_group_);
    }

    void MetricsFactory::onShutdown()
    {
        // Destroy agents, targets and clusters
        agents_.clear();
        targets_.clear();
        clusters_.clear();
        // Destroy global metrics publisher
        global_.metrics_pub.reset();
        // Destroy update timer
        update_timer_.reset();
    }

    // ════════════════════════════════════════════════════════════════════════════
    // PUBLIC METHODS: Public methods for adding/removing agents, targets and clusters
    // ════════════════════════════════════════════════════════════════════════════

    void MetricsFactory::addAgent(const ID& agent_id)
    {
        // Create and add agent
        agents_.insert({ agent_id, Agent() });

        // Initialize agent metrics
        createAgentMetrics(agents_[agent_id].metrics);
        createAgentMetrics(agents_[agent_id].prev_metrics);

        // Create agent position subscriber
        agents_[agent_id].position_sub = topic_tools_->createAgentPositionSubscriber(agent_id,
            [this, agent_id](const PointStampedMsg::SharedPtr msg)
            {
                this->agentPositionCallback(agent_id, msg);
            }, sub_options_with_module_cb_group_);

        // Create agent position setpoint subscriber
        agents_[agent_id].position_setpoint_sub = topic_tools_->createAgentPositionSetpointSubscriber(agent_id,
            [this, agent_id](const PointStampedMsg::SharedPtr msg)
            {
                this->agentPositionSetpointCallback(agent_id, msg);
            }, sub_options_with_module_cb_group_);

        // Create agent tracking setpoints subscriber
        agents_[agent_id].tracking_setpoints_sub = topic_tools_->createAgentTrackingSetpointsSubscriber(agent_id,
            [this, agent_id](const AgentTrackingSetpointsMsg::SharedPtr msg)
            {
                this->agentTrackingSetpointsCallback(agent_id, msg);
            }, sub_options_with_module_cb_group_);

        // Create agent metrics publisher
        agents_[agent_id].metrics_pub = topic_tools_->createAgentMetricsPublisher(agent_id);
    }

    void MetricsFactory::removeAgent(const ID& agent_id)
    {
        // Remove agent from map
        agents_.erase(agent_id);
    }

    void MetricsFactory::addTarget(const ID& target_id)
    {
        // Create and add target
        targets_.insert({ target_id, Target() });

        // Initialize target metrics
        createTargetMetrics(targets_[target_id].metrics);
        createTargetMetrics(targets_[target_id].prev_metrics);

        // Create target position subscriber
        targets_[target_id].position_sub = topic_tools_->createTargetTruePositionSubscriber(target_id,
            [this, target_id](const PointStampedMsg::SharedPtr msg)
            {
                this->targetPositionCallback(target_id, msg);
            }, sub_options_with_module_cb_group_);

        // Create target metrics publisher
        targets_[target_id].metrics_pub = topic_tools_->createTargetMetricsPublisher(target_id);
    }

    void MetricsFactory::removeTarget(const ID& target_id)
    {
        // Remove target from map
        targets_.erase(target_id);
    }

    void MetricsFactory::addCluster(const ID& cluster_id)
    {
        // Create and add cluster
        clusters_.insert({ cluster_id, Cluster() });

        // Initialize cluster metrics
        createClusterMetrics(clusters_[cluster_id].metrics);
        createClusterMetrics(clusters_[cluster_id].prev_metrics);

        // Create cluster geometry subscriber
        clusters_[cluster_id].geometry_sub = topic_tools_->createClusterGeometrySubscriber(cluster_id,
            [this, cluster_id](const ClusterGeometryMsg::SharedPtr msg)
            {
                this->clusterGeometryCallback(cluster_id, msg);
            }, sub_options_with_module_cb_group_);

        // Create cluster metrics publisher
        clusters_[cluster_id].metrics_pub = topic_tools_->createClusterMetricsPublisher(cluster_id);
    }

    void MetricsFactory::removeCluster(const ID& cluster_id)
    {
        // Remove cluster from map
        clusters_.erase(cluster_id);
    }

    // ════════════════════════════════════════════════════════════════════════════
    // CALLBACKS: Callback functions
    // ════════════════════════════════════════════════════════════════════════════

    void MetricsFactory::agentPositionCallback(const ID& agent_id, const PointStampedMsg::SharedPtr msg)
    {
        // Update agent position
        agents_[agent_id].metrics.position = msg->point;
    }

    void MetricsFactory::agentPositionSetpointCallback(const ID& agent_id, const PointStampedMsg::SharedPtr msg)
    {
        // Update agent position setpoint
        agents_[agent_id].metrics.setpoint = msg->point;
    }

    void MetricsFactory::agentTrackingSetpointsCallback(const ID& agent_id, const AgentTrackingSetpointsMsg::SharedPtr msg)
    {
        // Update agent tracking setpoints
        agents_[agent_id].metrics.focals = msg->focals;
        agents_[agent_id].metrics.resolution_factors = msg->resolution_factors;
        agents_[agent_id].metrics.projected_sizes = msg->projected_sizes;
    }

    void MetricsFactory::targetPositionCallback(const ID& target_id, const PointStampedMsg::SharedPtr msg)
    {
        // Update target position
        targets_[target_id].metrics.position = msg->point;
    }

    void MetricsFactory::clusterGeometryCallback(const ID& cluster_id, const ClusterGeometryMsg::SharedPtr msg)
    {
        // Update cluster geometry
        clusters_[cluster_id].metrics.center = msg->center;
        clusters_[cluster_id].metrics.radius = msg->radius;
    }

    // ════════════════════════════════════════════════════════════════════════════
    // UPDATE: Update metrics
    // ════════════════════════════════════════════════════════════════════════════

    void MetricsFactory::update()
    {
        // Compute time step
        auto current_time = RosUtils::now(node_);
        float dt = (current_time - last_update_time_).seconds();
        last_update_time_ = current_time;

        // Iterate over agents
        for (auto& [agent_id, agent] : agents_)
        {
            // Update agent metrics
            updateAgentMetrics(agent.prev_metrics, agent.metrics, dt);

            // Publish agent metrics
            agent.metrics.header.stamp = current_time;
            agent.metrics_pub->publish(agent.metrics);

            // Update previous metrics
            agent.prev_metrics = agent.metrics;
        }

        // Iterate over targets
        for (auto& [target_id, target] : targets_)
        {
            // Update target metrics
            updateTargetMetrics(target.prev_metrics, target.metrics, dt);

            // Publish target metrics
            target.metrics.header.stamp = current_time;
            target.metrics_pub->publish(target.metrics);

            // Update previous metrics
            target.prev_metrics = target.metrics;
        }

        // Iterate over clusters
        for (auto& [cluster_id, cluster] : clusters_)
        {
            // Update cluster metrics
            updateClusterMetrics(cluster.prev_metrics, cluster.metrics, dt);

            // Publish cluster metrics
            cluster.metrics.header.stamp = current_time;
            cluster.metrics_pub->publish(cluster.metrics);

            // Update previous metrics
            cluster.prev_metrics = cluster.metrics;
        }

        // Update global metrics
        updateGlobalMetrics(global_.metrics, dt);

        // Publish global metrics
        global_.metrics.header.stamp = current_time;
        global_.metrics_pub->publish(global_.metrics);
    }

    // ════════════════════════════════════════════════════════════════════════════
    // METRICS METHODS: Metrics methods for creating and updating metrics
    // ════════════════════════════════════════════════════════════════════════════

    void MetricsFactory::createGlobalMetrics(GlobalMetricsMsg& metrics)
    {
        // Create header
        metrics.header = RosUtils::createHeader(node_, transform_tools_->getGlobalFrame());
        // Overall system metrics
        metrics.total_agents = 0;
        metrics.total_targets = 0;
        metrics.total_clusters = 0;
        // Mission metrics
        metrics.mission_time = 0.0f;
    }

    void MetricsFactory::createAgentMetrics(AgentMetricsMsg& metrics)
    {
        // Create header
        metrics.header = RosUtils::createHeader(node_, transform_tools_->getGlobalFrame());
        // Agent data
        metrics.position.x = 0.0f;
        metrics.position.y = 0.0f;
        metrics.position.z = 0.0f;
        metrics.setpoint.x = 0.0f;
        metrics.setpoint.y = 0.0f;
        metrics.setpoint.z = 0.0f;
        // Position and movement metrics
        metrics.distance_traveled = 0.0f;
        metrics.speed = 0.0f;
        // Goal-related metrics
        metrics.distance_to_goal = 0.0f;
        // Mission metrics
        metrics.time_elapsed = 0.0f;
        // Performance metrics
        metrics.average_speed = 0.0f;
    }

    void MetricsFactory::createTargetMetrics(TargetMetricsMsg& metrics)
    {
        // Create header
        metrics.header = RosUtils::createHeader(node_, transform_tools_->getGlobalFrame());
        // Target data
        metrics.position.x = 0.0f;
        metrics.position.y = 0.0f;
        metrics.position.z = 0.0f;
        // Position and movement metrics
        metrics.distance_traveled = 0.0f;
        metrics.speed = 0.0f;
        // Mission metrics
        metrics.time_elapsed = 0.0f;
        // Performance metrics
        metrics.average_speed = 0.0f;
    }

    void MetricsFactory::createClusterMetrics(ClusterMetricsMsg& metrics)
    {
        // Create header
        metrics.header = RosUtils::createHeader(node_, transform_tools_->getGlobalFrame());
        // Cluster data
        metrics.center.x = 0.0f;
        metrics.center.y = 0.0f;
        metrics.center.z = 0.0f;
        metrics.radius = 0.0f;
        // Position and movement metrics
        metrics.distance_traveled = 0.0f;
        metrics.speed = 0.0f;
        // Mission metrics
        metrics.time_elapsed = 0.0f;
        // Performance metrics
        metrics.average_speed = 0.0f;
    }

    void MetricsFactory::updateGlobalMetrics(GlobalMetricsMsg& metrics, float dt)
    {
        // Overall system metrics
        metrics.total_agents = static_cast<int>(agents_.size());
        metrics.total_targets = static_cast<int>(targets_.size());
        metrics.total_clusters = static_cast<int>(clusters_.size());

        // Mission metrics
        metrics.mission_time += dt;
    }

    void MetricsFactory::updateAgentMetrics(AgentMetricsMsg& prev_metrics, AgentMetricsMsg& curr_metrics, float dt)
    {
        // Get current and previous data as vectors
        const Vector3r curr_position = RosUtils::fromMsg(curr_metrics.position);
        const Vector3r prev_position = RosUtils::fromMsg(prev_metrics.position);
        const Vector3r curr_setpoint = RosUtils::fromMsg(curr_metrics.setpoint);

        // Update position and movement metrics
        const float distance_change = MathUtils::distance(curr_position, prev_position);
        curr_metrics.distance_traveled += distance_change;  // m
        curr_metrics.speed = distance_change / dt;          // m/s

        // Goal-related metrics
        const float distance_to_goal = MathUtils::distance(curr_position, curr_setpoint);
        curr_metrics.distance_to_goal = distance_to_goal; // m

        // Mission metrics
        curr_metrics.time_elapsed += dt;

        // Performance metrics
        curr_metrics.average_speed = curr_metrics.distance_traveled / curr_metrics.time_elapsed;
    }

    void MetricsFactory::updateTargetMetrics(TargetMetricsMsg& prev_metrics, TargetMetricsMsg& curr_metrics, float dt)
    {
        // Get current and previous data as vectors
        const Vector3r curr_position = RosUtils::fromMsg(curr_metrics.position);
        const Vector3r prev_position = RosUtils::fromMsg(prev_metrics.position);

        // Update position and movement metrics
        const float distance_change = MathUtils::distance(curr_position, prev_position);
        curr_metrics.distance_traveled += distance_change;  // m
        curr_metrics.speed = distance_change / dt;          // m/s

        // Mission metrics
        curr_metrics.time_elapsed += dt;

        // Performance metrics
        curr_metrics.average_speed = curr_metrics.distance_traveled / curr_metrics.time_elapsed;
    }

    void MetricsFactory::updateClusterMetrics(ClusterMetricsMsg& prev_metrics, ClusterMetricsMsg& curr_metrics, float dt)
    {
        // Get current and previous data as vectors
        const Vector3r curr_position = RosUtils::fromMsg(curr_metrics.center);
        const Vector3r prev_position = RosUtils::fromMsg(prev_metrics.center);

        // Update position and movement metrics
        const float distance_change = MathUtils::distance(curr_position, prev_position);
        curr_metrics.distance_traveled += distance_change;  // m
        curr_metrics.speed = distance_change / dt;          // m/s

        // Mission metrics
        curr_metrics.time_elapsed += dt;

        // Performance metrics
        curr_metrics.average_speed = curr_metrics.distance_traveled / curr_metrics.time_elapsed;
    }

} // namespace flychams::simulation