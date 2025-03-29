#include "flychams_dashboard/marker/marker_factory.hpp"

using namespace flychams::core;

namespace flychams::dashboard
{
    // ════════════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR: Constructor and destructor
    // ════════════════════════════════════════════════════════════════════════════

    void MarkerFactory::onInit()
    {
        // Get parameters from parameter server
        // Get update rate
        update_rate_ = RosUtils::getParameterOr<float>(node_, "marker_factory.update_rate", 10.0f);

        // Initialize data
        agents_.clear();
        targets_.clear();
        clusters_.clear();

        // Set update timer
        update_timer_ = RosUtils::createTimer(node_, update_rate_,
            std::bind(&MarkerFactory::update, this), module_cb_group_);
    }

    void MarkerFactory::onShutdown()
    {
        // Destroy agents, targets and clusters
        agents_.clear();
        targets_.clear();
        clusters_.clear();
        // Destroy update timer
        update_timer_.reset();
    }

    // ════════════════════════════════════════════════════════════════════════════
    // PUBLIC METHODS: Public methods for adding/removing agents, targets and clusters
    // ════════════════════════════════════════════════════════════════════════════

    void MarkerFactory::addAgent(const ID& agent_id)
    {
        // Create and add agent
        agents_.insert({ agent_id, Agent() });

        // Initialize agent markers
        createAgentMarkers(agents_[agent_id].markers);

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

        // Create agent marker publisher
        agents_[agent_id].marker_pub = topic_tools_->createAgentMarkersPublisher(agent_id);
    }

    void MarkerFactory::removeAgent(const ID& agent_id)
    {
        // Remove agent from map
        agents_.erase(agent_id);
    }

    void MarkerFactory::addTarget(const ID& target_id)
    {
        // Create and add target
        targets_.insert({ target_id, Target() });

        // Initialize target markers
        createTargetMarkers(targets_[target_id].markers);

        // Create target position subscriber
        targets_[target_id].position_sub = topic_tools_->createTargetTruePositionSubscriber(target_id,
            [this, target_id](const PointStampedMsg::SharedPtr msg)
            {
                this->targetPositionCallback(target_id, msg);
            }, sub_options_with_module_cb_group_);

        // Create target marker publisher
        targets_[target_id].marker_pub = topic_tools_->createTargetMarkersPublisher(target_id);
    }

    void MarkerFactory::removeTarget(const ID& target_id)
    {
        // Remove target from map
        targets_.erase(target_id);
    }

    void MarkerFactory::addCluster(const ID& cluster_id)
    {
        // Create and add cluster
        clusters_.insert({ cluster_id, Cluster() });

        // Initialize cluster markers
        createClusterMarkers(clusters_[cluster_id].markers);

        // Create cluster geometry subscriber
        clusters_[cluster_id].geometry_sub = topic_tools_->createClusterGeometrySubscriber(cluster_id,
            [this, cluster_id](const ClusterGeometryMsg::SharedPtr msg)
            {
                this->clusterGeometryCallback(cluster_id, msg);
            }, sub_options_with_module_cb_group_);

        // Create cluster marker publisher
        clusters_[cluster_id].marker_pub = topic_tools_->createClusterMarkersPublisher(cluster_id);
    }

    void MarkerFactory::removeCluster(const ID& cluster_id)
    {
        // Remove cluster from map
        clusters_.erase(cluster_id);
    }

    // ════════════════════════════════════════════════════════════════════════════
    // CALLBACKS: Callback functions
    // ════════════════════════════════════════════════════════════════════════════

    void MarkerFactory::agentPositionCallback(const ID& agent_id, const PointStampedMsg::SharedPtr msg)
    {
        // Update agent position
        agents_[agent_id].markers.markers[0].header.stamp = RosUtils::now(node_);
        agents_[agent_id].markers.markers[0].pose.position = msg->point;
    }

    void MarkerFactory::agentPositionSetpointCallback(const ID& agent_id, const PointStampedMsg::SharedPtr msg)
    {
        // Update agent position setpoint
        agents_[agent_id].markers.markers[1].header.stamp = RosUtils::now(node_);
        agents_[agent_id].markers.markers[1].pose.position = msg->point;
    }

    void MarkerFactory::targetPositionCallback(const ID& target_id, const PointStampedMsg::SharedPtr msg)
    {
        // Update target position
        targets_[target_id].markers.markers[0].header.stamp = RosUtils::now(node_);
        targets_[target_id].markers.markers[0].pose.position = msg->point;
    }

    void MarkerFactory::clusterGeometryCallback(const ID& cluster_id, const ClusterGeometryMsg::SharedPtr msg)
    {
        // Update cluster geometry
        const float& cx = msg->center.x;
        const float& cy = msg->center.y;
        const float& cz = msg->center.z;
        const float& r = msg->radius;

        // Update X points
        const float half_size = BASE_MARKER_SIZE * 0.5f;
        // First line of X (top-left to bottom-right)
        clusters_[cluster_id].markers.markers[0].header.stamp = RosUtils::now(node_);
        auto& p1 = clusters_[cluster_id].markers.markers[0].points[0];
        p1.x = cx - half_size;
        p1.y = cy - half_size;
        p1.z = cz;
        auto& p2 = clusters_[cluster_id].markers.markers[0].points[1];
        p2.x = cx + half_size;
        p2.y = cy + half_size;
        p2.z = cz;
        // Second line of X (top-right to bottom-left)
        auto& p3 = clusters_[cluster_id].markers.markers[0].points[2];
        p3.x = cx + half_size;
        p3.y = cy - half_size;
        p3.z = cz;
        auto& p4 = clusters_[cluster_id].markers.markers[0].points[3];
        p4.x = cx - half_size;
        p4.y = cy + half_size;
        p4.z = cz;

        // Update cluster boundary
        clusters_[cluster_id].markers.markers[1].header.stamp = RosUtils::now(node_);
        clusters_[cluster_id].markers.markers[1].pose.position = msg->center;
        clusters_[cluster_id].markers.markers[1].scale.x = 2.0f * r;
        clusters_[cluster_id].markers.markers[1].scale.y = 2.0f * r;
    }

    // ════════════════════════════════════════════════════════════════════════════
    // UPDATE: Update markers
    // ════════════════════════════════════════════════════════════════════════════

    void MarkerFactory::update()
    {
        // Iterate over agents
        for (auto& [agent_id, agent] : agents_)
        {
            // Publish agent markers
            agent.marker_pub->publish(agent.markers);
        }

        // Iterate over targets
        for (auto& [target_id, target] : targets_)
        {
            // Publish target markers
            target.marker_pub->publish(target.markers);
        }

        // Iterate over clusters
        for (auto& [cluster_id, cluster] : clusters_)
        {
            // Publish cluster markers
            cluster.marker_pub->publish(cluster.markers);
        }
    }

    // ════════════════════════════════════════════════════════════════════════════
    // MARKERS METHODS: Markers methods for creating markers
    // ════════════════════════════════════════════════════════════════════════════

    void MarkerFactory::createAgentMarkers(MarkerArrayMsg& markers)
    {
        // Create marker for UAV body (a blue point)
        MarkerMsg body_marker;
        body_marker.header = RosUtils::createHeader(node_, transform_tools_->getGlobalFrame());
        body_marker.id = 0;
        body_marker.type = MarkerMsg::SPHERE;
        body_marker.action = MarkerMsg::ADD;
        body_marker.lifetime = rclcpp::Duration::from_seconds(1.0f / update_rate_ * 1.25f);
        // Set initial pose
        body_marker.pose.position.x = 0.0f;
        body_marker.pose.position.y = 0.0f;
        body_marker.pose.position.z = 0.0f;
        body_marker.pose.orientation.w = 1.0f;
        // Set scale
        body_marker.scale.x = BASE_MARKER_SIZE * 1.0f; // length
        body_marker.scale.y = BASE_MARKER_SIZE * 1.0f; // width
        body_marker.scale.z = BASE_MARKER_SIZE * 1.0f; // height
        // Set color (blue)
        body_marker.color.r = 0.0f;
        body_marker.color.g = 0.0f;
        body_marker.color.b = 1.0f;
        body_marker.color.a = BASE_MARKER_ALPHA;
        // Add marker to array
        markers.markers.push_back(body_marker);

        // Create marker for UAV goal (a green point)
        MarkerMsg goal_marker;
        goal_marker.header = RosUtils::createHeader(node_, transform_tools_->getGlobalFrame());
        goal_marker.id = 1;
        goal_marker.type = MarkerMsg::SPHERE;
        goal_marker.action = MarkerMsg::ADD;
        goal_marker.lifetime = rclcpp::Duration::from_seconds(1.0f / update_rate_ * 1.25f);
        // Set initial pose
        goal_marker.pose.position.x = 0.0f;
        goal_marker.pose.position.y = 0.0f;
        goal_marker.pose.position.z = 0.0f;
        goal_marker.pose.orientation.w = 1.0f;
        // Set scale
        goal_marker.scale.x = BASE_MARKER_SIZE * 1.0f; // length
        goal_marker.scale.y = BASE_MARKER_SIZE * 1.0f; // width
        goal_marker.scale.z = BASE_MARKER_SIZE * 1.0f; // height
        // Set color (yellow)
        goal_marker.color.r = 1.0f;
        goal_marker.color.g = 1.0f;
        goal_marker.color.b = 0.0f;
        goal_marker.color.a = BASE_MARKER_ALPHA;
        // Add marker to array
        markers.markers.push_back(goal_marker);
    }

    void MarkerFactory::createTargetMarkers(MarkerArrayMsg& markers)
    {
        // Create marker for target (a red point)
        MarkerMsg target_marker;
        target_marker.header = RosUtils::createHeader(node_, transform_tools_->getGlobalFrame());
        target_marker.id = 0;
        target_marker.type = MarkerMsg::SPHERE;
        target_marker.action = MarkerMsg::ADD;
        target_marker.lifetime = rclcpp::Duration::from_seconds(1.0f / update_rate_ * 1.25f);
        // Set initial pose
        target_marker.pose.position.x = 0.0f;
        target_marker.pose.position.y = 0.0f;
        target_marker.pose.position.z = 0.0f;
        target_marker.pose.orientation.w = 1.0f;
        // Set scale
        target_marker.scale.x = BASE_MARKER_SIZE * 0.7f; // length
        target_marker.scale.y = BASE_MARKER_SIZE * 0.7f; // width
        target_marker.scale.z = BASE_MARKER_SIZE * 0.7f; // height
        // Set color (red)
        target_marker.color.r = 1.0f;
        target_marker.color.g = 0.0f;
        target_marker.color.b = 0.0f;
        target_marker.color.a = BASE_MARKER_ALPHA;
        // Add marker to array
        markers.markers.push_back(target_marker);
    }

    void MarkerFactory::createClusterMarkers(MarkerArrayMsg& markers)
    {
        // Create marker for cluster center (a yellow X)
        MarkerMsg x_marker;
        x_marker.header = RosUtils::createHeader(node_, transform_tools_->getGlobalFrame());
        x_marker.id = 0;
        x_marker.type = MarkerMsg::LINE_LIST;
        x_marker.action = MarkerMsg::ADD;
        x_marker.lifetime = rclcpp::Duration::from_seconds(1.0f / update_rate_ * 1.25f);
        // Set initial pose
        x_marker.pose.position.x = 0.0f;
        x_marker.pose.position.y = 0.0f;
        x_marker.pose.position.z = 0.0f;
        x_marker.pose.orientation.w = 1.0f;
        // Create the X shape using line segments
        float half_size = BASE_MARKER_SIZE * 0.5f;
        PointMsg p1, p2, p3, p4;
        // First line of X (top-left to bottom-right)
        p1.x = 0.0f - half_size;
        p1.y = 0.0f - half_size;
        p1.z = 0.0f;
        p2.x = 0.0f + half_size;
        p2.y = 0.0f + half_size;
        p2.z = 0.0f;
        // Second line of X (top-right to bottom-left)
        p3.x = 0.0f + half_size;
        p3.y = 0.0f - half_size;
        p3.z = 0.0f;
        p4.x = 0.0f - half_size;
        p4.y = 0.0f + half_size;
        p4.z = 0.0f;
        // Add points to marker
        x_marker.points.push_back(p1);
        x_marker.points.push_back(p2);
        x_marker.points.push_back(p3);
        x_marker.points.push_back(p4);
        // Set scale
        x_marker.scale.x = BASE_LINE_WIDTH * 0.5f;
        // Set color (yellow)
        x_marker.color.r = 1.0f;
        x_marker.color.g = 1.0f;
        x_marker.color.b = 0.0f;
        x_marker.color.a = BASE_MARKER_ALPHA;
        // Add marker to array
        markers.markers.push_back(x_marker);

        // Create marker for cluster boundary (a cyan semi-transparent cylinder)
        MarkerMsg boundary_marker;
        boundary_marker.header = RosUtils::createHeader(node_, transform_tools_->getGlobalFrame());
        boundary_marker.id = 1;
        boundary_marker.type = MarkerMsg::CYLINDER;
        boundary_marker.action = MarkerMsg::ADD;
        boundary_marker.lifetime = rclcpp::Duration::from_seconds(1.0f / update_rate_ * 1.25f);
        // Set initial pose
        boundary_marker.pose.position.x = 0.0f;
        boundary_marker.pose.position.y = 0.0f;
        boundary_marker.pose.position.z = 0.0f;
        boundary_marker.pose.orientation.w = 1.0f;
        // Set initial scale
        boundary_marker.scale.x = 0.0f;  // length
        boundary_marker.scale.y = 0.0f;  // width
        boundary_marker.scale.z = 0.05f; // height
        // Set color (cyan, semi-transparent)
        boundary_marker.color.r = 0.0f;
        boundary_marker.color.g = 1.0f;
        boundary_marker.color.b = 1.0f;
        boundary_marker.color.a = 0.15f;
        // Add marker to array
        markers.markers.push_back(boundary_marker);
    }

} // namespace flychams::dashboard