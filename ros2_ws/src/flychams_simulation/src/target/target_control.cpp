#include "flychams_simulation/target/target_control.hpp"

using namespace flychams::core;

namespace flychams::simulation
{
    // ════════════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR: Constructor and destructor
    // ════════════════════════════════════════════════════════════════════════════

    void TargetControl::onInit()
    {
        // Get parameters from parameter server
        // Get update rate
        update_rate_ = RosUtils::getParameterOr<float>(node_, "target_control.update_rate", 20.0f);

        // Initialize data
        targets_.clear();
        clusters_.clear();

        // Set update timer
        update_timer_ = RosUtils::createTimer(node_, update_rate_,
            std::bind(&TargetControl::update, this), module_cb_group_);
    }

    void TargetControl::onShutdown()
    {
        // Destroy target and cluster maps
        targets_.clear();
        clusters_.clear();
        // Destroy update timer
        update_timer_.reset();
    }

    // ════════════════════════════════════════════════════════════════════════════
    // PUBLIC METHODS: Public methods for adding/removing clusters and targets
    // ════════════════════════════════════════════════════════════════════════════

    void TargetControl::addTarget(const ID& target_id)
    {
        // Create and add target
        targets_.insert({ target_id, Target() });

        // Create target position subscriber
        targets_[target_id].position_sub = topic_tools_->createTargetTruePositionSubscriber(target_id,
            [this, target_id](const PointStampedMsg::SharedPtr msg)
            {
                this->targetPositionCallback(target_id, msg);
            }, sub_options_with_module_cb_group_);

        // Spawn target in simulation
        PointMsg initial_position;
        initial_position.x = -500.0f;
        initial_position.y = -500.0f;
        initial_position.z = 0.0f;
        spawnTarget(target_id, initial_position, config_tools_->getTarget(target_id)->type);

        // Delay to ensure target is spawned
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    void TargetControl::removeTarget(const ID& target_id)
    {
        // Remove target from map
        targets_.erase(target_id);
    }

    void TargetControl::addCluster(const ID& cluster_id)
    {
        // Create and add cluster
        clusters_.insert({ cluster_id, Cluster() });

        // Create cluster geometry subscriber
        clusters_[cluster_id].geometry_sub = topic_tools_->createClusterGeometrySubscriber(cluster_id,
            [this, cluster_id](const ClusterGeometryMsg::SharedPtr msg)
            {
                this->clusterGeometryCallback(cluster_id, msg);
            }, sub_options_with_module_cb_group_);

        // Spawn cluster in simulation
        PointMsg initial_position;
        initial_position.x = -500.0f;
        initial_position.y = -500.0f;
        initial_position.z = 0.0f;
        float initial_radius = 1.0f;
        spawnCluster(cluster_id, initial_position, initial_radius);

        // Delay to ensure cluster is spawned
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    void TargetControl::removeCluster(const ID& cluster_id)
    {
        // Remove cluster from map
        clusters_.erase(cluster_id);
    }

    // ════════════════════════════════════════════════════════════════════════════
    // CALLBACKS: Callback functions
    // ════════════════════════════════════════════════════════════════════════════

    void TargetControl::targetPositionCallback(const ID& target_id, const PointStampedMsg::SharedPtr msg)
    {
        // Update target position
        targets_[target_id].position = msg->point;
        targets_[target_id].has_position = true;
    }

    void TargetControl::clusterGeometryCallback(const ID& cluster_id, const ClusterGeometryMsg::SharedPtr msg)
    {
        // Update cluster geometry
        clusters_[cluster_id].position = msg->center;
        clusters_[cluster_id].radius = msg->radius;
        clusters_[cluster_id].has_geometry = true;
    }

    // ════════════════════════════════════════════════════════════════════════════
    // UPDATE: Update control
    // ════════════════════════════════════════════════════════════════════════════

    void TargetControl::update()
    {
        // Update targets in simulation
        updateTargets();

        // Delay to ensure targets are updated
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // Update clusters in simulation
        updateClusters();
    }

    // ════════════════════════════════════════════════════════════════════════════
    // CONTROL: Control methods
    // ════════════════════════════════════════════════════════════════════════════

    void TargetControl::destroyTargets()
    {
        framework_tools_->removeAllTargets();
    }

    void TargetControl::destroyClusters()
    {
        framework_tools_->removeAllClusters();
    }

    void TargetControl::spawnTarget(const ID& target_id, const PointMsg& initial_position, const TargetType& target_type)
    {
        // Highlight color (red with low alpha)
        ColorMsg highlight_color;
        highlight_color.r = 1.0f;
        highlight_color.g = 0.0f;
        highlight_color.b = 0.0f;
        highlight_color.a = 0.005f;

        // Add target to simulation
        framework_tools_->addTargetGroup({ target_id }, { target_type }, { initial_position }, true, { highlight_color });
    }

    void TargetControl::spawnCluster(const ID& cluster_id, const PointMsg& initial_center, const float& initial_radius)
    {
        // // Highlight color (cyan with low alpha)
        // ColorMsg highlight_color;
        // highlight_color.r = 0.0f;
        // highlight_color.g = 1.0f;
        // highlight_color.b = 1.0f;
        // highlight_color.a = 0.005f;

        // Highlight color (orange with medium alpha)
        ColorMsg highlight_color;
        highlight_color.r = 1.0f;
        highlight_color.g = 0.5f;
        highlight_color.b = 0.0f;
        highlight_color.a = 0.15f;

        // Add cluster to simulation
        framework_tools_->addClusterGroup({ cluster_id }, { initial_center }, { initial_radius }, true, { highlight_color });
    }

    void TargetControl::updateTargets()
    {
        // Iterate over all targets
        std::vector<ID> target_ids;
        std::vector<PointMsg> target_positions;
        for (const auto& [target_id, target] : targets_)
        {
            // Check if we have a valid target position
            if (!target.has_position)
            {
                RCLCPP_WARN(node_->get_logger(), "Target control: Target %s has no true position", target_id.c_str());
                continue; // Skip if we don't have a valid target position
            }

            // Add target to command vectors
            target_ids.push_back(target_id);
            target_positions.push_back(target.position);
        }

        // Send target commands to simulation
        framework_tools_->updateTargetGroup(target_ids, target_positions);
    }

    void TargetControl::updateClusters()
    {
        // Iterate over all clusters
        std::vector<ID> cluster_ids;
        std::vector<PointMsg> cluster_positions;
        std::vector<float> cluster_radii;
        for (const auto& [cluster_id, cluster] : clusters_)
        {
            // Check if we have a valid cluster geometry
            if (!cluster.has_geometry)
            {
                RCLCPP_WARN(node_->get_logger(), "Target control: Cluster %s has no geometry", cluster_id.c_str());
                continue; // Skip if we don't have a valid cluster geometry
            }

            // Add cluster to command vectors
            cluster_ids.push_back(cluster_id);
            cluster_positions.push_back(cluster.position);
            cluster_radii.push_back(cluster.radius);
        }

        // Update clusters in simulation
        framework_tools_->updateClusterGroup(cluster_ids, cluster_positions, cluster_radii);
    }

} // namespace flychams::simulation