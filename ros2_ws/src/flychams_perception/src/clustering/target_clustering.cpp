#include "flychams_perception/clustering/target_clustering.hpp"

using namespace flychams::core;

namespace flychams::perception
{
	// ════════════════════════════════════════════════════════════════════════════
	// CONSTRUCTOR: Constructor and destructor
	// ════════════════════════════════════════════════════════════════════════════

	void TargetClustering::onInit()
	{
		// Get parameters from parameter server
		// Get update rate
		update_rate_ = RosUtils::getParameterOr<float>(node_, "target_clustering.update_rate", 3.33f);
		// Get persistence parameters
		float ini_bonding_coef = RosUtils::getParameterOr<float>(node_, "target_clustering.ini_bonding_coef", 1.0f);
		float max_bonding_coef = RosUtils::getParameterOr<float>(node_, "target_clustering.max_bonding_coef", 1.0f);
		float bonding_coef_time_to_max = RosUtils::getParameterOr<float>(node_, "target_clustering.bonding_coef_time_to_max", 100.0f);
		float max_hysteresis_ratio = RosUtils::getParameterOr<float>(node_, "target_clustering.max_hysteresis_ratio", 0.4f);
		float min_hysteresis_ratio = RosUtils::getParameterOr<float>(node_, "target_clustering.min_hysteresis_ratio", 0.2f);

		// Compute command timeout
		cmd_timeout_ = (1.0f / update_rate_) * 1.25f;

		// Initialize data
		targets_.clear();
		clusters_.clear();

		// Initialize K-Means
		k_means_.reset();
		k_means_.setParameters(ini_bonding_coef, max_bonding_coef, bonding_coef_time_to_max, max_hysteresis_ratio, min_hysteresis_ratio);

		// Set update timer
		last_update_time_ = RosUtils::now(node_);
		update_timer_ = RosUtils::createTimer(node_, update_rate_,
			std::bind(&TargetClustering::update, this), module_cb_group_);
	}

	void TargetClustering::onShutdown()
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

	void TargetClustering::addCluster(const ID& cluster_id)
	{
		// Create and add cluster
		clusters_.insert(cluster_id);

		// Reset K-Means
		k_means_.reset();
	}

	void TargetClustering::removeCluster(const ID& cluster_id)
	{
		// Remove cluster from map
		clusters_.erase(cluster_id);

		// Reset K-Means
		k_means_.reset();
	}

	void TargetClustering::addTarget(const ID& target_id)
	{
		// Create and add target
		targets_.insert({ target_id, Target() });

		// Create target true position subscriber
		targets_[target_id].position_sub = topic_tools_->createTargetTruePositionSubscriber(target_id,
			[this, target_id](const PointStampedMsg::SharedPtr msg)
			{
				this->targetPositionCallback(target_id, msg);
			}, sub_options_with_module_cb_group_);

		// Create target assignment publisher
		targets_[target_id].assignment_pub = topic_tools_->createTargetAssignmentPublisher(target_id);

		// Reset K-Means
		k_means_.reset();
	}

	void TargetClustering::removeTarget(const ID& target_id)
	{
		// Remove target from map
		targets_.erase(target_id);

		// Reset K-Means
		k_means_.reset();
	}

	// ════════════════════════════════════════════════════════════════════════════
	// CALLBACKS: Callback functions
	// ════════════════════════════════════════════════════════════════════════════

	void TargetClustering::targetPositionCallback(const ID& target_id, const PointStampedMsg::SharedPtr msg)
	{
		// Update target position
		targets_[target_id].position = msg->point;
		targets_[target_id].has_position = true;
	}

	// ════════════════════════════════════════════════════════════════════════════
	// UPDATE: Update clustering
	// ════════════════════════════════════════════════════════════════════════════

	void TargetClustering::update()
	{
		// Check if we have a valid target positions
		for (const auto& [target_id, target] : targets_)
		{
			if (!target.has_position)
			{
				RCLCPP_WARN(node_->get_logger(), "Target clustering: Target %s has no position", target_id.c_str());
				return; // Skip clustering if we don't have a valid target position
			}
		}

		// Compute time step
		auto current_time = RosUtils::now(node_);
		float dt = (current_time - last_update_time_).seconds();
		last_update_time_ = current_time;

		// Limit dt to prevent extreme values after pauses
		dt = std::min(dt, cmd_timeout_);

		// Create points map
		KMeansMod::Points points;
		for (const auto& [target_id, target] : targets_)
		{
			points.insert({ target_id, RosUtils::fromMsg(target.position) });
		}

		// Perform clustering with available points
		RCLCPP_INFO(node_->get_logger(), "Target clustering: Running K-Means with %zu points and %zu clusters", points.size(), clusters_.size());
		const auto& assignments = k_means_.run(points, clusters_, dt);

		// Publish assignments
		for (const auto& [target_id, cluster_id] : assignments)
		{
			// Create assignment message
			StringMsg assignment_msg;
			assignment_msg.data = cluster_id;

			// Publish assignment
			targets_[target_id].assignment_pub->publish(assignment_msg);

			// Log assignment
			RCLCPP_INFO(node_->get_logger(), "Target clustering: Target %s assigned to cluster %s", target_id.c_str(), cluster_id.c_str());
		}
	}

} // namespace flychams::perception