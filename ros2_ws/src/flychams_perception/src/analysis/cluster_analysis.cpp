#include "flychams_perception/analysis/cluster_analysis.hpp"

using namespace flychams::core;

namespace flychams::perception
{
	// ════════════════════════════════════════════════════════════════════════════
	// CONSTRUCTOR: Constructor and destructor
	// ════════════════════════════════════════════════════════════════════════════

	void ClusterAnalysis::onInit()
	{
		// Get parameters from parameter server
		// Get update rate
		update_rate_ = RosUtils::getParameterOr<float>(node_, "cluster_analysis.analysis_rate", 20.0f);
		// Get circle parameters
		min_circle_radius_ = RosUtils::getParameterOr<float>(node_, "cluster_analysis.min_circle_radius", 0.10f);
		margin_circle_radius_ = RosUtils::getParameterOr<float>(node_, "cluster_analysis.margin_circle_radius", 0.05f);

		// Initialize data
		targets_.clear();
		clusters_.clear();

		// Set update timer
		update_timer_ = RosUtils::createTimer(node_, update_rate_,
			std::bind(&ClusterAnalysis::update, this), module_cb_group_);
	}

	void ClusterAnalysis::onShutdown()
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

	void ClusterAnalysis::addCluster(const ID& cluster_id)
	{
		// Create and add cluster
		clusters_.insert({ cluster_id, Cluster() });

		// Create cluster geometry publisher
		clusters_[cluster_id].geometry_pub = topic_tools_->createClusterGeometryPublisher(cluster_id);
	}

	void ClusterAnalysis::removeCluster(const ID& cluster_id)
	{
		// Remove cluster from map
		clusters_.erase(cluster_id);
	}

	void ClusterAnalysis::addTarget(const ID& target_id)
	{
		// Create and add target
		targets_.insert({ target_id, Target() });

		// Create target assignment subscriber
		targets_[target_id].assignment_sub = topic_tools_->createTargetAssignmentSubscriber(target_id,
			[this, target_id](const StringMsg::SharedPtr msg)
			{
				this->targetAssignmentCallback(target_id, msg);
			}, sub_options_with_module_cb_group_);

		// Create target true position subscriber
		targets_[target_id].position_sub = topic_tools_->createTargetTruePositionSubscriber(target_id,
			[this, target_id](const PointStampedMsg::SharedPtr msg)
			{
				this->targetPositionCallback(target_id, msg);
			}, sub_options_with_module_cb_group_);
	}

	void ClusterAnalysis::removeTarget(const ID& target_id)
	{
		// Remove target from map
		targets_.erase(target_id);
	}

	// ════════════════════════════════════════════════════════════════════════════
	// CALLBACKS: Callback functions
	// ════════════════════════════════════════════════════════════════════════════

	void ClusterAnalysis::targetAssignmentCallback(const core::ID& target_id, const core::StringMsg::SharedPtr msg)
	{
		// Update target assignment
		targets_[target_id].assigned_id = msg->data;
		targets_[target_id].has_assignment = true;
	}

	void ClusterAnalysis::targetPositionCallback(const core::ID& target_id, const core::PointStampedMsg::SharedPtr msg)
	{
		// Update target position
		targets_[target_id].position = msg->point;
		targets_[target_id].has_position = true;
	}

	// ════════════════════════════════════════════════════════════════════════════
	// UPDATE: Update analysis
	// ════════════════════════════════════════════════════════════════════════════

	void ClusterAnalysis::update()
	{
		// Check if we have a valid target assignments and positions
		for (const auto& [target_id, target] : targets_)
		{
			if (!target.has_assignment || !target.has_position)
			{
				RCLCPP_WARN(node_->get_logger(), "Cluster analysis: Target %s has no assignment or position", target_id.c_str());
				return; // Skip analysis if we don't have a valid target assignment and position
			}
		}

		// Iterate over all clusters
		for (auto& [cluster_id, cluster] : clusters_)
		{
			// Assign targets to each cluster
			const auto& points = assignCluster(cluster_id, targets_);

			// Calculate enclosing circle (minimum enclosing circle with enforced limits)
			const auto& [center, radius] = calculateEnclosingCircle(points, min_circle_radius_, margin_circle_radius_);

			// Update cluster geometry
			cluster.geometry.center.x = center.x();
			cluster.geometry.center.y = center.y();
			cluster.geometry.radius = radius;

			// Publish cluster geometry
			cluster.geometry_pub->publish(cluster.geometry);
		}
	}

	// ════════════════════════════════════════════════════════════════════════════
	// ANALYSIS: Analysis methods
	// ════════════════════════════════════════════════════════════════════════════

	std::vector<core::PointMsg> ClusterAnalysis::assignCluster(const core::ID& cluster_id, const std::unordered_map<core::ID, Target>& targets)
	{
		std::vector<core::PointMsg> points;
		for (const auto& [target_id, target] : targets)
		{
			if (target.assigned_id == cluster_id)
			{
				points.push_back(target.position);
			}
		}

		return points;
	}

	std::pair<core::Vector2r, float> ClusterAnalysis::calculateEnclosingCircle(const std::vector<core::PointMsg>& points, const float& min_radius, const float& margin_radius)
	{
		// Get target count
		int n = static_cast<int>(points.size());

		// Handle edge cases
		if (n == 0)
			return { {0.0f, 0.0f}, min_radius + margin_radius };
		if (n == 1)
			return { {points[0].x, points[0].y}, min_radius + margin_radius };

		// Minimal enclosing circle (Welzl's algorithm)
		std::vector<WelzlsCircle::Point2D> points_welzl;
		points_welzl.reserve(n);
		for (int i = 0; i < n; i++)
		{
			points_welzl.push_back({ static_cast<float>(points[i].x), static_cast<float>(points[i].y) });
		}
		WelzlsCircle::Circle mec = WelzlsCircle::welzl(points_welzl);

		// Set the enclosing radius, ensuring it's at least the minimum radius
		mec.R = std::max(mec.R, min_radius) + margin_radius;

		return { {mec.C.x, mec.C.y}, mec.R };
	}

} // namespace flychams::perception
