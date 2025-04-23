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
		clusters_.clear();
		targets_.clear();

		// Set update timer
		update_timer_ = RosUtils::createTimer(node_, update_rate_,
			std::bind(&ClusterAnalysis::update, this), module_cb_group_);
	}

	void ClusterAnalysis::onShutdown()
	{
		// Destroy clusters and targets
		clusters_.clear();
		targets_.clear();
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

		// Create cluster assignment subscriber
		clusters_[cluster_id].assignment_sub = topic_tools_->createClusterAssignmentSubscriber(cluster_id,
			[this, cluster_id](const ClusterAssignmentMsg::SharedPtr msg)
			{
				this->clusterAssignmentCallback(cluster_id, msg);
			}, sub_options_with_module_cb_group_);

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

	void ClusterAnalysis::clusterAssignmentCallback(const core::ID& cluster_id, const core::ClusterAssignmentMsg::SharedPtr msg)
	{
		// Update cluster assignment
		clusters_[cluster_id].assignment = msg->target_ids;
		clusters_[cluster_id].has_assignment = true;
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
		// Check if we have a valid assignment and position for each target and cluster
        for (const auto& [cluster_id, cluster] : clusters_)
        {
            if (!cluster.has_assignment)
            {
                RCLCPP_WARN(node_->get_logger(), "Cluster analysis: Cluster %s has no assignment", cluster_id.c_str());
                return; // Skip updating if we don't have a valid cluster assignment
            }
        }
		for (const auto& [target_id, target] : targets_)
		{
			if (!target.has_position)
			{
				RCLCPP_WARN(node_->get_logger(), "Cluster analysis: Target %s has no position", target_id.c_str());
				return; // Skip analysis if we don't have a valid target position
			}
		}

		// Compute cluster geometry and publish
		for (auto& [cluster_id, cluster] : clusters_)
		{
            // Iterate over the assignment and get the points
            int n = static_cast<int>(cluster.assignment.size());
			Matrix3Xr tab_P(3, n);
            for (int i = 0; i < n; i++)
            {
                const auto& target = targets_[cluster.assignment[i]];
                tab_P.col(i) = RosUtils::fromMsg(target.position);
            }

			// Calculate enclosing circle (minimum enclosing circle with enforced limits)
			const auto& [center, radius] = calculateEnclosingCircle(tab_P, min_circle_radius_, margin_circle_radius_);

			// Create geometry message with calculated center and radius
            ClusterGeometryMsg msg;
            msg.header = RosUtils::createHeader(node_, transform_tools_->getGlobalFrame());
			msg.center.x = center.x();
			msg.center.y = center.y();
			msg.center.z = 0.0f;
			msg.radius = radius;

			// Publish cluster geometry
			cluster.geometry_pub->publish(msg);
		}
	}

	// ════════════════════════════════════════════════════════════════════════════
	// ANALYSIS: Analysis methods
	// ════════════════════════════════════════════════════════════════════════════

	std::pair<core::Vector2r, float> ClusterAnalysis::calculateEnclosingCircle(const core::Matrix3Xr& tab_P, const float& min_radius, const float& margin_radius)
	{
		// Get number of points
		int n = tab_P.cols();

		// Handle edge cases
		if (n == 0)
			return { {0.0f, 0.0f}, min_radius + margin_radius };
		if (n == 1)
			return { {tab_P(0, 0), tab_P(1, 0)}, min_radius + margin_radius };

		// Minimal enclosing circle (Welzl's algorithm)
		std::vector<WelzlsCircle::Point2D> points_welzl;
		points_welzl.reserve(n);
		for (int i = 0; i < n; i++)
		{
			points_welzl.push_back({ tab_P(0, i), tab_P(1, i) });
		}
		WelzlsCircle::Circle mec = WelzlsCircle::welzl(points_welzl);

		// Set the enclosing radius, ensuring it's at least the minimum radius
		mec.R = std::max(mec.R, min_radius) + margin_radius;

		return { {mec.C.x, mec.C.y}, mec.R };
	}

} // namespace flychams::perception
