#include "flychams_perception/clustering/target_clustering.hpp"

using namespace std::chrono_literals;
using namespace flychams::core;

namespace flychams::perception
{
	// ════════════════════════════════════════════════════════════════════════════
	// CONSTRUCTOR: Constructor and destructor
	// ════════════════════════════════════════════════════════════════════════════

	void TargetClustering::onInit()
	{
		// Get parameters from parameter server
		// Get update rates
		float clustering_update_rate = RosUtils::getParameterOr<float>(node_, "target_clustering.clustering_update_rate", 1.0f);
		float analysis_update_rate = RosUtils::getParameterOr<float>(node_, "target_clustering.analysis_update_rate", 20.0f);
		// Get persistence weights
		float optimality_weight = RosUtils::getParameterOr<float>(node_, "target_clustering.persistence_weights.optimality", 1.0f);
		float cooldown_weight = RosUtils::getParameterOr<float>(node_, "target_clustering.persistence_weights.cooldown", 1.0f);
		float switch_weight = RosUtils::getParameterOr<float>(node_, "target_clustering.persistence_weights.switch", 2.0f);
		float bonding_weight = RosUtils::getParameterOr<float>(node_, "target_clustering.persistence_weights.bonding", 0.5f);
		// Get minimum and margin radii of the enclosing circle
		r_min_ = RosUtils::getParameterOr<float>(node_, "target_clustering.r_min", 0.3f);
		r_margin_ = RosUtils::getParameterOr<float>(node_, "target_clustering.r_margin", 1.0f);

		// Initialize target and cluster IDs
		target_ids_.clear();
		cluster_ids_.clear();

		// Initialize data
		state_ = State::Idle;
		points_ = std::unordered_map<ID, Vector3r>();
		has_points_ = std::unordered_map<ID, bool>();
		assignments_ = KmeansMod::Assignments();
		has_assignments_ = false;

		// Initialize subscribers and publishers
		info_subs_ = std::unordered_map<ID, core::SubscriberPtr<core::TargetInfoMsg>>();
		cluster_pubs_ = std::unordered_map<ID, core::PublisherPtr<core::ClusterInfoMsg>>();

		// Set K-Means parameters
		kmeans_ = std::make_shared<KmeansMod>();
		kmeans_->setPersistenceWeights(optimality_weight, cooldown_weight, switch_weight, bonding_weight);

		// Set update timers
		prev_time_ = RosUtils::getTimeNow(node_);
		clustering_timer_ = RosUtils::createTimerByRate(node_, clustering_update_rate,
			std::bind(&TargetClustering::updateClustering, this));
		analysis_timer_ = RosUtils::createTimerByRate(node_, analysis_update_rate,
			std::bind(&TargetClustering::updateAnalysis, this));
	}

	void TargetClustering::onShutdown()
	{
		std::lock_guard<std::mutex> lock(mutex_);
		// Set state to idle
		state_ = State::Idle;
		// Destroy identifier data
		target_ids_.clear();
		cluster_ids_.clear();
		// Destroy points and assignments
		points_.clear();
		has_points_.clear();
		assignments_.clear();
		has_assignments_ = false;
		// Destroy K-means
		kmeans_.reset();
		// Destroy subscribers and publishers
		info_subs_.clear();
		cluster_pubs_.clear();
		// Destroy update timers
		clustering_timer_.reset();
		analysis_timer_.reset();
	}

	// ════════════════════════════════════════════════════════════════════════════
	// SAFE CALLBACKS: Thread-safe callback functions
	// ════════════════════════════════════════════════════════════════════════════

	void TargetClustering::targetInfoCallback(const ID& target_id, const TargetInfoMsg::SharedPtr msg)
	{
		// Convert to Eigen
		std::lock_guard<std::mutex> lock(mutex_);
		const Vector3r position = MsgConversions::fromMsg(msg->position);
		points_[target_id] = position;
		has_points_[target_id] = true;
	}

	// ════════════════════════════════════════════════════════════════════════════
	// SAFE ADDERS/REMOVERS: Thread-safe adders/removers
	// ════════════════════════════════════════════════════════════════════════════

	void TargetClustering::addCluster(const ID& cluster_id)
	{
		std::lock_guard<std::mutex> lock(mutex_);
		// Add cluster to set
		cluster_ids_.insert(cluster_id);
		// Create publisher
		cluster_pubs_.insert({ cluster_id, topic_tools_->createClusterInfoPublisher(cluster_id) });
		// Reset K-means
		kmeans_->initialize(target_ids_, cluster_ids_);
		RCLCPP_WARN(node_->get_logger(), "Target clustering: Cluster added, re-initialized K-means with %d targets and %d clusters",
			static_cast<int>(target_ids_.size()), static_cast<int>(cluster_ids_.size()));
	}

	void TargetClustering::removeCluster(const ID& cluster_id)
	{
		std::lock_guard<std::mutex> lock(mutex_);
		// Remove cluster from set
		cluster_ids_.erase(cluster_id);
		// Destroy publisher
		cluster_pubs_.erase(cluster_id);
		// Reset K-means
		kmeans_->initialize(target_ids_, cluster_ids_);
		RCLCPP_WARN(node_->get_logger(), "Target clustering: Cluster removed, re-initialized K-means with %d targets and %d clusters",
			static_cast<int>(target_ids_.size()), static_cast<int>(cluster_ids_.size()));
	}

	void TargetClustering::addTarget(const ID& target_id)
	{
		std::lock_guard<std::mutex> lock(mutex_);
		// Add target to set
		target_ids_.insert(target_id);
		// Add point
		points_[target_id] = Vector3r::Zero();
		has_points_[target_id] = false;
		// Create subscriber
		info_subs_.insert({ target_id, topic_tools_->createTargetInfoSubscriber(target_id,
			[this, target_id](const TargetInfoMsg::SharedPtr msg)
			{
				this->targetInfoCallback(target_id, msg);
			}) });
		// Reset K-means
		kmeans_->initialize(target_ids_, cluster_ids_);
		RCLCPP_WARN(node_->get_logger(), "Target clustering: Target added, re-initialized K-means with %d targets and %d clusters",
			static_cast<int>(target_ids_.size()), static_cast<int>(cluster_ids_.size()));
	}

	void TargetClustering::removeTarget(const ID& target_id)
	{
		std::lock_guard<std::mutex> lock(mutex_);
		// Remove target from set
		target_ids_.erase(target_id);
		// Remove point
		points_.erase(target_id);
		has_points_.erase(target_id);
		// Destroy subscriber
		info_subs_.erase(target_id);
		// Reset K-means
		kmeans_->initialize(target_ids_, cluster_ids_);
		RCLCPP_WARN(node_->get_logger(), "Target clustering: Target removed, re-initialized K-means with %d targets and %d clusters",
			static_cast<int>(target_ids_.size()), static_cast<int>(cluster_ids_.size()));
	}

	// ════════════════════════════════════════════════════════════════════════════
	// UPDATE: Update clustering and cluster analysis
	// ════════════════════════════════════════════════════════════════════════════

	void TargetClustering::updateClustering()
	{
		std::lock_guard<std::mutex> lock(mutex_);

		// Get current time
		auto curr_time = RosUtils::getTimeNow(node_);
		float dt = (curr_time - prev_time_).seconds();
		prev_time_ = curr_time;

		// Check if we have data to process
		for (const auto& [target_id, point] : points_)
		{
			if (!has_points_[target_id])
			{
				RCLCPP_WARN(node_->get_logger(), "Target clustering: Target %s has no valid position", target_id.c_str());
				return;
			}
		}

		// Process based on current state
		switch (state_)
		{
		case State::Idle:
		{
			// Transition to Active state
			state_ = State::Active;
			RCLCPP_WARN(node_->get_logger(), "Target clustering: Targets and clusters available, transitioning to Active state");
		}
		break;

		case State::Active:
		{
			// Perform clustering
			if (!has_assignments_)
			{
				const auto& assignments = kmeans_->performClustering(points_, dt);
				RCLCPP_INFO(node_->get_logger(), "Target clustering: Clustering performed, assignments:");
				for (const auto& [target_id, cluster_id] : assignments)
				{
					RCLCPP_INFO(node_->get_logger(), "	-Target ID: %s, Cluster ID: %s", target_id.c_str(), cluster_id.c_str());
				}

				// Update assignments
				assignments_ = assignments;
				has_assignments_ = true;
			}
		}
		break;
		}
	}

	void TargetClustering::updateAnalysis()
	{
		std::lock_guard<std::mutex> lock(mutex_);

		// Check if assignments are valid
		if (!has_assignments_)
			return;

		// Cycle through each cluster
		std::vector<ID> cluster_ids;
		std::vector<PointMsg> cluster_centers;
		std::vector<float> cluster_radii;
		for (const auto& cluster_id : cluster_ids_)
		{
			// Get cluster points
			auto tab_P = ClusterAnalysis::getClusterPoints(cluster_id, assignments_, points_);
			if (tab_P.cols() <= 0)
				continue;

			// Get cluster enclosing circle
			ClusterAnalysis::Circle circle;
			try
			{
				circle = ClusterAnalysis::computeEnclosingCircle(tab_P, r_min_, r_margin_);
			}
			catch (const std::exception& e)
			{
				RCLCPP_ERROR(node_->get_logger(), "Target clustering: Error computing minimal enclosing circle: %s", e.what());
				continue;
			}

			// Create cluster info message
			ClusterInfoMsg cluster_info;
			cluster_info.header = RosUtils::createHeader(node_, transform_tools_->getGlobalFrame());
			cluster_info.center.x = circle.C.x;
			cluster_info.center.y = circle.C.y;
			cluster_info.center.z = 0.0f;
			cluster_info.radius = circle.R;

			// Publish cluster info
			cluster_pubs_[cluster_id]->publish(cluster_info);

			// Add cluster to vectors
			cluster_ids.push_back(cluster_id);
			cluster_centers.push_back(cluster_info.center);
			cluster_radii.push_back(cluster_info.radius);
		}

		// Update clusters in simulation
		framework_tools_->updateClusterGroup(cluster_ids, cluster_centers, cluster_radii);
	}

} // namespace flychams::perception