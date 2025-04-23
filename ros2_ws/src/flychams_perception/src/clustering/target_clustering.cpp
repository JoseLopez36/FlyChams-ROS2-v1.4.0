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
		clusters_.clear();
		C_.clear();
		targets_.clear();
		T_.clear();
		assignments_prev_.resize(0);
		is_first_run_ = true;

        // Create and initialize K-Means solver
        k_means_solver_ = std::make_shared<KMeansMod>();
        KMeansMod::Parameters solver_params;
        solver_params.ini_bonding_coef = ini_bonding_coef;
        solver_params.max_bonding_coef = max_bonding_coef;
        solver_params.bonding_coef_time_to_max = bonding_coef_time_to_max;
        solver_params.max_hysteresis_ratio = max_hysteresis_ratio;
        solver_params.min_hysteresis_ratio = min_hysteresis_ratio;
        k_means_solver_->init(solver_params);

		// Set update timer
		last_update_time_ = RosUtils::now(node_);
		update_timer_ = RosUtils::createTimer(node_, update_rate_,
			std::bind(&TargetClustering::update, this), module_cb_group_);
	}

	void TargetClustering::onShutdown()
	{
		// Destroy K-Means solver
		k_means_solver_->destroy();
		// Destroy clusters and targets
		clusters_.clear();
		C_.clear();
		targets_.clear();
		T_.clear();
		// Destroy update timer
		update_timer_.reset();
	}

	// ════════════════════════════════════════════════════════════════════════════
	// PUBLIC METHODS: Public methods for adding/removing clusters and targets
	// ════════════════════════════════════════════════════════════════════════════

	void TargetClustering::addCluster(const ID& cluster_id)
	{
        // Create and add cluster
        clusters_.insert({ cluster_id, Cluster() });
        C_.insert(cluster_id); // Add cluster to ordered set

        // Create cluster assignment publisher
        clusters_[cluster_id].assignment_pub = topic_tools_->createClusterAssignmentPublisher(cluster_id);
	}

	void TargetClustering::removeCluster(const ID& cluster_id)
	{
		// Remove cluster from map
        clusters_.erase(cluster_id);
        C_.erase(cluster_id); // Remove cluster from ordered set
	}

	void TargetClustering::addTarget(const ID& target_id)
	{
		// Create and add target
		targets_.insert({ target_id, Target() });
		T_.insert(target_id); // Add target to ordered set

		// Add target to previous assignments
        assignments_prev_.resize(assignments_prev_.size() + 1);
        assignments_prev_.setConstant(-1);

		// Create target true position subscriber
		targets_[target_id].position_sub = topic_tools_->createTargetTruePositionSubscriber(target_id,
			[this, target_id](const PointStampedMsg::SharedPtr msg)
			{
				this->targetPositionCallback(target_id, msg);
			}, sub_options_with_module_cb_group_);
	}

	void TargetClustering::removeTarget(const ID& target_id)
	{
		// Remove target from map
        targets_.erase(target_id);
        T_.erase(target_id); // Remove target from ordered set
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
		// Check if there are any clusters and targets
		if (C_.empty() || T_.empty())
		{
			RCLCPP_WARN(node_->get_logger(), "Target clustering: No clusters or targets available");
			return;
		}

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

		// Get vectors of target data with ordered data
        // It is important that the data is ordered according to the ordered set T_,
        // since the K-Means solver assumes that the data follows the same order always.
        // Targets
        int n = static_cast<int>(T_.size());
        Matrix3Xr tab_P(3, n);
        int i = 0;
        for (const auto& target_id : T_)
        {
            const auto& target = targets_[target_id];
            tab_P.col(i) = RosUtils::fromMsg(target.position);
            i++;
        }

		// Get number of clusters
		int K = static_cast<int>(C_.size());

		// Get clustering mode
		KMeansMod::Mode mode = KMeansMod::Mode::CONSISTENT_AND_PERSISTENT;
		if (is_first_run_)
		{
			mode = KMeansMod::Mode::INITIAL;
			is_first_run_ = false;
		}

		// Perform clustering with available points
		RCLCPP_INFO(node_->get_logger(), "Target clustering: Performing clustering...");
		const auto& assignments = k_means_solver_->run(K, tab_P, assignments_prev_, mode, dt);
		RCLCPP_INFO(node_->get_logger(), "Target clustering: Clustering completed: ");
		for (int i = 0; i < n; i++)
		{
			RCLCPP_INFO(node_->get_logger(), "Target clustering:     %d", assignments(i));
		}

		// Update previous assignments
		assignments_prev_ = assignments;

        // Create and publish an assignment message for each cluster
        int k = 0;
        for (const auto& cluster_id : C_)
        {
            // Create message
            ClusterAssignmentMsg msg;
            msg.header.stamp = node_->get_clock()->now();

            // Get assignment
            for (int i = 0; i < n; i++)
            {
                const int& c = assignments(i);
                if (c == k)
                {
					const std::string target_id = *std::next(T_.begin(), i);
                    msg.target_ids.push_back(target_id);
                }
            }

            // Publish
            clusters_[cluster_id].assignment_pub->publish(msg);

            k++;
        }
	}

} // namespace flychams::perception