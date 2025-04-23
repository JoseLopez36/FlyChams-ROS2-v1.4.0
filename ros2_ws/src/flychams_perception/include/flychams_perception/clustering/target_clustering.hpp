#pragma once

// K-Means modified include
#include "flychams_perception/clustering/k_means_mod.hpp"

// Base module include
#include "flychams_core/base/base_module.hpp"

namespace flychams::perception
{
	/**
	 * ════════════════════════════════════════════════════════════════
	 * @brief Clustering of targets
	 *
	 * @details
	 * This class is responsible for clustering targets using K-Means
	 * modified algorithm. It also performs cluster analysis to determine
	 * the minimal enclosing circle and other characteristics.
	 *
	 * ════════════════════════════════════════════════════════════════
	 * @author Jose Francisco Lopez Ruiz
	 * @date 2025-02-26
	 * ════════════════════════════════════════════════════════════════
	 */
	class TargetClustering : public core::BaseModule
	{
	public: // Constructor/Destructor
		TargetClustering(core::NodePtr node, core::ConfigTools::SharedPtr config_tools, core::FrameworkTools::SharedPtr framework_tools, core::TopicTools::SharedPtr topic_tools, core::TransformTools::SharedPtr transform_tools, core::CallbackGroupPtr module_cb_group)
			: BaseModule(node, config_tools, framework_tools, topic_tools, transform_tools, module_cb_group)
		{
			init();
		}

	protected: // Overrides
		void onInit() override;
		void onShutdown() override;

	public: // Types
		using SharedPtr = std::shared_ptr<TargetClustering>;
		struct Cluster
		{
			// Publisher
			core::PublisherPtr<core::ClusterAssignmentMsg> assignment_pub;
			// Constructor
			Cluster()
				: assignment_pub()
			{
			}
		};
		struct Target
		{
			// Position data
			core::PointMsg position;
			bool has_position;
			// Subscriber
			core::SubscriberPtr<core::PointStampedMsg> position_sub;
			// Constructor
			Target()
				: position(), has_position(false), position_sub()
			{
			}
		};

	private: // Parameters
		float update_rate_;
		// Command timeout
		float cmd_timeout_;

	private: // Data
		// Clusters
		std::unordered_map<core::ID, Cluster> clusters_;
		std::set<core::ID> C_;
		// Targets
		std::unordered_map<core::ID, Target> targets_;
		std::set<core::ID> T_;
		// K-Means clustering data
		core::RowVectorXi assignments_prev_;
		bool is_first_run_;
		// K-Means clustering solver
		KMeansMod::SharedPtr k_means_solver_;
		// Time step
		core::Time last_update_time_;

	public: // Public methods
		void addCluster(const core::ID& cluster_id);
		void addTarget(const core::ID& target_id);
		void removeCluster(const core::ID& cluster_id);
		void removeTarget(const core::ID& target_id);

	private: // Callbacks
		void targetPositionCallback(const core::ID& target_id, const core::PointStampedMsg::SharedPtr msg);

	private: // Clustering management
		void update();

	private: // ROS components
		// Timer
		core::TimerPtr update_timer_;
	};

} // namespace flychams::perception