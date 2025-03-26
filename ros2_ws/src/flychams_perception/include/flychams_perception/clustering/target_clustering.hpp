#pragma once

// Standard includes
#include <mutex>

// K-Means modified include
#include "flychams_perception/clustering/kmeans_mod.hpp"

// Cluster analysis include
#include "flychams_perception/clustering/cluster_analysis.hpp"

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
		TargetClustering(core::NodePtr node, core::ConfigTools::SharedPtr config_tools, core::FrameworkTools::SharedPtr framework_tools, core::TopicTools::SharedPtr topic_tools, core::TransformTools::SharedPtr transform_tools)
			: BaseModule(node, config_tools, framework_tools, topic_tools, transform_tools)
		{
			init();
		}

	protected: // Overrides
		void onInit() override;
		void onShutdown() override;

	public: // Types
		using SharedPtr = std::shared_ptr<TargetClustering>;
		enum class State
		{
			Idle,  		// Waiting for targets
			Active 	    // Performing clustering
		};

	private: // Parameters
		float r_min_;
		float r_margin_;

	private: // Data
		// Clustering state
		State state_;
		// Target data
		core::UnorderedIDs target_ids_;
		std::unordered_map<core::ID, core::Vector3r> points_;
		std::unordered_map<core::ID, bool> has_points_;
		// Cluster data
		core::UnorderedIDs cluster_ids_;
		// Target assignments (target ID -> cluster ID)
		KmeansMod::Assignments assignments_;
		bool has_assignments_;
		// Thread-safety
		std::mutex mutex_;
		// Time data
		core::Time prev_time_;
		// K-Means clustering
		KmeansMod::SharedPtr kmeans_;

	private: // Safe callbacks
		void targetInfoCallback(const core::ID& target_id, const core::TargetInfoMsg::SharedPtr msg);

	public: // Safe adders/removers
		void addCluster(const core::ID& cluster_id);
		void addTarget(const core::ID& target_id);
		void removeCluster(const core::ID& cluster_id);
		void removeTarget(const core::ID& target_id);

	private: // Implementation
		// Update
		void updateClustering();
		void updateAnalysis();

	private:
		// Subscribers
		std::unordered_map<core::ID, core::SubscriberPtr<core::TargetInfoMsg>> info_subs_;
		// Publishers
		std::unordered_map<core::ID, core::PublisherPtr<core::ClusterInfoMsg>> cluster_pubs_;
		// Timers
		core::TimerPtr clustering_timer_;
		core::TimerPtr analysis_timer_;
	};

} // namespace flychams::perception