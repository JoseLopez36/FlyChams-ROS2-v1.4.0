#pragma once

// Perception includes
#include "flychams_perception/analysis/welzls_circle.hpp"

// Base module include
#include "flychams_core/base/base_module.hpp"

namespace flychams::perception
{
	/**
	 * ════════════════════════════════════════════════════════════════
	 * @brief Cluster analysis implementation
	 *
	 * @details
	 * This class implements a cluster analysis.
	 * It provides methods for calculating cluster data (such as
	 * centroid or enclosing circle).
	 *
	 * ════════════════════════════════════════════════════════════════
	 * @author Jose Francisco Lopez Ruiz
	 * @date 2025-02-26
	 * ════════════════════════════════════════════════════════════════
	 */
	class ClusterAnalysis : public core::BaseModule
	{
	public: // Constructor/Destructor
		ClusterAnalysis(core::NodePtr node, core::ConfigTools::SharedPtr config_tools, core::FrameworkTools::SharedPtr framework_tools, core::TopicTools::SharedPtr topic_tools, core::TransformTools::SharedPtr transform_tools)
			: BaseModule(node, config_tools, framework_tools, topic_tools, transform_tools)
		{
			init();
		}

	protected: // Overrides
		void onInit() override;
		void onShutdown() override;

	public: // Types
		using SharedPtr = std::shared_ptr<ClusterAnalysis>;
		struct Target
		{
			// Assignment data
			core::ID assigned_id;
			bool has_assignment;
			// Position data
			core::PointMsg position;
			bool has_position;
			// Subscribers
			core::SubscriberPtr<core::StringMsg> assignment_sub;
			core::SubscriberPtr<core::PointStampedMsg> position_sub;
			// Constructor
			Target()
				: assigned_id(), has_assignment(false), position(), has_position(false), assignment_sub(), position_sub()
			{
			}
		};
		struct Cluster
		{
			// Geometry data
			core::ClusterGeometryMsg geometry;
			// Publisher
			core::PublisherPtr<core::ClusterGeometryMsg> geometry_pub;
			// Constructor
			Cluster()
				: geometry(), geometry_pub()
			{
			}
		};

	private: // Parameters
		float update_rate_;
		float min_circle_radius_;
		float margin_circle_radius_;

	private: // Data
		// Targets
		std::unordered_map<core::ID, Target> targets_;
		// Clusters
		std::unordered_map<core::ID, Cluster> clusters_;

	public: // Public methods
		void addCluster(const core::ID& cluster_id);
		void addTarget(const core::ID& target_id);
		void removeCluster(const core::ID& cluster_id);
		void removeTarget(const core::ID& target_id);

	private: // Callbacks
		void targetAssignmentCallback(const core::ID& target_id, const core::StringMsg::SharedPtr msg);
		void targetPositionCallback(const core::ID& target_id, const core::PointStampedMsg::SharedPtr msg);

	private: // Analysis management
		void update();

	private: // Analysis methods
		std::vector<core::PointMsg> assignCluster(const core::ID& cluster_id, const std::unordered_map<core::ID, Target>& targets);
		std::pair<core::Vector2r, float> calculateEnclosingCircle(const std::vector<core::PointMsg>& points, const float& min_radius, const float& margin_radius);

	private: // ROS components
		// Callback group
		core::CallbackGroupPtr callback_group_;
		// Timer
		core::TimerPtr update_timer_;
	};

} // namespace flychams::perception