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
		ClusterAnalysis(core::NodePtr node, core::ConfigTools::SharedPtr config_tools, core::FrameworkTools::SharedPtr framework_tools, core::TopicTools::SharedPtr topic_tools, core::TransformTools::SharedPtr transform_tools, core::CallbackGroupPtr module_cb_group)
			: BaseModule(node, config_tools, framework_tools, topic_tools, transform_tools, module_cb_group)
		{
			init();
		}

	protected: // Overrides
		void onInit() override;
		void onShutdown() override;

	public: // Types
		using SharedPtr = std::shared_ptr<ClusterAnalysis>;
		struct Cluster
		{
			// Assignment data
			std::vector<core::ID> assignment;
			bool has_assignment;
			// Subscriber
			core::SubscriberPtr<core::ClusterAssignmentMsg> assignment_sub;
			// Publisher
			core::PublisherPtr<core::ClusterGeometryMsg> geometry_pub;
			// Constructor
			Cluster()
				: assignment(), has_assignment(false), assignment_sub(), geometry_pub()
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
		// Enclosing circle parameters
		float min_circle_radius_;
		float margin_circle_radius_;

	private: // Data
		// Clusters
		std::unordered_map<core::ID, Cluster> clusters_;
		// Targets
		std::unordered_map<core::ID, Target> targets_;

	public: // Public methods
		void addCluster(const core::ID& cluster_id);
		void addTarget(const core::ID& target_id);
		void removeCluster(const core::ID& cluster_id);
		void removeTarget(const core::ID& target_id);

	private: // Callbacks
		void clusterAssignmentCallback(const core::ID& cluster_id, const core::ClusterAssignmentMsg::SharedPtr msg);
		void targetPositionCallback(const core::ID& target_id, const core::PointStampedMsg::SharedPtr msg);

	private: // Analysis management
		void update();

	private: // Analysis methods
		std::pair<core::Vector2r, float> calculateEnclosingCircle(const core::Matrix3Xr& tab_P, const float& min_radius, const float& margin_radius);

	private: // ROS components
		// Timer
		core::TimerPtr update_timer_;
	};

} // namespace flychams::perception