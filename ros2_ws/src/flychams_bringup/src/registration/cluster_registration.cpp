#include "flychams_bringup/registration/cluster_registration.hpp"

using namespace flychams::core;

namespace flychams::bringup
{
	// ════════════════════════════════════════════════════════════════════════════
	// CONSTRUCTOR: Constructor and destructor
	// ════════════════════════════════════════════════════════════════════════════

	void ClusterRegistration::onInit()
	{
		// Iterate over all agents in the configuration
		clusters_.clear();
		int cluster_index = 0;
		for (const auto& [agent_id, agent_ptr] : config_tools_->getAgentTeam())
		{
			// Register clusters based on the tracking mode
			const auto& tracking_params = config_tools_->getTrackingParameters(agent_id);
			switch (tracking_params.mode)
			{
				case TrackingMode::MultiCamera:
					// Iterate over all heads and register clusters for tracking heads
					for (const auto& head : tracking_params.head_params)
					{
						if (head.role == TrackingRole::Tracking)
						{
							// Generate cluster ID
							std::stringstream ss;
							ss << "CLUSTER" << std::setw(2) << std::setfill('0') << cluster_index;
							const ID cluster_id = ss.str();

							// Add clusters to list
							clusters_.push_back(cluster_id);

							// Increment cluster index
							cluster_index++;
						}
					}
					break;

				case TrackingMode::MultiWindow:
					// Iterate over all windows and register clusters for tracking windows
					for (const auto& window : tracking_params.window_params)
					{
						if (window.role == TrackingRole::Tracking)
						{
							// Generate cluster ID
							std::stringstream ss;
							ss << "CLUSTER" << std::setw(2) << std::setfill('0') << cluster_index;
							const ID cluster_id = ss.str();

							// Add clusters to list
							clusters_.push_back(cluster_id);

							// Increment cluster index
							cluster_index++;
						}
					}
					break;

				default:
					RCLCPP_ERROR(node_->get_logger(), "Cluster registration: Invalid tracking mode");
					break;
			}
		}
	}

	void ClusterRegistration::onShutdown()
	{
		// Destroy clusters
		clusters_.clear();
	}

} // namespace flychams::bringup