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
			// Get maximum number of assignments per agent
			const int max_assign = config_tools_->getMaxAssignments(agent_id);

			// Iterate over all assignments and register clusters
			for (int i = 0; i < max_assign; i++)
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
	}

	void ClusterRegistration::onShutdown()
	{
		// Destroy clusters
		clusters_.clear();
	}

	// ════════════════════════════════════════════════════════════════════════════
	// METHODS
	// ════════════════════════════════════════════════════════════════════════════

	void ClusterRegistration::spawnClusters()
	{
		// Get spawn parameters
		std::vector<PointMsg> centers;
		std::vector<float> radii;
		std::vector<ColorMsg> highlight_colors;
		int cluster_index = 0;
		for (const auto& cluster_id : clusters_)
		{
			// Position
			PointMsg center;
			center.x = -500.0f - 10.0f * cluster_index; // Position away from origin
			center.y = -500.0f - 10.0f * cluster_index;
			center.z = 10.0f;
			centers.push_back(center);

			// Radius
			radii.push_back(1.0f);

			// Highlight color
			ColorMsg highlight_color;
			highlight_color.r = 0.0f;
			highlight_color.g = 1.0f;
			highlight_color.b = 1.0f;
			highlight_color.a = 0.005f;
			highlight_colors.push_back(highlight_color);

			// Increment cluster index
			cluster_index++;
		}

		// Add targets to simulation
		framework_tools_->addClusterGroup(clusters_, centers, radii, true, highlight_colors);
	}

} // namespace flychams::bringup