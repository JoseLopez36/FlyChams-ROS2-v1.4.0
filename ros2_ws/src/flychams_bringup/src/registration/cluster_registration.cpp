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
		for (const auto& agent : config_tools_->getAgents())
		{
			// Extract maximum number of assignments per agent
			const int max_assign = agent.second->max_assignments;

			// Iterate over all assignments and register clusters
			for (int i = 0; i < max_assign; i++)
			{
				// Generate cluster ID
				std::stringstream ss;
				ss << "CLUSTER" << std::setw(2) << std::setfill('0') << i;
				const ID cluster_id = ss.str();

				// Add clusters to list
				clusters_.push_back(cluster_id);
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
		bool draw_world_markers = config_tools_->getSimulation()->draw_world_markers;
		std::vector<ColorMsg> highlight_colors;
		int i = 0;
		for (const auto& cluster_id : clusters_)
		{
			// Position
			PointMsg center;
			center.x = -500.0f - 10.0f * i; // Position away from origin
			center.y = -500.0f - 10.0f * i;
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

			i++;
		}

		// Add targets to simulation
		if (draw_world_markers)
			ext_tools_->addClusterGroup(clusters_, centers, radii, true, highlight_colors);
	}

} // namespace flychams::bringup