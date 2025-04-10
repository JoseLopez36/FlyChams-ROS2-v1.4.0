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
			// Get assignment count per agent
			const int assignment_count = config_tools_->getTrackingParameters(agent_id).n;

			// Iterate over all assignments and register clusters
			for (int i = 0; i < assignment_count; i++)
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

} // namespace flychams::bringup