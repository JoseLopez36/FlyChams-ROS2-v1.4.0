#include "flychams_bringup/registration/agent_registration.hpp"

using namespace flychams::core;

namespace flychams::bringup
{
	// ════════════════════════════════════════════════════════════════════════════
	// CONSTRUCTOR: Constructor and destructor
	// ════════════════════════════════════════════════════════════════════════════

	void AgentRegistration::onInit()
	{
		// Iterate over all agents in the configuration
		agents_.clear();
		for (const auto& [agent_id, agent_ptr] : config_tools_->getAgentTeam())
		{
			// Add agent to list
			agents_.push_back(agent_id);
		}
	}

	void AgentRegistration::onShutdown()
	{
		// Destroy agents
		agents_.clear();
	}

} // namespace flychams::bringup