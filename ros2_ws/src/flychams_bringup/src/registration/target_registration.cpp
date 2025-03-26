#include "flychams_bringup/registration/target_registration.hpp"

using namespace flychams::core;

namespace flychams::bringup
{
	// ════════════════════════════════════════════════════════════════════════════
	// CONSTRUCTOR: Constructor and destructor
	// ════════════════════════════════════════════════════════════════════════════

	void TargetRegistration::onInit()
	{
		// Iterate over all targets in the configuration
		targets_.clear();
		for (const auto& [target_id, target_ptr] : config_tools_->getTargetGroup())
		{
			// Add target to list
			targets_.push_back(target_id);
		}
	}

	void TargetRegistration::onShutdown()
	{
		// Destroy targets
		targets_.clear();
	}

	// ════════════════════════════════════════════════════════════════════════════
	// METHODS
	// ════════════════════════════════════════════════════════════════════════════

	void TargetRegistration::spawnTargets()
	{
		// Get spawn parameters
		std::vector<PointMsg> positions;
		std::vector<TargetType> types;
		std::vector<ColorMsg> highlight_colors;
		int target_index = 0;
		for (const auto& target_id : targets_)
		{
			// Position
			PointMsg position;
			position.x = 500.0f + 10.0f * target_index; // Position away from origin
			position.y = 500.0f + 10.0f * target_index;
			position.z = 10.0f;
			positions.push_back(position);

			// Type
			types.push_back(config_tools_->getTarget(target_id)->type);

			// Highlight color
			ColorMsg highlight_color;
			highlight_color.r = 1.0f;
			highlight_color.g = 0.0f;
			highlight_color.b = 0.0f;
			highlight_color.a = 0.015f;
			highlight_colors.push_back(highlight_color);

			// Increment target index
			target_index++;
		}

		// Add targets to simulation
		framework_tools_->addTargetGroup(targets_, types, positions, true, highlight_colors);
	}

} // namespace flychams::bringup