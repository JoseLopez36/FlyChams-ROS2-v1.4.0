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
		for (const auto& [target_id, target_config] : config_tools_->getTargets())
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
		RegionType region_type = config_tools_->getMap()->region_type;
		bool draw_world_markers = config_tools_->getSimulation()->draw_world_markers;
		int i = 0;
		for (const auto& target_id : targets_)
		{
			// Position
			PointMsg position;
			position.x = 500.0f + 10.0f * i; // Position away from origin
			position.y = 500.0f + 10.0f * i;
			position.z = 10.0f;
			positions.push_back(position);

			// Type
			types.push_back(config_tools_->getTarget(target_id)->target_type);

			// Highlight color
			ColorMsg highlight_color;
			highlight_color.r = 1.0f;
			highlight_color.g = 0.0f;
			highlight_color.b = 0.0f;
			highlight_color.a = 0.015f;
			highlight_colors.push_back(highlight_color);

			i++;
		}

		// Add targets to simulation
		ext_tools_->addTargetGroup(targets_, types, positions, draw_world_markers, highlight_colors, region_type);
	}

} // namespace flychams::bringup