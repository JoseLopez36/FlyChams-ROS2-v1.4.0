#include "flychams_bringup/registration/gui_registration.hpp"

using namespace flychams::core;

namespace flychams::bringup
{
	// ════════════════════════════════════════════════════════════════════════════
	// CONSTRUCTOR: Constructor and destructor
	// ════════════════════════════════════════════════════════════════════════════

	void GuiRegistration::onInit()
	{
		// Get window IDs
		windows_.clear();
		windows_.push_back(config_tools_->getSystem().scenario_view_id);
		windows_.push_back(config_tools_->getSystem().agent_view_id);
		windows_.push_back(config_tools_->getSystem().payload_view_id);
		windows_.push_back(config_tools_->getSystem().map_view_id);
		windows_.push_back(config_tools_->getSystem().central_view_id);
		for (const auto& tracking_id : config_tools_->getSystem().tracking_view_ids)
			windows_.push_back(tracking_id);

		// Get window count
		const int num_windows = static_cast<int>(windows_.size());

		// Set window parameters (empty for now)
		IDs vehicle_ids(num_windows);
		IDs camera_ids(num_windows);
		std::vector<int> crop_x(num_windows);
		std::vector<int> crop_y(num_windows);
		std::vector<int> crop_w(num_windows);
		std::vector<int> crop_h(num_windows);
		for (int i = 0; i < num_windows; i++)
		{
			vehicle_ids[i] = "";
			camera_ids[i] = "";
			crop_x[i] = 0;
			crop_y[i] = 0;
			crop_w[i] = 0;
			crop_h[i] = 0;
		}

		// Send command to set the window images
		framework_tools_->setWindowImageGroup(windows_, vehicle_ids, camera_ids, crop_x, crop_y, crop_w, crop_h);
	}

	void GuiRegistration::onShutdown()
	{
		// Destroy window IDs
		windows_.clear();
	}

} // namespace flychams::bringup