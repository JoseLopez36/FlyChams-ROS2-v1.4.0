#include "flychams_bringup/registration/gui_registration.hpp"

using namespace flychams::core;

namespace flychams::bringup
{
	// ════════════════════════════════════════════════════════════════════════════
	// CONSTRUCTOR: Constructor and destructor
	// ════════════════════════════════════════════════════════════════════════════

	void GuiRegistration::onInit()
	{
		// Get windows IDs from parameter server
		fixed_window_ids_ = RosUtils::getParameter<IDs>(node_, "fixed_window_ids");
		central_window_id_ = RosUtils::getParameter<std::string>(node_, "central_window_id");
		tracking_window_ids_ = RosUtils::getParameter<IDs>(node_, "tracking_window_ids");

		// Get window count
		int num_fixed_windows = static_cast<int>(fixed_window_ids_.size());
		int num_tracking_windows = static_cast<int>(tracking_window_ids_.size());
		int num_windows = num_fixed_windows + 1 + num_tracking_windows;

		// Set window parameters (empty for now)
		IDs window_ids(num_windows);
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

		// Add window IDs
		for (int i = 0; i < num_fixed_windows; i++)
			window_ids[i] = fixed_window_ids_[i];
		window_ids[num_fixed_windows] = central_window_id_;
		for (int i = 0; i < num_tracking_windows; i++)
			window_ids[num_fixed_windows + 1 + i] = tracking_window_ids_[i];

		// Send command to set the window images
		ext_tools_->setWindowImageGroup(window_ids, vehicle_ids, camera_ids, crop_x, crop_y, crop_w, crop_h);
	}

	void GuiRegistration::onShutdown()
	{
		// Destroy window IDs
		fixed_window_ids_.clear();
		central_window_id_.clear();
		tracking_window_ids_.clear();
	}

} // namespace flychams::bringup