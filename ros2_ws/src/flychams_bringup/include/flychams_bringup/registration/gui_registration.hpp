#pragma once

// Base module include
#include "flychams_core/base/base_module.hpp"

namespace flychams::bringup
{
	/**
	 * ════════════════════════════════════════════════════════════════
	 * @brief Registration of GUI windows
	 *
	 * @details
	 * This class is responsible for registering GUI windows.
	 *
	 * ════════════════════════════════════════════════════════════════
	 * @author Jose Francisco Lopez Ruiz
	 * @date 2025-03-21
	 * ════════════════════════════════════════════════════════════════
	 */
	class GuiRegistration : public core::BaseModule
	{
	public: // Constructor/Destructor
		GuiRegistration(core::NodePtr node, core::ConfigTools::SharedPtr config_tools, core::ExternalTools::SharedPtr ext_tools, core::TopicTools::SharedPtr topic_tools, core::TfTools::SharedPtr tf_tools)
			: BaseModule(node, config_tools, ext_tools, topic_tools, tf_tools)
		{
			init();
		}

	protected: // Overrides
		void onInit() override;
		void onShutdown() override;

	public: // Types
		using SharedPtr = std::shared_ptr<GuiRegistration>;

	private: // Data
		core::IDs fixed_window_ids_;
		core::ID central_window_id_;
		core::IDs tracking_window_ids_;

	public: // Methods
		const core::IDs& getFixedWindows() const { return fixed_window_ids_; }
		const core::ID& getCentralWindow() const { return central_window_id_; }
		const core::IDs& getTrackingWindows() const { return tracking_window_ids_; }
	};

} // namespace flychams::bringup