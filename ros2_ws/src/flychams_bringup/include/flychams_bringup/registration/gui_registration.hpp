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
		GuiRegistration(core::NodePtr node, core::ConfigTools::SharedPtr config_tools, core::FrameworkTools::SharedPtr framework_tools, core::TopicTools::SharedPtr topic_tools, core::TransformTools::SharedPtr transform_tools)
			: BaseModule(node, config_tools, framework_tools, topic_tools, transform_tools)
		{
			init();
		}

	protected: // Overrides
		void onInit() override;
		void onShutdown() override;

	public: // Types
		using SharedPtr = std::shared_ptr<GuiRegistration>;

	private: // Data
		core::IDs windows_;

	public: // Methods
		const core::IDs& getWindows() const { return windows_; }
	};

} // namespace flychams::bringup