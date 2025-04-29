#pragma once

// Base module include
#include "flychams_core/base/base_module.hpp"

namespace flychams::bringup
{
	/**
	 * ════════════════════════════════════════════════════════════════
	 * @brief Registration of clusters
	 * ════════════════════════════════════════════════════════════════
	 * @author Jose Francisco Lopez Ruiz
	 * @date 2025-03-21
	 * ════════════════════════════════════════════════════════════════
	 */
	class ClusterRegistration : public core::BaseModule
	{
	public: // Constructor/Destructor
		ClusterRegistration(core::NodePtr node, core::ConfigTools::SharedPtr config_tools, core::FrameworkTools::SharedPtr framework_tools, core::TopicTools::SharedPtr topic_tools, core::TransformTools::SharedPtr transform_tools, core::CallbackGroupPtr module_cb_group)
			: BaseModule(node, config_tools, framework_tools, topic_tools, transform_tools, module_cb_group)
		{
			init();
		}

	protected: // Overrides
		void onInit() override;
		void onShutdown() override;

	public: // Types
		using SharedPtr = std::shared_ptr<ClusterRegistration>;

	private: // Data
		core::IDs clusters_;

	public: // Methods
		const core::IDs& getClusters() const { return clusters_; }
	};

} // namespace flychams::bringup