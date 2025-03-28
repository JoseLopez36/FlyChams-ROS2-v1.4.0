#pragma once

// Standard includes
#include <mutex>

// Base module include
#include "flychams_core/base/base_module.hpp"

namespace flychams::control
{
	/**
	 * ════════════════════════════════════════════════════════════════
	 * @brief Controller for UAV heads
	 *
	 * @details
	 * This class is responsible for controlling the position and yaw
	 * of UAV heads using PID controllers. It manages multiple heads
	 * and their respective control parameters.
	 *
	 * ════════════════════════════════════════════════════════════════
	 * @author Jose Francisco Lopez Ruiz
	 * @date 2025-03-04
	 * ════════════════════════════════════════════════════════════════
	 */
	class HeadControl : public core::BaseModule
	{
	public: // Constructor/Destructor
		HeadControl(const core::ID& agent_id, core::NodePtr node, core::ConfigTools::SharedPtr config_tools, core::FrameworkTools::SharedPtr framework_tools, core::TopicTools::SharedPtr topic_tools, core::TransformTools::SharedPtr transform_tools, core::CallbackGroupPtr module_cb_group)
			: BaseModule(node, config_tools, framework_tools, topic_tools, transform_tools, module_cb_group), agent_id_(agent_id)
		{
			init();
		}

	protected: // Overrides
		void onInit() override;
		void onShutdown() override;

	public: // Types
		using SharedPtr = std::shared_ptr<HeadControl>;
		struct HeadCmd
		{
			core::ID id;
			core::QuaternionMsg ori;
			float fov;
		};

	private: // Parameters
		core::ID agent_id_;
		float update_rate_;

	private: // Data
		// Current status
		core::AgentStatus curr_status_;
		bool has_status_;
		// Head setpoints
		core::AgentHeadSetpointsMsg head_setpoints_;
		bool has_head_setpoints_;
		// Central head command
		HeadCmd central_cmd_;

	private: // Callbacks
		void statusCallback(const core::AgentStatusMsg::SharedPtr msg);
		void headSetpointsCallback(const core::AgentHeadSetpointsMsg::SharedPtr msg);

	private: // Head management
		// Update
		void update();

	private:
		// Subscribers
		core::SubscriberPtr<core::AgentStatusMsg> status_sub_;
		core::SubscriberPtr<core::AgentHeadSetpointsMsg> head_setpoints_sub_;
		// Timer
		core::TimerPtr update_timer_;
	};

} // namespace flychams::control