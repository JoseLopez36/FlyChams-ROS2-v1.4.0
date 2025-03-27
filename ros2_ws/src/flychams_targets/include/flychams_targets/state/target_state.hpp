#pragma once

// Trajectory includes
#include "flychams_targets/control/trajectory_parser.hpp"

// Base module include
#include "flychams_core/base/base_module.hpp"

namespace flychams::targets
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief State manager for targets
     *
     * @details
     * This class is responsible for managing the state of targets.
     * It handles the state of the target, mainly its instantaneus
     * position.
     *
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-03-27
     * ════════════════════════════════════════════════════════════════
     */
    class TargetState : public core::BaseModule
    {
    public: // Constructor/Destructor
        TargetState(const core::ID& target_id, core::NodePtr node, core::ConfigTools::SharedPtr config_tools, core::FrameworkTools::SharedPtr framework_tools, core::TopicTools::SharedPtr topic_tools, core::TransformTools::SharedPtr transform_tools)
            : BaseModule(node, config_tools, framework_tools, topic_tools, transform_tools), target_id_(target_id)
        {
            init();
        }

    protected: // Overrides
        void onInit() override;
        void onShutdown() override;

    public: // Types
        using SharedPtr = std::shared_ptr<TargetState>;

    private: // Parameters
        core::ID target_id_;
        float update_rate_;
        float cmd_timeout_;

    private: // Data
        // Trajectory
        std::vector<TrajectoryParser::Point> trajectory_;
        int current_idx_;
        int num_points_;
        bool reverse_;
        float time_elapsed_;
        // Message
        core::PointStampedMsg position_msg_;
        // Time step
        core::Time last_update_time_;

    private: // State management
        void update();

    private:
        // Publisher
        core::PublisherPtr<core::PointStampedMsg> position_pub_;
        // Timer
        core::TimerPtr update_timer_;
    };

} // namespace flychams::targets