#pragma once

// Standard includes
#include <mutex>

// Trajectory includes
#include "flychams_targets/target_trajectory/trajectory_parser.hpp"

// Base module include
#include "flychams_core/base/base_module.hpp"

namespace flychams::targets
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Control for target trajectories and data publishing
     *
     * @details
     * This class is responsible for controlling the position of targets.
     * It also handles the data publishing of target information.
     *
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-02-26
     * ════════════════════════════════════════════════════════════════
     */
    class TargetController : public core::BaseModule
    {
    public: // Constructor/Destructor
        TargetController(const core::ID& target_id, core::NodePtr node, core::ConfigTools::SharedPtr config_tools, core::ExternalTools::SharedPtr ext_tools, core::TopicTools::SharedPtr topic_tools, core::TfTools::SharedPtr tf_tools)
            : BaseModule(node, config_tools, ext_tools, topic_tools, tf_tools), target_id_(target_id)
        {
            init();
        }

    protected: // Overrides
        void onInit() override;
        void onShutdown() override;

    public: // Types
        using SharedPtr = std::shared_ptr<TargetController>;

    private: // Parameters
        core::ID target_id_;

    private: // Data
        // Trajectory data
        std::vector<TrajectoryParser::Point> trajectory_;
        int current_idx_;
        int num_points_;
        bool reverse_;
        float time_elapsed_;
        // Target info
        core::TargetInfoMsg info_;

    public: // Public methods
        // Update
        void update(const float& dt);
        core::PointMsg getPosition() const;

    private: // Methods
        void updateInfo(const float& dt, core::PointMsg& position);
        void publishInfo(const core::TargetInfoMsg& info);

    private:
        // Publishers
        core::PublisherPtr<core::TargetInfoMsg> info_pub_;
    };

} // namespace flychams::targets