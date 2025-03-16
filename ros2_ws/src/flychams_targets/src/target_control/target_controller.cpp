#include "flychams_targets/target_control/target_controller.hpp"

using namespace std::chrono_literals;
using namespace flychams::core;

namespace flychams::targets
{
    // ════════════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR: Constructor and destructor
    // ════════════════════════════════════════════════════════════════════════════

    void TargetController::onInit()
    {
        // Get parameters from parameter server
        // Get update rate
        float update_rate = RosUtils::getParameterOr<float>(node_, "target_registration.target_update_rate", 20.0f);

        // Parse trajectory
        const auto& path = config_tools_->getTarget(target_id_)->trajectory_path;
        RCLCPP_INFO(node_->get_logger(), "Parsing trajectory for target %s with path %s", target_id_.c_str(), path.c_str());
        trajectory_ = TrajectoryParser::parse(path);

        // Initialize trajectory data
        current_idx_ = 0;
        time_elapsed_ = 0.0f;
        num_points_ = static_cast<int>(trajectory_.size());
        reverse_ = false;

        // Initialize target info
        info_.header = RosUtils::createHeader(node_, tf_tools_->getGlobalFrame());
        info_.position.x = trajectory_[0].x;
        info_.position.y = trajectory_[0].y;
        info_.position.z = trajectory_[0].z;

        // Initialize target info publisher
        info_pub_ = topic_tools_->createTargetInfoPublisher(target_id_);
    }

    void TargetController::onShutdown()
    {
        // Destroy trajectory
        trajectory_.clear();
        // Destroy publisher
        info_pub_.reset();
    }

    // ════════════════════════════════════════════════════════════════════════════
    // PUBLIC METHODS: Methods for initializing and updating the target
    // ════════════════════════════════════════════════════════════════════════════

    void TargetController::update(const float& dt)
    {
        // Update target info
        info_.header.stamp = RosUtils::getTimeNow(node_);
        updateInfo(dt, info_.position);

        // Publish target info
        publishInfo(info_);
    }

    core::PointMsg TargetController::getPosition() const
    {
        return info_.position;
    }

    // ════════════════════════════════════════════════════════════════════════════
    // PRIVATE METHODS: Methods for updating the target
    // ════════════════════════════════════════════════════════════════════════════

    void TargetController::updateInfo(const float& dt, core::PointMsg& position)
    {
        // Check if trajectory is empty
        if (trajectory_.empty())
            return;

        // Update time elapsed
        time_elapsed_ += dt;

        // Get next and previous points
        TrajectoryParser::Point next_point;
        TrajectoryParser::Point prev_point;
        if (current_idx_ + 1 >= num_points_)
            next_point = trajectory_[current_idx_];
        else
            next_point = trajectory_[current_idx_ + 1];
        if (current_idx_ - 1 < 0)
            prev_point = trajectory_[current_idx_];
        else
            prev_point = trajectory_[current_idx_ - 1];

        // Find the closest trajectory point based on time
        if (!reverse_)
        {
            if (current_idx_ < num_points_ - 1 && next_point.t <= time_elapsed_)
            {
                current_idx_++;
                // If we reach the end, start moving backwards
                if (current_idx_ == num_points_ - 1)
                {
                    reverse_ = true;
                }
            }
        }
        else
        {
            if (current_idx_ > 0 && prev_point.t <= time_elapsed_)
            {
                current_idx_--;
                // If we reach the start, start moving forwards again
                if (current_idx_ == 0)
                {
                    reverse_ = false;
                }
            }
        }

        // Get current point
        const auto& point = trajectory_[current_idx_];

        // Update target position
        position.x = point.x;
        position.y = point.y;
        position.z = point.z;
    }

    void TargetController::publishInfo(const core::TargetInfoMsg& info)
    {
        // Publish target info
        info_pub_->publish(info);
    }

} // namespace flychams::targets