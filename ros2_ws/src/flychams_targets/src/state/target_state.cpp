#include "flychams_targets/state/target_state.hpp"

using namespace flychams::core;

namespace flychams::targets
{
    // ════════════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR: Constructor and destructor
    // ════════════════════════════════════════════════════════════════════════════

    void TargetState::onInit()
    {
        // Get parameters from parameter server
        // Get update rate
        update_rate_ = RosUtils::getParameterOr<float>(node_, "target_state.state_update_rate", 10.0f);

        // Compute command timeout
        cmd_timeout_ = (1.0f / update_rate_) * 1.25f;

        // Parse trajectory
        const auto& root = config_tools_->getSystem().trajectory_root;
        const auto& folder = config_tools_->getTarget(target_id_)->trajectory_folder;
        const auto& index = config_tools_->getTarget(target_id_)->target_index;
        const auto& path = root + "/" + folder + "/" + "TRAJ" + std::to_string(index + 1) + ".csv";
        RCLCPP_INFO(node_->get_logger(), "Target state: Parsing trajectory for target %s with path %s", target_id_.c_str(), path.c_str());
        trajectory_ = TrajectoryParser::parse(path);

        // Initialize trajectory data
        current_idx_ = 0;
        time_elapsed_ = 0.0f;
        num_points_ = static_cast<int>(trajectory_.size());
        reverse_ = false;

        // Check if trajectory is empty
        if (num_points_ == 0)
        {
            RCLCPP_ERROR(node_->get_logger(), "Target state: No trajectory data available for target %s",
                target_id_.c_str());
            rclcpp::shutdown();
            return;
        }

        // Initialize target position
        position_msg_.header = RosUtils::createHeader(node_, transform_tools_->getGlobalFrame());
        position_msg_.point.x = trajectory_[0].x;
        position_msg_.point.y = trajectory_[0].y;
        position_msg_.point.z = trajectory_[0].z;

        // Initialize target position publisher and publish first message
        position_pub_ = topic_tools_->createTargetTruePositionPublisher(target_id_);
        position_pub_->publish(position_msg_);

        // Set update timer
        last_update_time_ = RosUtils::now(node_);
        update_timer_ = RosUtils::createTimer(node_, update_rate_,
            std::bind(&TargetState::update, this), module_cb_group_);
    }

    void TargetState::onShutdown()
    {
        // Reset time elapsed
        time_elapsed_ = 0.0f;
        // Destroy update timer
        update_timer_.reset();
    }

    // ════════════════════════════════════════════════════════════════════════════
    // UPDATE: Update motion
    // ════════════════════════════════════════════════════════════════════════════

    void TargetState::update()
    {
        // Compute time step
        auto current_time = RosUtils::now(node_);
        float dt = (current_time - last_update_time_).seconds();
        last_update_time_ = current_time;

        // Limit dt to prevent extreme values after pauses
        dt = std::min(dt, cmd_timeout_);

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

        // Find the closest trajectory point based on time and direction
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

        // Update and publish target position
        TrajectoryParser::Point pos = trajectory_[current_idx_];
        position_msg_.header.stamp = RosUtils::now(node_);
        position_msg_.point.x = pos.x;
        position_msg_.point.y = pos.y;
        position_msg_.point.z = pos.z;
        position_pub_->publish(position_msg_);
    }

} // namespace flychams::targets