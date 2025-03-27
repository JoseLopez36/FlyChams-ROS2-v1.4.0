#include "flychams_targets/control/target_motion.hpp"

using namespace flychams::core;

namespace flychams::targets
{
	// ════════════════════════════════════════════════════════════════════════════
	// CONSTRUCTOR: Constructor and destructor
	// ════════════════════════════════════════════════════════════════════════════

	void TargetMotion::onInit()
	{
		// Get parameters from parameter server
		// Get update rate
		update_rate_ = RosUtils::getParameterOr<float>(node_, "target_motion.motion_update_rate", 10.0f);

		// Compute command timeout
		cmd_timeout_ = (1.0f / update_rate_) * 1.25f;

		// Parse trajectory
		const auto& root = config_tools_->getSystem().trajectory_root;
		const auto& folder = config_tools_->getTarget(target_id_)->trajectory_folder;
		const auto& index = config_tools_->getTarget(target_id_)->target_index;
		const auto& path = root + "/" + folder + "/" + "TRAJ" + std::to_string(index) + ".csv";
		RCLCPP_INFO(node_->get_logger(), "Target motion: Parsing trajectory for target %s with path %s", target_id_.c_str(), path.c_str());
		trajectory_ = TrajectoryParser::parse(path);

		// Initialize trajectory data
		current_idx_ = 0;
		time_elapsed_ = 0.0f;
		num_points_ = static_cast<int>(trajectory_.size());
		reverse_ = false;

		// Check if trajectory is empty
		if (num_points_ == 0)
		{
			RCLCPP_ERROR(node_->get_logger(), "Target motion: No trajectory data available for target %s",
				target_id_.c_str());
			rclcpp::shutdown();
			return;
		}

		// Set update timer
		last_update_time_ = RosUtils::now(node_);
		update_timer_ = RosUtils::createTimer(node_, update_rate_,
			std::bind(&TargetMotion::update, this));
	}

	void TargetMotion::onShutdown()
	{
		// Reset time elapsed
		time_elapsed_ = 0.0f;
		// Destroy update timer
		update_timer_.reset();
	}

	// ════════════════════════════════════════════════════════════════════════════
	// UPDATE: Update motion
	// ════════════════════════════════════════════════════════════════════════════

	void TargetMotion::update()
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

		// Command target position
		TrajectoryParser::Point pos = trajectory_[current_idx_];
		PointMsg pos_msg;
		pos_msg.x = pos.x;
		pos_msg.y = pos.y;
		pos_msg.z = pos.z;
		framework_tools_->updateTargetGroup({ target_id_ }, { pos_msg });
	}

} // namespace flychams::targets