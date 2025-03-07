#include "flychams_coordination/tracking/agent_tracking.hpp"

using namespace std::chrono_literals;
using namespace flychams::core;

namespace flychams::coordination
{
    // ════════════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR: Constructor and destructor
    // ════════════════════════════════════════════════════════════════════════════

    void AgentTracking::onInit()
    {
        // Get parameters from parameter server
        // Get update rate
        float update_rate = RosUtils::getParameterOr<float>(node_, "agent_tracking.tracking_update_rate", 20.0f);
        // Get ROI parameters
        kappa_s_ = RosUtils::getParameterOr<float>(node_, "tracking.kappa_s", 0.8f);
        s_min_pix_ = RosUtils::getParameterOr<float>(node_, "tracking.s_min_pix", 200.0f);
        tracking_window_ids_ = RosUtils::getParameter<std::vector<core::ID>>(node_, "window_ids.tracking_ids");
        num_tracking_windows_ = RosUtils::getParameter<int>(node_, "tracking.num_tracking_windows");
        if (tracking_window_ids_.size() != num_tracking_windows_)
        {
            RCLCPP_ERROR(node_->get_logger(), "AgentTracking: Tracking IDs size (%d) does not match number of tracking windows (%d)", static_cast<int>(tracking_window_ids_.size()), num_tracking_windows_);
            rclcpp::shutdown();
            return;
        }

        // Get head IDs
        central_head_id_ = config_tools_->getAgent(agent_id_)->central_head_id;
        tracking_head_ids_ = config_tools_->getAgent(agent_id_)->tracking_head_ids;
        num_tracking_heads_ = static_cast<int>(tracking_head_ids_.size());
        camera_params_.clear();
        for (int i = 0; i < num_tracking_heads_; i++)
        {
            // Extract camera parameters for each tracking head
            camera_params_.push_back(config_tools_->getCameraParameters(agent_id_, tracking_head_ids_[i]));
        }

        // Initialize agent data
        curr_pos_ = Vector3r::Zero();
        has_odom_ = false;
        clusters_ = std::make_pair(Matrix3Xr::Zero(3, 0), RowVectorXr::Zero(0));
        has_clusters_ = false;
        prev_multi_gimbal_goal_ = MultiGimbalTrackingGoal();
        is_first_update_ = true;

        // Get agent parameters
        tracking_mode_ = config_tools_->getAgent(agent_id_)->tracking_mode;

        // Subscribe to odom and goal topics
        odom_sub_ = topic_tools_->createAgentOdomSubscriber(agent_id_,
            std::bind(&AgentTracking::odomCallback, this, std::placeholders::_1));
        info_sub_ = topic_tools_->createAgentInfoSubscriber(agent_id_,
            std::bind(&AgentTracking::infoCallback, this, std::placeholders::_1));

        // Publish to tracking goal topic
        goal_pub_ = topic_tools_->createTrackingGoalPublisher(agent_id_);

        // Set update timers
        tracking_timer_ = RosUtils::createTimerByRate(node_, update_rate,
            std::bind(&AgentTracking::updateTracking, this));
    }

    void AgentTracking::onShutdown()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // Destroy subscribers
        odom_sub_.reset();
        info_sub_.reset();
        // Destroy publishers
        goal_pub_.reset();
        // Destroy update timers
        tracking_timer_.reset();
    }

    // ════════════════════════════════════════════════════════════════════════════
    // CALLBACKS: Callback functions
    // ════════════════════════════════════════════════════════════════════════════

    void AgentTracking::odomCallback(const core::OdometryMsg::SharedPtr msg)
    {
        // Transform to world frame
        const std::string& source_frame = msg->header.frame_id;
        const std::string& child_frame = msg->child_frame_id;
        const std::string& target_frame = tf_tools_->getWorldFrame();
        const PointMsg& curr_pos_msg = tf_tools_->transformPointMsg(msg->pose.pose.position, source_frame, target_frame);
        // Convert to Eigen
        std::lock_guard<std::mutex> lock(mutex_);
        curr_pos_ = MsgConversions::fromMsg(curr_pos_msg);
        has_odom_ = true;
    }

    void AgentTracking::infoCallback(const core::AgentInfoMsg::SharedPtr msg)
    {
        // Get cluster centers and radii
        std::lock_guard<std::mutex> lock(mutex_);
        clusters_ = MsgConversions::fromMsg(*msg);
        has_clusters_ = true;
    }

    // ════════════════════════════════════════════════════════════════════════════
    // UPDATE: Update tracking
    // ════════════════════════════════════════════════════════════════════════════

    void AgentTracking::updateTracking()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        // Check if agent has odometry and clusters
        if (!has_odom_ || !has_clusters_)
        {
            RCLCPP_WARN(node_->get_logger(), "Agent %s has no odometry or clusters", agent_id_.c_str());
            return;
        }

        // Create tracking goal message based on tracking mode
        TrackingGoalMsg goal_msg;
        switch (tracking_mode_)
        {
        case TrackingMode::MultiGimbalTracking:
            MsgConversions::toMsg(computeMultiGimbalTracking(clusters_.first, clusters_.second), goal_msg);
            break;

        case TrackingMode::MultiCropTracking:
            MsgConversions::toMsg(computeMultiCropTracking(clusters_.first, clusters_.second), goal_msg);
            break;

        case TrackingMode::PriorityHybridTracking:
            MsgConversions::toMsg(computePriorityHybridTracking(clusters_.first, clusters_.second), goal_msg);
            break;
        }

        // Publish tracking goal
        goal_pub_->publish(goal_msg);
        RCLCPP_INFO(node_->get_logger(), "Tracking goal published for agent %s", agent_id_.c_str());
    }

    // ════════════════════════════════════════════════════════════════════════════
    // IMPLEMENTATION: Multi-mode tracking methods
    // ════════════════════════════════════════════════════════════════════════════

    MultiGimbalTrackingGoal AgentTracking::computeMultiGimbalTracking(const Matrix3Xr& tab_P, const RowVectorXr& tab_r)
    {
        // Get head ids
        if (num_tracking_windows_ != num_tracking_heads_)
        {
            RCLCPP_ERROR(node_->get_logger(), "AgentTracking-MultiGimbalTrackingMode: Number of tracking windows (%d) does not match number of tracking ids (%d) for agent %s", num_tracking_windows_, num_tracking_heads_, agent_id_.c_str());
            return MultiGimbalTrackingGoal();
        }

        // Create tracking goal
        MultiGimbalTrackingGoal tracking_goal(num_tracking_heads_);

        for (int i = 0; i < num_tracking_heads_; i++)
        {
            // Get head id and parameters
            const std::string& id = tracking_head_ids_[i];
            const auto& camera_params = camera_params_[i];

            // Get target
            const auto& wPt = tab_P.col(i);
            const auto& r = tab_r(i);

            // First, get the transform between world and optical frame
            const TransformMsg& wTc = tf_tools_->getTransformBetweenFrames(tf_tools_->getWorldFrame(), tf_tools_->getHeadOpticalFrame(agent_id_, id));
            const Vector3r& wPc = MsgConversions::fromMsg(wTc.translation);
            const Matrix3r& wRc = MathUtils::quaternionToRotationMatrix(MsgConversions::fromMsg(wTc.rotation));

            // Second, compute tracking focal length
            float f = TrackingUtils::computeFocal(wPt, r, wPc, camera_params.f_min, camera_params.f_max, camera_params.s_ref);

            // Third, compute tracking orientation
            // Get previous orientation
            Vector3r prev_rpy = Vector3r::Zero();
            if (!is_first_update_)
                prev_rpy = prev_multi_gimbal_goal_.angles.col(i);
            // Calculate new orientation
            Vector3r new_rpy = TrackingUtils::computeOrientation(wPt, wPc, wRc, prev_rpy, is_first_update_);

            // Update tracking goal
            tracking_goal.window_ids[i] = tracking_window_ids_[i];
            tracking_goal.head_ids[i] = id;
            tracking_goal.angles.col(i) = new_rpy;
            tracking_goal.focals[i] = f;
            tracking_goal.sensor_widths[i] = camera_params.sensor_width;
        }

        // Update previous tracking goal
        prev_multi_gimbal_goal_ = tracking_goal;
        is_first_update_ = false;

        return tracking_goal;
    }

    MultiCropTrackingGoal AgentTracking::computeMultiCropTracking(const Matrix3Xr& tab_P, const RowVectorXr& tab_r)
    {
        // TODO: Implement
        return MultiCropTrackingGoal();
    }

    PriorityHybridTrackingGoal AgentTracking::computePriorityHybridTracking(const Matrix3Xr& tab_P, const RowVectorXr& tab_r)
    {
        // TODO: Implement
        return PriorityHybridTrackingGoal();
    }

} // namespace flychams::coordination