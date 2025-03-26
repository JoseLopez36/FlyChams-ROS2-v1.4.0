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

        // Get tracking parameters
        tracking_params_ = config_tools_->getTrackingParameters(agent_id_);

        // Get central head ID
        central_head_id_ = config_tools_->getAgent(agent_id_)->central_head_id;

        // Initialize agent data
        curr_pos_ = Vector3r::Zero();
        has_odom_ = false;
        clusters_ = std::make_pair(Matrix3Xr::Zero(3, 0), RowVectorXr::Zero(0));
        has_clusters_ = false;
        prev_angles_ = std::vector<Vector3r>(tracking_params_.n);
        is_first_update_ = true;

        // Prepare tracking goal message
        int n = tracking_params_.n;
        goal_.header = RosUtils::createHeader(node_, transform_tools_->getGlobalFrame());
        switch (tracking_params_.mode)
        {
        case TrackingMode::MultiCameraTracking:
            goal_.unit_types = std::vector<uint8_t>(n);
            goal_.head_ids = std::vector<std::string>(n);
            goal_.orientations = std::vector<QuaternionMsg>(n);
            goal_.fovs = std::vector<float>(n);

            for (int i = 0; i < n; i++)
            {
                goal_.unit_types[i] = static_cast<uint8_t>(TrackingUnitType::Physical);
                goal_.head_ids[i] = tracking_params_.camera_params[i].id;
                goal_.orientations[i] = QuaternionMsg();
                goal_.fovs[i] = 0.0f;
            }
            break;

        case TrackingMode::MultiWindowTracking:
            goal_.camera_id = tracking_params_.window_params[0].camera_params.id;
            goal_.unit_types = std::vector<uint8_t>(n);
            goal_.crops = std::vector<CropMsg>(n);
            goal_.resolution_factors = std::vector<float>(n);

            for (int i = 0; i < n; i++)
            {
                goal_.unit_types[i] = static_cast<uint8_t>(TrackingUnitType::Digital);
                goal_.crops[i] = CropMsg();
                goal_.resolution_factors[i] = 0.0f;
            }
            break;

        default:
            RCLCPP_ERROR(node_->get_logger(), "AgentTracking: Invalid tracking mode for agent %s", agent_id_.c_str());
            rclcpp::shutdown();
            return;
        }

        // Subscribe to odom and agent info topics
        odom_sub_ = topic_tools_->createAgentOdomSubscriber(agent_id_,
            std::bind(&AgentTracking::odomCallback, this, std::placeholders::_1));
        info_sub_ = topic_tools_->createAgentTrackingInfoSubscriber(agent_id_,
            std::bind(&AgentTracking::infoCallback, this, std::placeholders::_1));

        // Publish to tracking goal topic   
        goal_pub_ = topic_tools_->createAgentTrackingGoalPublisher(agent_id_);

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
        // Get agent position
        std::lock_guard<std::mutex> lock(mutex_);
        curr_pos_ = MsgConversions::fromMsg(msg->pose.pose.position);
        has_odom_ = true;
    }

    void AgentTracking::infoCallback(const core::TrackingInfoMsg::SharedPtr msg)
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

        // Update tracking goal message based on tracking mode
        switch (tracking_params_.mode)
        {
        case TrackingMode::MultiCameraTracking:
            computeMultiCameraTracking(clusters_.first, clusters_.second, goal_);
            break;

        case TrackingMode::MultiWindowTracking:
            computeMultiWindowTracking(clusters_.first, clusters_.second, goal_);
            break;

        default:
            RCLCPP_ERROR(node_->get_logger(), "AgentTracking: Invalid tracking mode for agent %s", agent_id_.c_str());
            return;
        }

        // Publish tracking goal
        goal_.header.stamp = RosUtils::getTimeNow(node_);
        goal_pub_->publish(goal_);
        RCLCPP_INFO(node_->get_logger(), "Tracking goal published for agent %s", agent_id_.c_str());
    }

    // ════════════════════════════════════════════════════════════════════════════
    // IMPLEMENTATION: Multi-mode tracking methods
    // ════════════════════════════════════════════════════════════════════════════

    void AgentTracking::computeMultiCameraTracking(const Matrix3Xr& tab_P, const RowVectorXr& tab_r, TrackingGoalMsg& goal)
    {
        for (int i = 0; i < tracking_params_.n; i++)
        {
            // Get camera parameters
            const auto& camera_params = tracking_params_.camera_params[i];
            const auto& projection_params = tracking_params_.projection_params[i];

            // Get target position and interest radius
            const auto& wPt = tab_P.col(i);
            const auto& r = tab_r(i);

            // First, get the transform between world and optical frame
            const TransformMsg& wTc = transform_tools_->getTransformBetweenFrames(transform_tools_->getGlobalFrame(), transform_tools_->getCameraOpticalFrame(agent_id_, camera_params.id));
            const Vector3r& wPc = MsgConversions::fromMsg(wTc.translation);
            const Matrix3r& wRc = MathUtils::quaternionToRotationMatrix(MsgConversions::fromMsg(wTc.rotation));

            // Second, compute tracking focal length
            float new_f = TrackingUtils::computeFocal(wPt, r, wPc, camera_params, projection_params);

            // Third, compute tracking orientation
            // Get previous orientation
            Vector3r prev_rpy = Vector3r::Zero();
            if (!is_first_update_)
                prev_rpy = prev_angles_[i];
            // Calculate new orientation
            Vector3r new_rpy = TrackingUtils::computeOrientation(wPt, wPc, wRc, prev_rpy, is_first_update_);
            prev_angles_[i] = new_rpy;

            // Update tracking goal
            MsgConversions::toMsg(MathUtils::eulerToQuaternion(new_rpy), goal.orientations[i]);
            goal.fovs[i] = CameraUtils::computeFov(new_f, camera_params.sensor_width);
        }

        // Update first update flag
        is_first_update_ = false;
    }

    void AgentTracking::computeMultiWindowTracking(const Matrix3Xr& tab_P, const RowVectorXr& tab_r, TrackingGoalMsg& goal)
    {
        // Get central camera parameters
        const auto& camera_params = tracking_params_.window_params[0].camera_params;

        // Get the transform between world and optical frame
        const std::string& world_frame = transform_tools_->getGlobalFrame();
        const std::string& optical_frame = transform_tools_->getCameraOpticalFrame(agent_id_, central_head_id_);
        const TransformMsg& world_to_optical = transform_tools_->getTransform(world_frame, optical_frame);
        const Matrix4r& wTc = MsgConversions::fromMsg(world_to_optical);
        const Vector3r& wPc = wTc.block<3, 1>(0, 3);

        // Project points on central camera
        Matrix2Xr tab_p = CameraUtils::projectPoints(tab_P, wTc, camera_params.k_ref);

        // Flip the horizontal coordinates to handle X-axis mirroring
        for (int i = 0; i < tab_p.cols(); i++)
        {
            tab_p(0, i) = camera_params.width - tab_p(0, i);
        }

        // Update tracking goal
        for (int i = 0; i < tracking_params_.n; i++)
        {
            // Get window and central camera parameters
            const auto& window_params = tracking_params_.window_params[i];
            const auto& projection_params = tracking_params_.projection_params[i];

            // Get target position, interest radius and projected point
            const auto& wPt = tab_P.col(i);
            const auto& r = tab_r(i);
            const auto& p = tab_p.col(i);

            // Create tracking crop parameters
            float lambda;
            Vector2i size = TrackingUtils::computeWindowSize(wPt, r, wPc, window_params, projection_params, lambda);
            Vector2i corner = TrackingUtils::computeWindowCorner(p, size);

            // Check if crop is out of bounds (i.e. if the crop is completely outside the image bounds)
            bool is_out_of_bounds =
                (corner(0) + size(0) <= 0) ||           // Completely to the left
                (corner(1) + size(1) <= 0) ||           // Completely above
                (corner(0) >= camera_params.width) ||   // Completely to the right
                (corner(1) >= camera_params.height);    // Completely below

            // Update tracking goal
            goal.crops[i].x = corner(0);
            goal.crops[i].y = corner(1);
            goal.crops[i].w = size(0);
            goal.crops[i].h = size(1);
            goal.crops[i].is_out_of_bounds = is_out_of_bounds;
            goal.resolution_factors[i] = lambda;

            // Print tracking goal
            RCLCPP_INFO(node_->get_logger(), "Tracking goal for window %d: x=%d, y=%d, w=%d, h=%d, lambda=%f", i, goal.crops[i].x, goal.crops[i].y, goal.crops[i].w, goal.crops[i].h, goal.resolution_factors[i]);
        }
    }

} // namespace flychams::coordination