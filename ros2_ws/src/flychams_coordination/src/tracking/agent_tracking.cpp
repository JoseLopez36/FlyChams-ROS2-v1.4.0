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
        // Get tracking window IDs
        tracking_window_ids_ = RosUtils::getParameter<std::vector<core::ID>>(node_, "window_ids.tracking_ids");

        // Get tracking parameters
        tracking_params_ = config_tools_->getTrackingParameters(agent_id_);

        // Initialize agent data
        curr_pos_ = Vector3r::Zero();
        has_odom_ = false;
        clusters_ = std::make_pair(Matrix3Xr::Zero(3, 0), RowVectorXr::Zero(0));
        has_clusters_ = false;
        central_camera_info_ = CameraInfoMsg();
        has_central_camera_info_ = false;
        prev_angles_ = std::vector<Vector3r>(tracking_params_.n);
        is_first_update_ = true;

        // Prepare tracking goal message
        int n = tracking_params_.n;
        goal_.header = RosUtils::createHeader(node_, tf_tools_->getWorldFrame());
        switch (tracking_params_.mode)
        {
        case TrackingMode::MultiCameraTracking:
            goal_.unit_types = std::vector<uint8_t>(n);
            goal_.window_ids = std::vector<std::string>(n);
            goal_.head_ids = std::vector<std::string>(n);
            goal_.orientations = std::vector<QuaternionMsg>(n);
            goal_.fovs = std::vector<float>(n);

            for (int i = 0; i < n; i++)
            {
                goal_.unit_types[i] = static_cast<uint8_t>(TrackingUnitType::Physical);
                goal_.window_ids[i] = tracking_window_ids_[i];
                goal_.head_ids[i] = tracking_params_.camera_params[i].id;
                goal_.orientations[i] = QuaternionMsg();
                goal_.fovs[i] = 0.0f;
            }
            break;

        case TrackingMode::MultiWindowTracking:
            goal_.unit_types = std::vector<uint8_t>(n);
            goal_.window_ids = std::vector<std::string>(n);
            goal_.camera_id = tracking_params_.window_params[0].camera_params.id;
            goal_.crops = std::vector<CropMsg>(n);

            for (int i = 0; i < n; i++)
            {
                goal_.unit_types[i] = static_cast<uint8_t>(TrackingUnitType::Digital);
                goal_.window_ids[i] = tracking_window_ids_[i];
                goal_.crops[i] = CropMsg();
            }
            break;

        default:
            RCLCPP_ERROR(node_->get_logger(), "AgentTracking: Invalid tracking mode for agent %s", agent_id_.c_str());
            return;
        }

        // Subscribe to odom, agent info and central camera info topics
        odom_sub_ = topic_tools_->createAgentOdomSubscriber(agent_id_,
            std::bind(&AgentTracking::odomCallback, this, std::placeholders::_1));
        info_sub_ = topic_tools_->createAgentInfoSubscriber(agent_id_,
            std::bind(&AgentTracking::infoCallback, this, std::placeholders::_1));
        camera_info_sub_ = topic_tools_->createAgentCameraInfoArraySubscriber(agent_id_,
            std::bind(&AgentTracking::cameraInfoCallback, this, std::placeholders::_1));

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

    void AgentTracking::cameraInfoCallback(const core::CameraInfoArrayMsg::SharedPtr msg)
    {
        // Get camera info
        std::lock_guard<std::mutex> lock(mutex_);
        central_camera_info_ = msg->infos[0];
        has_central_camera_info_ = true;
    }

    // ════════════════════════════════════════════════════════════════════════════
    // UPDATE: Update tracking
    // ════════════════════════════════════════════════════════════════════════════

    void AgentTracking::updateTracking()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        // Check if agent has odometry, clusters and central camera info
        if (!has_odom_ || !has_clusters_ || !has_central_camera_info_)
        {
            RCLCPP_WARN(node_->get_logger(), "Agent %s has no odometry, clusters or central camera info", agent_id_.c_str());
            return;
        }

        // Update tracking goal message based on tracking mode
        switch (tracking_params_.mode)
        {
        case TrackingMode::MultiCameraTracking:
            computeMultiCameraTracking(clusters_.first, clusters_.second, goal_);
            break;

        case TrackingMode::MultiWindowTracking:
            computeMultiWindowTracking(clusters_.first, clusters_.second, central_camera_info_, goal_);
            break;

        default:
            RCLCPP_ERROR(node_->get_logger(), "AgentTracking: Invalid tracking mode for agent %s", agent_id_.c_str());
            return;
        }

        // Publish tracking goal
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
            const TransformMsg& wTc = tf_tools_->getTransformBetweenFrames(tf_tools_->getWorldFrame(), tf_tools_->getHeadOpticalFrame(agent_id_, camera_params.id));
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

    void AgentTracking::computeMultiWindowTracking(const Matrix3Xr& tab_P, const RowVectorXr& tab_r, const CameraInfoMsg& camera_info, TrackingGoalMsg& goal)
    {
        // Get the transform between world and optical frame
        const TransformMsg& wTc = tf_tools_->getTransformBetweenFrames(tf_tools_->getWorldFrame(), tf_tools_->getHeadOpticalFrame(agent_id_, camera_info.header.frame_id));
        const Vector3r& wPc = MsgConversions::fromMsg(wTc.translation);

        for (int i = 0; i < tracking_params_.n; i++)
        {
            // Get window and central camera parameters
            const auto& window_params = tracking_params_.window_params[i];
            const auto& projection_params = tracking_params_.projection_params[i];

            // Get target position and interest radius
            const auto& wPt = tab_P.col(i);
            const auto& r = tab_r(i);

            // Project target on central camera
            PointMsg wPt_msg;
            MsgConversions::toMsg(wPt, wPt_msg);
            const Vector2r p = CameraUtils::projectPoint(wPt_msg, wTc, camera_info);

            // Create tracking crop parameters
            Crop crop = TrackingUtils::computeWindowCrop(wPt, r, wPc, p, window_params, projection_params);

            // Update tracking goal
            goal.crops[i].x = crop.corner(0);
            goal.crops[i].y = crop.corner(1);
            goal.crops[i].w = crop.size(0);
            goal.crops[i].h = crop.size(1);
        }
    }

} // namespace flychams::coordination