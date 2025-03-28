#include "flychams_coordination/tracking/agent_tracking.hpp"

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
        update_rate_ = RosUtils::getParameterOr<float>(node_, "agent_tracking.tracking_rate", 20.0f);

        // Initialize data
        agent_ = Agent();

        // Get tracking parameters
        tracking_params_ = config_tools_->getTrackingParameters(agent_id_);
        int n = tracking_params_.n; // Number of tracking units

        // Get central head parameters
        const auto& central_id = config_tools_->getCentralHead(agent_id_)->id;

        // Get tracking heads
        const auto& tracking_head_ptrs = config_tools_->getTrackingHeads(agent_id_);

        // Initialize setpoint messages
        agent_.head_setpoints.header = RosUtils::createHeader(node_, transform_tools_->getGlobalFrame());
        agent_.window_setpoints.header = RosUtils::createHeader(node_, transform_tools_->getGlobalFrame());
        switch (tracking_params_.mode)
        {
        case TrackingMode::MultiCameraTracking:
            agent_.head_setpoints.head_ids = std::vector<std::string>(n);
            agent_.head_setpoints.quat_setpoints = std::vector<QuaternionMsg>(n, QuaternionMsg());
            agent_.head_setpoints.fov_setpoints = std::vector<float>(n, 0.0f);

            for (int i = 0; i < n; i++)
            {
                agent_.head_setpoints.head_ids[i] = tracking_head_ptrs[i]->id;
            }
            break;

        case TrackingMode::MultiWindowTracking:
            agent_.window_setpoints.camera_id = central_id;
            agent_.window_setpoints.crop_setpoints = std::vector<CropMsg>(n, CropMsg());
            break;

        default:
            RCLCPP_ERROR(node_->get_logger(), "AgentTracking: Invalid tracking mode for agent %s", agent_id_.c_str());
            rclcpp::shutdown();
            return;
        }

        // Initialize solvers
        solvers_.resize(n);
        for (int i = 0; i < n; i++)
        {
            solvers_[i].reset();
        }

        // Create subscribers for agent status, position and clusters
        agent_.status_sub = topic_tools_->createAgentStatusSubscriber(agent_id_,
            std::bind(&AgentTracking::statusCallback, this, std::placeholders::_1), sub_options_with_module_cb_group_);
        agent_.position_sub = topic_tools_->createAgentPositionSubscriber(agent_id_,
            std::bind(&AgentTracking::positionCallback, this, std::placeholders::_1), sub_options_with_module_cb_group_);
        agent_.clusters_sub = topic_tools_->createAgentClustersSubscriber(agent_id_,
            std::bind(&AgentTracking::clustersCallback, this, std::placeholders::_1), sub_options_with_module_cb_group_);

        // Create publisher for tracking setpoints
        agent_.head_setpoints_pub = topic_tools_->createAgentHeadSetpointsPublisher(agent_id_);
        agent_.window_setpoints_pub = topic_tools_->createAgentWindowSetpointsPublisher(agent_id_);

        // Set update timer
        update_timer_ = RosUtils::createTimer(node_, update_rate_,
            std::bind(&AgentTracking::update, this), module_cb_group_);
    }

    void AgentTracking::onShutdown()
    {
        // Destroy agent data
        agent_.status_sub.reset();
        agent_.position_sub.reset();
        agent_.clusters_sub.reset();
        agent_.head_setpoints_pub.reset();
        agent_.window_setpoints_pub.reset();
        // Destroy update timer
        update_timer_.reset();
    }

    // ════════════════════════════════════════════════════════════════════════════
    // CALLBACKS: Callback functions
    // ════════════════════════════════════════════════════════════════════════════

    void AgentTracking::statusCallback(const AgentStatusMsg::SharedPtr msg)
    {
        // Update agent status
        agent_.status = static_cast<AgentStatus>(msg->status);
        agent_.has_status = true;
    }

    void AgentTracking::positionCallback(const PointStampedMsg::SharedPtr msg)
    {
        // Update agent position
        agent_.position = msg->point;
        agent_.has_position = true;
    }

    void AgentTracking::clustersCallback(const AgentClustersMsg::SharedPtr msg)
    {
        // Update agent clusters
        agent_.clusters = *msg;
        agent_.has_clusters = true;
    }

    // ════════════════════════════════════════════════════════════════════════════
    // UPDATE: Update tracking
    // ════════════════════════════════════════════════════════════════════════════

    void AgentTracking::update()
    {
        // Check if we have a valid agent status, position and cluster assignments
        if (!agent_.has_status || !agent_.has_position || !agent_.has_clusters)
        {
            RCLCPP_WARN(node_->get_logger(), "Agent tracking: Agent %s has no status, position or clusters", agent_id_.c_str());
            return; // Skip tracking if we don't have a valid agent status, position or clusters
        }

        // Check if we are in the correct state to track
        if (agent_.status != AgentStatus::TRACKING)
        {
            RCLCPP_WARN(node_->get_logger(), "Agent tracking: Agent %s is not in the correct state to track",
                agent_id_.c_str());
            return;
        }

        // Convert messages to Eigen types
        Vector3r x = RosUtils::fromMsg(agent_.position);
        int n = static_cast<int>(agent_.clusters.cluster_ids.size());
        Matrix3Xr tab_P = Matrix3Xr::Zero(3, n);
        RowVectorXr tab_r = RowVectorXr::Zero(n);
        for (size_t i = 0; i < n; i++)
        {
            tab_P.col(i) = RosUtils::fromMsg(agent_.clusters.centers[i]);
            tab_r(i) = agent_.clusters.radii[i];
        }

        // Solve tracking
        switch (tracking_params_.mode)
        {
        case TrackingMode::MultiCameraTracking:
            computeMultiCamera(tab_P, tab_r, agent_.head_setpoints);
            break;

        case TrackingMode::MultiWindowTracking:
            computeMultiWindow(tab_P, tab_r, agent_.window_setpoints);
            break;
        }

        // Publish tracking setpoints
        agent_.head_setpoints_pub->publish(agent_.head_setpoints);
        agent_.window_setpoints_pub->publish(agent_.window_setpoints);
    }

    // ════════════════════════════════════════════════════════════════════════════
    // IMPLEMENTATION: Multi-mode tracking methods
    // ════════════════════════════════════════════════════════════════════════════

    void AgentTracking::computeMultiCamera(const Matrix3Xr& tab_P, const RowVectorXr& tab_r, AgentHeadSetpointsMsg& setpoints)
    {
        for (int i = 0; i < tracking_params_.n; i++)
        {
            // Get camera parameters
            const auto& camera_params = tracking_params_.camera_params[i];
            const auto& projection_params = tracking_params_.projection_params[i];

            // Get target position and interest radius
            const auto& z = tab_P.col(i);
            const auto& r = tab_r(i);

            // Get the transform between world and camera optical frame
            const std::string& world_frame = transform_tools_->getGlobalFrame();
            const std::string& optical_frame = transform_tools_->getCameraOpticalFrame(agent_id_, camera_params.id);
            const TransformMsg& world_to_optical = transform_tools_->getTransform(world_frame, optical_frame);
            const Matrix4r& T = RosUtils::fromMsg(world_to_optical);

            // Solve tracking for this camera
            const auto& [focal, rpy] = solvers_[i].runCamera(z, r, T, camera_params, projection_params);

            // Update tracking setpoint
            RosUtils::toMsg(MathUtils::eulerToQuaternion(rpy), setpoints.quat_setpoints[i]);
            setpoints.fov_setpoints[i] = MathUtils::computeFov(focal, camera_params.sensor_width);
        }
    }

    void AgentTracking::computeMultiWindow(const Matrix3Xr& tab_P, const RowVectorXr& tab_r, AgentWindowSetpointsMsg& setpoints)
    {
        // Get the transform between world and central camera optical frame
        const std::string& world_frame = transform_tools_->getGlobalFrame();
        const std::string& optical_frame = transform_tools_->getCameraOpticalFrame(agent_id_, setpoints.camera_id);
        const TransformMsg& world_to_optical = transform_tools_->getTransform(world_frame, optical_frame);
        const Matrix4r& T = RosUtils::fromMsg(world_to_optical);

        // Update tracking setpoints
        for (int i = 0; i < tracking_params_.n; i++)
        {
            // Get window and central camera parameters
            const auto& window_params = tracking_params_.window_params[i];
            const auto& projection_params = tracking_params_.projection_params[i];

            // Get target position and interest radius
            const auto& z = tab_P.col(i);
            const auto& r = tab_r(i);

            // Solve tracking for this window
            const auto& [size, corner] = solvers_[i].runWindow(z, r, T, window_params, projection_params);

            // Check if crop is out of bounds (i.e. if the crop is completely outside the image)
            bool is_out_of_bounds =
                (corner(0) + size(0) <= 0) ||                           // Completely to the left
                (corner(1) + size(1) <= 0) ||                           // Completely above
                (corner(0) >= window_params.camera_params.width) ||     // Completely to the right
                (corner(1) >= window_params.camera_params.height);      // Completely below

            // Update tracking setpoint
            auto& crop = setpoints.crop_setpoints[i];
            crop.x = corner(0);
            crop.y = corner(1);
            crop.w = size(0);
            crop.h = size(1);
            crop.is_out_of_bounds = is_out_of_bounds;
        }
    }

} // namespace flychams::coordination