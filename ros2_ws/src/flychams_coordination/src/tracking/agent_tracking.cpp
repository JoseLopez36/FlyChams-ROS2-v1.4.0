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
        head_solvers_.clear();
        window_solvers_.clear();

        // Get tracking parameters
        const auto& tracking_params = config_tools_->getTrackingParameters(agent_id_);
        mode_ = tracking_params.mode;
        n_heads_ = tracking_params.n_heads;
        n_windows_ = tracking_params.n_windows;
        head_params_ = tracking_params.head_params;
        window_params_ = tracking_params.window_params;

        // Get relevant transform frames
        world_frame_ = transform_tools_->getGlobalFrame();
        for (const auto& head : head_params_)
        {
            optical_frames_.push_back(transform_tools_->getCameraOpticalFrame(agent_id_, head.id));
        }

        // Initialize selected tracking mode
        switch (mode_)
        {
        case TrackingMode::MultiCamera:
            initializeMultiCamera();
            break;

        case TrackingMode::MultiWindow:
            initializeMultiWindow();
            break;

        default:
            RCLCPP_ERROR(node_->get_logger(), "Agent tracking: Invalid tracking mode");
            break;
        }

        // Create subscribers for agent status, position and clusters
        agent_.status_sub = topic_tools_->createAgentStatusSubscriber(agent_id_,
            std::bind(&AgentTracking::statusCallback, this, std::placeholders::_1), sub_options_with_module_cb_group_);
        agent_.clusters_sub = topic_tools_->createAgentClustersSubscriber(agent_id_,
            std::bind(&AgentTracking::clustersCallback, this, std::placeholders::_1), sub_options_with_module_cb_group_);

        // Create publisher for tracking and GUI setpoints
        agent_.tracking_setpoints_pub = topic_tools_->createAgentTrackingSetpointsPublisher(agent_id_);
        agent_.gui_setpoints_pub = topic_tools_->createGuiSetpointsPublisher(agent_id_);

        // Set update timer
        update_timer_ = RosUtils::createTimer(node_, update_rate_,
            std::bind(&AgentTracking::update, this), module_cb_group_);
    }

    void AgentTracking::onShutdown()
    {
        // Destroy agent data
        agent_.status_sub.reset();
        agent_.clusters_sub.reset();
        agent_.tracking_setpoints_pub.reset();
        agent_.gui_setpoints_pub.reset();
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
        // Check if we have a valid agent status and cluster assignments
        if (!agent_.has_status || !agent_.has_clusters)
        {
            RCLCPP_WARN(node_->get_logger(), "Agent tracking: Agent %s has no status or clusters", agent_id_.c_str());
            return; // Skip tracking if we don't have a valid agent status or clusters
        }

        // Check if we are in the correct state to track
        if (agent_.status != AgentStatus::TRACKING)
        {
            RCLCPP_WARN(node_->get_logger(), "Agent tracking: Agent %s is not in the correct state to track",
                agent_id_.c_str());
            return;
        }

        // Convert clusters message to Eigen types
        int n_clusters = static_cast<int>(agent_.clusters.centers.size());
        Matrix3Xr tab_P = Matrix3Xr::Zero(3, n_clusters);
        RowVectorXr tab_r = RowVectorXr::Zero(n_clusters);
        for (size_t c = 0; c < n_clusters; c++)
        {
            tab_P.col(c) = RosUtils::fromMsg(agent_.clusters.centers[c]);
            tab_r(c) = agent_.clusters.radii[c];
        }

        // Get head transforms
        std::vector<Matrix4r> tab_T(n_heads_);
        for (int h = 0; h < n_heads_; h++)
        {
            const TransformMsg& T = transform_tools_->getTransform(world_frame_, optical_frames_[h]);
            tab_T[h] = RosUtils::fromMsg(T);
        }

        // Solve tracking for selected mode
        switch (mode_)
        {
        case TrackingMode::MultiCamera:
            updateMultiCamera(tab_P, tab_r, tab_T);
            break;

        case TrackingMode::MultiWindow:
            updateMultiWindow(tab_P, tab_r, tab_T[0]);
            break;

        default:
            RCLCPP_ERROR(node_->get_logger(), "Agent tracking: Invalid tracking mode");
            break;
        }

        // Publish tracking and GUI setpoints messages
        agent_.tracking_setpoints_pub->publish(agent_.tracking_setpoints);
        agent_.gui_setpoints_pub->publish(agent_.gui_setpoints);
    }

    // ════════════════════════════════════════════════════════════════════════════
    // TRACKING: Tracking methods
    // ════════════════════════════════════════════════════════════════════════════

    void AgentTracking::initializeMultiCamera()
    {
        // Initialize tracking setpoints message
        agent_.tracking_setpoints.header = RosUtils::createHeader(node_, transform_tools_->getGlobalFrame());

        // Iterate over all heads to fill tracking setpoints
        for (const auto& head : head_params_)
        {
            // Fill message
            agent_.tracking_setpoints.head_ids.push_back(head.id);
            agent_.tracking_setpoints.angles.push_back(Vector3Msg());
            agent_.tracking_setpoints.focals.push_back(0.0f);
            agent_.tracking_setpoints.projected_sizes.push_back(0.0f);
        }

        // Initialize GUI setpoints message
        agent_.gui_setpoints.header = RosUtils::createHeader(node_, transform_tools_->getGlobalFrame());

        // Iterate over all heads to fill GUI setpoints
        for (const auto& head : head_params_)
        {
            // Parameters
            CropMsg full_crop;
            full_crop.x = 0;
            full_crop.y = 0;
            full_crop.w = 0;
            full_crop.h = 0;
            full_crop.is_out_of_bounds = false;

            // Fill message
            agent_.gui_setpoints.camera_ids.push_back(head.id);
            agent_.gui_setpoints.crops.push_back(full_crop);
        }

        // Initialize head solvers
        for (const auto& head : head_params_)
        {
            head_solvers_.push_back(HeadSolver());
            head_solvers_.back().reset();
        }
    }

    void AgentTracking::initializeMultiWindow()
    {
        // Initialize tracking setpoints message
        agent_.tracking_setpoints.header = RosUtils::createHeader(node_, transform_tools_->getGlobalFrame());

        // Fill message for central head
        const auto& head = head_params_[0];
        agent_.tracking_setpoints.head_ids.push_back(head.id);
        agent_.tracking_setpoints.angles.push_back(Vector3Msg());
        agent_.tracking_setpoints.focals.push_back(0.0f);
        // agent_.tracking_setpoints.projected_sizes.push_back(0.0f);   // Not used for multi-window tracking

        // Iterate through all windows to fill tracking setpoints
        for (const auto& window : window_params_)
        {
            // Fill message
            agent_.tracking_setpoints.resolution_factors.push_back(0.0f);
            agent_.tracking_setpoints.projected_sizes.push_back(0.0f);
        }

        // Initialize GUI setpoints message
        agent_.gui_setpoints.header = RosUtils::createHeader(node_, transform_tools_->getGlobalFrame());

        // Iterate through all windows to fill GUI setpoints
        for (const auto& window : window_params_)
        {
            // Parameters
            CropMsg full_crop;
            full_crop.x = 0;
            full_crop.y = 0;
            full_crop.w = 0;
            full_crop.h = 0;
            full_crop.is_out_of_bounds = false;

            // Fill message
            agent_.gui_setpoints.camera_ids.push_back(window.source_id);
            agent_.gui_setpoints.crops.push_back(full_crop);
        }

        // Initialize central head and window solvers
        head_solvers_.push_back(HeadSolver());
        head_solvers_.back().reset();
        for (const auto& window : window_params_)
        {
            window_solvers_.push_back(WindowSolver());
            window_solvers_.back().reset();
        }
    }

    void AgentTracking::updateMultiCamera(const Matrix3Xr& tab_P, const RowVectorXr& tab_r, const std::vector<Matrix4r>& tab_T)
    {
        // Iterate over all heads and solve tracking
        int h = 0;
        for (auto& solver : head_solvers_)
        {
            // Create auxiliary transform
            // This transform is used in the calculation of tracking orientation
            // Reference: world frame
            // Location: optical frame origin
            // Rotation: 180 deg around X
            Matrix4r wTaux = Matrix4r::Identity();
            // Location
            wTaux.block<3, 1>(0, 3) = tab_T[h].block<3, 1>(0, 3);
            // Rotation
            const Quaternionr wQaux = Quaternionr(0.0f, 1.0f, 0.0f, 0.0f);  // w, x, y, z
            wTaux.block<3, 3>(0, 0) = MathUtils::quaternionToRotationMatrix(wQaux);

            // Compute head setpoint
            const auto& [focal, auxRPYc, s_proj_pix] = solver.runCamera(
                tab_P.col(h), tab_r(h), wTaux, head_params_[h]);

            // Convert auxiliary orientation to world frame (same X, inverted Y and Z)
            const Vector3r wRPYc = Vector3r(auxRPYc(0), auxRPYc(1) - M_PI_2f, auxRPYc(2) - M_PI_2f);

            // Update head setpoint
            RosUtils::toMsg(wRPYc, agent_.tracking_setpoints.angles[h]);
            agent_.tracking_setpoints.focals[h] = focal;
            agent_.tracking_setpoints.projected_sizes[h] = s_proj_pix;

            h = h + 1;
        }
    }

    void AgentTracking::updateMultiWindow(const Matrix3Xr& tab_P, const RowVectorXr& tab_r, const Matrix4r& central_T)
    {
        // Solve tracking for central head
        // Create auxiliary transform
        // This transform is used in the calculation of tracking orientation
        // Reference: world frame
        // Location: optical frame origin
        // Rotation: 180 deg around X
        Matrix4r wTaux = Matrix4r::Identity();
        // Location
        wTaux.block<3, 1>(0, 3) = central_T.block<3, 1>(0, 3);
        // Rotation
        const Quaternionr wQaux = Quaternionr(0.0f, 1.0f, 0.0f, 0.0f);  // w, x, y, z
        wTaux.block<3, 3>(0, 0) = MathUtils::quaternionToRotationMatrix(wQaux);

        // Compute head setpoint
        const auto& [focal, auxRPYc, s_proj_pix] = head_solvers_[0].runCamera(
            tab_P.col(0), tab_r(0), wTaux, head_params_[0]);

        // Convert auxiliary orientation to world frame (same X, inverted Y and Z)
        const Vector3r wRPYc = Vector3r(auxRPYc(0), auxRPYc(1) - M_PI_2f, auxRPYc(2) - M_PI_2f);

        // Update head setpoint
        RosUtils::toMsg(wRPYc, agent_.tracking_setpoints.angles[0]);
        agent_.tracking_setpoints.focals[0] = focal;
        //agent_.tracking_setpoints.projected_sizes[0] = s_proj_pix; // Not used for multi-window tracking

        // Iterate over all windows and solve tracking
        int w = 0;
        for (auto& solver : window_solvers_)
        {
            // Compute window setpoint
            const auto& [crop, lambda, s_proj_pix] = solver.runWindow(
                tab_P.col(w), tab_r(w), central_T, head_params_[0], window_params_[w]);

            // Update window setpoint
            agent_.tracking_setpoints.resolution_factors[w] = lambda;
            agent_.tracking_setpoints.projected_sizes[w] = s_proj_pix;

            // Update GUI setpoint (only if window is tracking)
            if (window_params_[w].role == TrackingRole::Tracking)
            {
                RosUtils::toMsg(crop, agent_.gui_setpoints.crops[w]);
            }

            w = w + 1;
        }
    }

} // namespace flychams::coordination