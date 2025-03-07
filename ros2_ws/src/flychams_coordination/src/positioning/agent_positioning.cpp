#include "flychams_coordination/positioning/agent_positioning.hpp"

using namespace std::chrono_literals;
using namespace flychams::core;

namespace flychams::coordination
{
    // ════════════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR: Constructor and destructor
    // ════════════════════════════════════════════════════════════════════════════

    void AgentPositioning::onInit()
    {
        // Get parameters from parameter server
        // Get update rates
        float update_rate = RosUtils::getParameterOr<float>(node_, "agent_positioning.positioning_update_rate", 5.0f);
        // Get ROI parameters
        kappa_s_ = RosUtils::getParameterOr<float>(node_, "tracking.kappa_s", 0.8f);
        s_min_pix_ = RosUtils::getParameterOr<float>(node_, "tracking.s_min_pix", 200.0f);
        // Get solver parameters
        float convergence_tolerance = RosUtils::getParameterOr<float>(node_, "agent_positioning.solver_params.convergence_tolerance", 1.0e-6f);
        int max_iterations = RosUtils::getParameterOr<int>(node_, "agent_positioning.solver_params.max_iterations", 100);
        float eps = RosUtils::getParameterOr<float>(node_, "agent_positioning.solver_params.eps", 1.0f);

        // Initialize agent data
        curr_pos_ = Vector3r::Zero();
        has_odom_ = false;
        clusters_ = std::make_pair(Matrix3Xr::Zero(3, 0), RowVectorXr::Zero(0));
        has_clusters_ = false;

        // Set solver parameters
        solver_ = std::make_shared<PositionSolver>();
        PositionSolver::SolverParams solver_params;
        solver_params.tol = convergence_tolerance;
        solver_params.max_iter = max_iterations;
        solver_params.eps = eps;
        solver_->setSolverParams(solver_params);

        // Set function parameters
        const auto& agent_ptr = config_tools_->getAgent(agent_id_);
        float h_min = agent_ptr->min_admissible_height;
        float h_max = agent_ptr->max_admissible_height;
        int tracking_count = static_cast<int>(agent_ptr->tracking_head_ids.size());
        auto function_params = PositionSolver::FunctionParams(tracking_count, h_min, h_max);
        for (int i = 0; i < tracking_count; i++)
        {
            function_params.camera_params_[i] = config_tools_->getCameraParameters(agent_id_, agent_ptr->tracking_head_ids[i]);
        }
        solver_->setFunctionParams(function_params);

        // Initialize solver
        solver_->initSolver();

        // Subscribe to odom and goal topics
        odom_sub_ = topic_tools_->createAgentOdomSubscriber(agent_id_,
            std::bind(&AgentPositioning::odomCallback, this, std::placeholders::_1));
        info_sub_ = topic_tools_->createAgentInfoSubscriber(agent_id_,
            std::bind(&AgentPositioning::infoCallback, this, std::placeholders::_1));

        // Publish to goal topic
        goal_pub_ = topic_tools_->createAgentGoalPublisher(agent_id_);

        // Set update timers
        positioning_timer_ = RosUtils::createTimerByRate(node_, update_rate,
            std::bind(&AgentPositioning::updatePosition, this));
    }

    void AgentPositioning::onShutdown()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // Destroy subscribers
        odom_sub_.reset();
        info_sub_.reset();
        // Destroy publishers
        goal_pub_.reset();
        // Destroy solver
        solver_->destroySolver();
        // Destroy update timers
        positioning_timer_.reset();
    }

    // ════════════════════════════════════════════════════════════════════════════
    // CALLBACKS: Callback functions
    // ════════════════════════════════════════════════════════════════════════════

    void AgentPositioning::odomCallback(const core::OdometryMsg::SharedPtr msg)
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

    void AgentPositioning::infoCallback(const core::AgentInfoMsg::SharedPtr msg)
    {
        // Get cluster centers and radii
        std::lock_guard<std::mutex> lock(mutex_);
        clusters_ = MsgConversions::fromMsg(*msg);
        has_clusters_ = true;
    }

    // ════════════════════════════════════════════════════════════════════════════
    // UPDATE: Solve positioning problem
    // ════════════════════════════════════════════════════════════════════════════

    void AgentPositioning::updatePosition()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        // Check if agent has odom and clusters
        if (!has_odom_ || !has_clusters_)
        {
            RCLCPP_WARN(node_->get_logger(), "Agent positioning: Agent %s has no odom or clusters", agent_id_.c_str());
            return;
        }

        // Solve for the optimal position
        Vector3r optimal_pos = solver_->solve(curr_pos_, clusters_.first, clusters_.second);

        // Publish the goal
        AgentGoalMsg goal_msg;
        goal_msg.header = RosUtils::createHeader(node_, tf_tools_->getWorldFrame());
        MsgConversions::toMsg(optimal_pos, goal_msg);
        goal_pub_->publish(goal_msg);

        RCLCPP_INFO(node_->get_logger(), "Agent positioning: Agent %s has updated optimal goal to x=%f, y=%f, z=%f",
            agent_id_.c_str(), optimal_pos.x(), optimal_pos.y(), optimal_pos.z());
    }

} // namespace flychams::coordination