#include "flychams_coordination/assignment/agent_assignment.hpp"

using namespace std::chrono_literals;
using namespace flychams::core;

namespace flychams::coordination
{
    // ════════════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR: Constructor and destructor
    // ════════════════════════════════════════════════════════════════════════════

    void AgentAssignment::onInit()
    {
        // Get parameters from parameter server
        // Get update rates
        float update_rate = RosUtils::getParameterOr<float>(node_, "agent_assignment.assignment_update_rate", 0.2f);
        float publish_rate = RosUtils::getParameterOr<float>(node_, "agent_assignment.assignment_publish_rate", 10.0f);
        // Get cost weights
        float observation_weight = RosUtils::getParameterOr<float>(node_, "agent_assignment.cost_weights.observation", 1.0f);
        float distance_weight = RosUtils::getParameterOr<float>(node_, "agent_assignment.cost_weights.distance", 10.0f);
        float switch_weight = RosUtils::getParameterOr<float>(node_, "agent_assignment.cost_weights.switch", 3500.0f);
        // Get solver parameters
        float convergence_tolerance = RosUtils::getParameterOr<float>(node_, "agent_assignment.solver_params.convergence_tolerance", 1.0e-6f);
        int max_iterations = RosUtils::getParameterOr<int>(node_, "agent_assignment.solver_params.max_iterations", 100);
        float eps = RosUtils::getParameterOr<float>(node_, "agent_assignment.solver_params.eps", 1.0f);

        // Initialize data
        agent_ids_.clear();
        agents_.clear();
        cluster_ids_.clear();
        clusters_.clear();
        assignments_.clear();
        agent_clusters_.clear();
        has_assignment_ = false;

        // Initialize subscribers and publishers
        agent_odom_subs_.clear();
        cluster_info_subs_.clear();
        agent_info_pubs_.clear();

        // Initialize solver
        AssignmentSolver::SolverParams solver_params;
        solver_params.tol = convergence_tolerance;
        solver_params.max_iter = max_iterations;
        solver_params.eps = eps;
        solver_.setSolverParams(solver_params);
        AssignmentSolver::FunctionParams function_params;
        function_params.w_obs = observation_weight;
        function_params.w_dist = distance_weight;
        function_params.w_switch = switch_weight;
        solver_.setFunctionParams(function_params);

        // Set update timers
        assignment_timer_ = RosUtils::createTimerByRate(node_, update_rate,
            std::bind(&AgentAssignment::updateAssignment, this));
        publish_timer_ = RosUtils::createTimerByRate(node_, publish_rate,
            std::bind(&AgentAssignment::publishAssignment, this));
    }

    void AgentAssignment::onShutdown()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // Destroy subscribers
        agent_odom_subs_.clear();
        cluster_info_subs_.clear();
        // Destroy publishers
        agent_info_pubs_.clear();
        // Destroy data
        agent_ids_.clear();
        agents_.clear();
        cluster_ids_.clear();
        clusters_.clear();
        assignments_.clear();
        // Destroy update timers
        assignment_timer_.reset();
        publish_timer_.reset();
    }

    // ════════════════════════════════════════════════════════════════════════════
    // CALLBACKS: Callback functions
    // ════════════════════════════════════════════════════════════════════════════

    void AgentAssignment::agentOdomCallback(const core::ID& agent_id, const core::OdometryMsg::SharedPtr msg)
    {
        // Transform to world frame
        const std::string& source_frame = msg->header.frame_id;
        const std::string& target_frame = tf_tools_->getWorldFrame();
        const PointMsg& curr_pos_msg = tf_tools_->transformPointMsg(msg->pose.pose.position, source_frame, target_frame);
        // Convert to Eigen
        std::lock_guard<std::mutex> lock(mutex_);
        agents_[agent_id].pos = MsgConversions::fromMsg(curr_pos_msg);
        agents_[agent_id].valid = true;
    }

    void AgentAssignment::clusterInfoCallback(const core::ID& cluster_id, const core::ClusterInfoMsg::SharedPtr msg)
    {
        // Get cluster centers and radii
        std::lock_guard<std::mutex> lock(mutex_);
        clusters_[cluster_id].center = MsgConversions::fromMsg(msg->center);
        clusters_[cluster_id].radius = msg->radius;
        clusters_[cluster_id].valid = true;
    }

    // ════════════════════════════════════════════════════════════════════════════
    // UPDATE: Update assignment and publish
    // ════════════════════════════════════════════════════════════════════════════

    void AgentAssignment::updateAssignment()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        // Check if agents and clusters are valid
        for (const auto& [agent_id, agent] : agents_)
        {
            if (!agent.valid)
            {
                RCLCPP_WARN(node_->get_logger(), "Agent assignment: Agent %s is not valid", agent_id.c_str());
                return;
            }
        }
        for (const auto& [cluster_id, cluster] : clusters_)
        {
            if (!cluster.valid)
            {
                RCLCPP_WARN(node_->get_logger(), "Agent assignment: Cluster %s is not valid", cluster_id.c_str());
                return;
            }
        }

        // Solve for the optimal position
        assignments_ = solver_.solveGreedy(agents_, clusters_);

        // Update agent clusters
        agent_clusters_.clear();
        for (const auto& [cluster_id, agent_id] : assignments_)
        {
            agent_clusters_[agent_id].insert(cluster_id);
        }

        // Set has assignment flag
        has_assignment_ = true;

        RCLCPP_INFO(node_->get_logger(), "Agent assignment: Assignment performed, assignments:");
        for (const auto& [agent_id, cluster_ids] : agent_clusters_)
        {
            RCLCPP_INFO(node_->get_logger(), "	-Agent ID: %s", agent_id.c_str());
            for (const auto& cluster_id : cluster_ids)
            {
                RCLCPP_INFO(node_->get_logger(), "		-Cluster ID: %s", cluster_id.c_str());
            }
        }
    }

    void AgentAssignment::publishAssignment()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        // Check if assignment is valid
        if (!has_assignment_)
            return;

        // Create and fill agent infos
        std::unordered_map<ID, AgentInfoMsg> infos;
        for (const auto& [agent_id, cluster_ids] : agent_clusters_)
        {
            // Get number of clusters
            size_t n_clusters = cluster_ids.size();

            // Get agent info
            auto& info = infos[agent_id];

            // Add cluster info to agent info
            for (const auto& cluster_id : cluster_ids)
            {
                // Extract cluster info
                const auto& cluster = clusters_.at(cluster_id);
                PointMsg center_msg;
                MsgConversions::toMsg(cluster.center, center_msg);
                float radius_msg = cluster.radius;

                // Add to message
                info.cluster_ids.push_back(cluster_id);
                info.centers.push_back(center_msg);
                info.radii.push_back(radius_msg);
            }

            // Publish agent info
            info.header = RosUtils::createHeader(node_, tf_tools_->getWorldFrame());
            agent_info_pubs_[agent_id]->publish(info);
        }
    }

    // ════════════════════════════════════════════════════════════════════════════
    // METHODS: Helper methods
    // ════════════════════════════════════════════════════════════════════════════

    void AgentAssignment::addAgent(const core::ID& agent_id)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // Add agent to set
        agent_ids_.insert(agent_id);
        // Create agent data
        agents_.insert({ agent_id, AssignmentSolver::Agent(config_tools_->getAgent(agent_id)->max_assignments) });
        // Create subscriber
        agent_odom_subs_.insert({ agent_id, topic_tools_->createAgentOdomSubscriber(agent_id,
            [this, agent_id](const OdometryMsg::SharedPtr msg)
            {
                this->agentOdomCallback(agent_id, msg);
            }) });
        // Create publisher
        agent_info_pubs_.insert({ agent_id, topic_tools_->createAgentInfoPublisher(agent_id) });
    }

    void AgentAssignment::removeAgent(const core::ID& agent_id)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // Remove agent from set
        agent_ids_.erase(agent_id);
        // Destroy agent data
        agents_.erase(agent_id);
        // Destroy subscriber
        agent_odom_subs_.erase(agent_id);
        // Destroy publisher
        agent_info_pubs_.erase(agent_id);
    }

    void AgentAssignment::addCluster(const core::ID& cluster_id)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // Add cluster to set
        cluster_ids_.insert(cluster_id);
        // Create cluster data
        clusters_.insert({ cluster_id, AssignmentSolver::Cluster() });
        // Create subscriber
        cluster_info_subs_.insert({ cluster_id, topic_tools_->createClusterInfoSubscriber(cluster_id,
            [this, cluster_id](const ClusterInfoMsg::SharedPtr msg)
            {
                this->clusterInfoCallback(cluster_id, msg);
            }) });
    }

    void AgentAssignment::removeCluster(const core::ID& cluster_id)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // Remove cluster from set
        cluster_ids_.erase(cluster_id);
        // Destroy cluster data
        clusters_.erase(cluster_id);
        // Destroy subscriber
        cluster_info_subs_.erase(cluster_id);
    }

} // namespace flychams::coordination