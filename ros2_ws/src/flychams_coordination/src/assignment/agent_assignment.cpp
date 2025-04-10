#include "flychams_coordination/assignment/agent_assignment.hpp"

using namespace flychams::core;

namespace flychams::coordination
{
    // ════════════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR: Constructor and destructor
    // ════════════════════════════════════════════════════════════════════════════

    void AgentAssignment::onInit()
    {
        // Get parameters from parameter server
        // Get update rate
        update_rate_ = RosUtils::getParameterOr<float>(node_, "agent_assignment.update_rate", 1.0f);
        // Get assignment mode
        assignment_mode_ = static_cast<AssignmentSolver::AssignmentMode>(RosUtils::getParameterOr<int>(node_, "agent_assignment.mode", 0));

        // Initialize data
        clusters_.clear();
        agents_.clear();

        // Initialize solver
        solver_.reset();
        solver_.setMode(assignment_mode_);

        // Set update timer
        update_timer_ = RosUtils::createTimer(node_, update_rate_,
            std::bind(&AgentAssignment::update, this), module_cb_group_);
    }

    void AgentAssignment::onShutdown()
    {
        // Destroy clusters and agents
        clusters_.clear();
        agents_.clear();
        // Destroy update timer
        update_timer_.reset();
    }

    // ════════════════════════════════════════════════════════════════════════════
    // PUBLIC METHODS: Public methods for adding/removing clusters and agents
    // ════════════════════════════════════════════════════════════════════════════

    void AgentAssignment::addAgent(const ID& agent_id)
    {
        // Create and add agent
        agents_.insert({ agent_id, Agent() });

        // Get assignment count
        agents_[agent_id].max_assignments = config_tools_->getTrackingParameters(agent_id).n;

        // Create agent status subscriber
        agents_[agent_id].status_sub = topic_tools_->createAgentStatusSubscriber(agent_id,
            [this, agent_id](const AgentStatusMsg::SharedPtr msg)
            {
                this->agentStatusCallback(agent_id, msg);
            }, sub_options_with_module_cb_group_);

        // Create agent position subscriber
        agents_[agent_id].position_sub = topic_tools_->createAgentPositionSubscriber(agent_id,
            [this, agent_id](const PointStampedMsg::SharedPtr msg)
            {
                this->agentPositionCallback(agent_id, msg);
            }, sub_options_with_module_cb_group_);
    }

    void AgentAssignment::removeAgent(const ID& agent_id)
    {
        // Remove agent from map
        agents_.erase(agent_id);
    }

    void AgentAssignment::addCluster(const ID& cluster_id)
    {
        // Create and add cluster
        clusters_.insert({ cluster_id, Cluster() });

        // Create cluster geometry subscriber
        clusters_[cluster_id].geometry_sub = topic_tools_->createClusterGeometrySubscriber(cluster_id,
            [this, cluster_id](const ClusterGeometryMsg::SharedPtr msg)
            {
                this->clusterGeometryCallback(cluster_id, msg);
            }, sub_options_with_module_cb_group_);

        // Create cluster assignment publisher
        clusters_[cluster_id].assignment_pub = topic_tools_->createClusterAssignmentPublisher(cluster_id);
    }

    void AgentAssignment::removeCluster(const ID& cluster_id)
    {
        // Remove cluster from map
        clusters_.erase(cluster_id);
    }

    // ════════════════════════════════════════════════════════════════════════════
    // CALLBACKS: Callback functions
    // ════════════════════════════════════════════════════════════════════════════

    void AgentAssignment::clusterGeometryCallback(const ID& cluster_id, const ClusterGeometryMsg::SharedPtr msg)
    {
        // Update cluster geometry
        clusters_[cluster_id].center = msg->center;
        clusters_[cluster_id].radius = msg->radius;
        clusters_[cluster_id].has_geometry = true;
    }

    void AgentAssignment::agentStatusCallback(const ID& agent_id, const AgentStatusMsg::SharedPtr msg)
    {
        // Update agent status
        agents_[agent_id].status = static_cast<AgentStatus>(msg->status);
        agents_[agent_id].has_status = true;
    }

    void AgentAssignment::agentPositionCallback(const ID& agent_id, const PointStampedMsg::SharedPtr msg)
    {
        // Update agent position
        agents_[agent_id].position = msg->point;
        agents_[agent_id].has_position = true;
    }

    // ════════════════════════════════════════════════════════════════════════════
    // UPDATE: Update assignment
    // ════════════════════════════════════════════════════════════════════════════

    void AgentAssignment::update()
    {
        // Check if we have a valid agent status, position and cluster geometry
        for (const auto& [agent_id, agent] : agents_)
        {
            if (!agent.has_status || !agent.has_position)
            {
                RCLCPP_WARN(node_->get_logger(), "Agent assignment: Agent %s has no status or position", agent_id.c_str());
                return; // Skip assignment if we don't have a valid agent status or position
            }

            // Check if we are in the correct state to assign clusters
            if (agent.status != AgentStatus::TRACKING)
            {
                RCLCPP_WARN(node_->get_logger(), "Agent assignment: Agent %s is not in the correct state to assign clusters",
                    agent_id.c_str());
                return;
            }
        }
        for (const auto& [cluster_id, cluster] : clusters_)
        {
            if (!cluster.has_geometry)
            {
                RCLCPP_WARN(node_->get_logger(), "Agent assignment: Cluster %s has no geometry", cluster_id.c_str());
                return; // Skip assignment if we don't have a valid cluster geometry
            }
        }

        // Create points map
        AssignmentSolver::Clusters clusters;
        for (const auto& [cluster_id, cluster] : clusters_)
        {
            clusters.insert({ cluster_id, {RosUtils::fromMsg(cluster.center), cluster.radius} });
        }
        AssignmentSolver::Agents agents;
        for (const auto& [agent_id, agent] : agents_)
        {
            agents.insert({ agent_id, {RosUtils::fromMsg(agent.position), agent.max_assignments} });
        }

        // Perform agent assignment based on assignment mode
        AssignmentSolver::Assignments assignments;
        switch (assignment_mode_)
        {
        case AssignmentSolver::AssignmentMode::GREEDY:
        {
            assignments = solver_.runGreedy(clusters, agents);
            break;
        }

        default:
            RCLCPP_ERROR(node_->get_logger(), "Agent assignment: Invalid assignment mode");
            return;
        }

        // Publish assignments
        for (const auto& [cluster_id, agent_id] : assignments)
        {
            // Create assignment message
            StringMsg assignment_msg;
            assignment_msg.data = agent_id;

            // Publish assignment
            clusters_[cluster_id].assignment_pub->publish(assignment_msg);

            // Log assignment
            RCLCPP_INFO(node_->get_logger(), "Agent assignment: Cluster %s assigned to agent %s", cluster_id.c_str(), agent_id.c_str());
        }
    }

} // namespace flychams::coordination