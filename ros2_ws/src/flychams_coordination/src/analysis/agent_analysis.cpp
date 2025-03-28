#include "flychams_coordination/analysis/agent_analysis.hpp"

using namespace flychams::core;

namespace flychams::coordination
{
    // ════════════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR: Constructor and destructor
    // ════════════════════════════════════════════════════════════════════════════

    void AgentAnalysis::onInit()
    {
        // Get parameters from parameter server
        // Get update rate
        update_rate_ = RosUtils::getParameterOr<float>(node_, "agent_analysis.analysis_rate", 20.0f);

        // Initialize data
        clusters_.clear();
        agents_.clear();

        // Set update timer
        update_timer_ = RosUtils::createTimer(node_, update_rate_,
            std::bind(&AgentAnalysis::update, this), module_cb_group_);
    }

    void AgentAnalysis::onShutdown()
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

    void AgentAnalysis::addAgent(const ID& agent_id)
    {
        // Create and add agent
        agents_.insert({ agent_id, Agent() });

        // Create agent status subscriber
        agents_[agent_id].status_sub = topic_tools_->createAgentStatusSubscriber(agent_id,
            [this, agent_id](const AgentStatusMsg::SharedPtr msg)
            {
                this->agentStatusCallback(agent_id, msg);
            }, sub_options_with_module_cb_group_);

        // Create agent clusters publisher
        agents_[agent_id].clusters_pub = topic_tools_->createAgentClustersPublisher(agent_id);
    }

    void AgentAnalysis::removeAgent(const ID& agent_id)
    {
        // Remove agent from map
        agents_.erase(agent_id);
    }

    void AgentAnalysis::addCluster(const ID& cluster_id)
    {
        // Create and add cluster
        clusters_.insert({ cluster_id, Cluster() });

        // Create cluster geometry subscriber
        clusters_[cluster_id].geometry_sub = topic_tools_->createClusterGeometrySubscriber(cluster_id,
            [this, cluster_id](const ClusterGeometryMsg::SharedPtr msg)
            {
                this->clusterGeometryCallback(cluster_id, msg);
            }, sub_options_with_module_cb_group_);

        // Create cluster assignment subscriber
        clusters_[cluster_id].assignment_sub = topic_tools_->createClusterAssignmentSubscriber(cluster_id,
            [this, cluster_id](const StringMsg::SharedPtr msg)
            {
                this->clusterAssignmentCallback(cluster_id, msg);
            }, sub_options_with_module_cb_group_);
    }

    void AgentAnalysis::removeCluster(const ID& cluster_id)
    {
        // Remove cluster from map
        clusters_.erase(cluster_id);
    }

    // ════════════════════════════════════════════════════════════════════════════
    // CALLBACKS: Callback functions
    // ════════════════════════════════════════════════════════════════════════════

    void AgentAnalysis::clusterGeometryCallback(const ID& cluster_id, const ClusterGeometryMsg::SharedPtr msg)
    {
        // Update cluster geometry
        clusters_[cluster_id].center = msg->center;
        clusters_[cluster_id].radius = msg->radius;
        clusters_[cluster_id].has_geometry = true;
    }

    void AgentAnalysis::clusterAssignmentCallback(const ID& cluster_id, const StringMsg::SharedPtr msg)
    {
        // Update cluster assignment
        clusters_[cluster_id].assignment = msg->data;
        clusters_[cluster_id].has_assignment = true;
    }

    void AgentAnalysis::agentStatusCallback(const ID& agent_id, const AgentStatusMsg::SharedPtr msg)
    {
        // Update agent status
        agents_[agent_id].status = static_cast<AgentStatus>(msg->status);
        agents_[agent_id].has_status = true;
    }

    // ════════════════════════════════════════════════════════════════════════════
    // UPDATE: Update analysis
    // ════════════════════════════════════════════════════════════════════════════

    void AgentAnalysis::update()
    {
        // Check if we have a valid agent status, cluster geometries and assignments
        for (const auto& [agent_id, agent] : agents_)
        {
            if (!agent.has_status)
            {
                RCLCPP_WARN(node_->get_logger(), "Agent analysis: Agent %s has no status", agent_id.c_str());
                return; // Skip updating if we don't have a valid agent status
            }

            // Check if we are in the correct state to analyze
            if (agent.status != AgentStatus::TRACKING)
            {
                RCLCPP_WARN(node_->get_logger(), "Agent analysis: Agent %s is not in the correct state to analyze",
                    agent_id.c_str());
                return;
            }
        }
        for (const auto& [cluster_id, cluster] : clusters_)
        {
            if (!cluster.has_geometry || !cluster.has_assignment)
            {
                RCLCPP_WARN(node_->get_logger(), "Agent analysis: Cluster %s has no geometry or assignment", cluster_id.c_str());
                return; // Skip updating if we don't have a valid cluster geometry or assignment
            }
        }

        // Iterate over agents to get their assignments
        for (const auto& [agent_id, agent] : agents_)
        {
            // Create agent clusters message
            AgentClustersMsg clusters_msg;
            clusters_msg.header = RosUtils::createHeader(node_, transform_tools_->getGlobalFrame());

            // Add cluster info to agent info
            for (const auto& [cluster_id, cluster] : clusters_)
            {
                // Add to message
                clusters_msg.cluster_ids.push_back(cluster_id);
                clusters_msg.centers.push_back(cluster.center);
                clusters_msg.radii.push_back(cluster.radius);
            }

            // Publish agent clusters
            agent.clusters_pub->publish(clusters_msg);
        }
    }

} // namespace flychams::coordination