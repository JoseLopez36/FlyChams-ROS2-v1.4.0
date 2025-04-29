#pragma once

// Assignment solver include
#include "flychams_coordination/assignment/assignment_solver.hpp"

// Base module include
#include "flychams_core/base/base_module.hpp"

namespace flychams::coordination
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Assignment manager for UAV agents
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-01-29
     * ════════════════════════════════════════════════════════════════
     */
    class AgentAssignment : public core::BaseModule
    {
    public: // Constructor/Destructor
        AgentAssignment(core::NodePtr node, core::ConfigTools::SharedPtr config_tools, core::FrameworkTools::SharedPtr framework_tools, core::TopicTools::SharedPtr topic_tools, core::TransformTools::SharedPtr transform_tools, core::CallbackGroupPtr module_cb_group)
            : BaseModule(node, config_tools, framework_tools, topic_tools, transform_tools, module_cb_group)
        {
            init();
        }

    protected: // Overrides
        void onInit() override;
        void onShutdown() override;

    public: // Types
        using SharedPtr = std::shared_ptr<AgentAssignment>;
        struct Agent
        {
            // Status data
            core::AgentStatus status;
            bool has_status;
            // Position data
            core::PointMsg position;
            bool has_position;
            // Subscribers
            core::SubscriberPtr<core::AgentStatusMsg> status_sub;
            core::SubscriberPtr<core::PointStampedMsg> position_sub;
            // Publisher
            core::PublisherPtr<core::AgentAssignmentMsg> assignment_pub;
            // Position solver
            PositionSolver::SharedPtr position_solver;
            // Constructor
            Agent()
                : status(), has_status(false), position(), has_position(false), status_sub(), position_sub(), assignment_pub(), position_solver()
            {
            }
            // Destructor
            ~Agent()
            {
                if (position_solver)
                {
                    position_solver->destroy();
                }
            }
        };
        struct Cluster
        {
            // Geometric data
            core::PointMsg center;
            float radius;
            bool has_geometry;
            // Subscriber
            core::SubscriberPtr<core::ClusterGeometryMsg> geometry_sub;
            // Constructor
            Cluster()
                : center(), radius(), has_geometry(false), geometry_sub()
            {
            }
        };

    private: // Parameters
        float update_rate_;
        // Position solver parameters
        PositionSolver::SolverMode position_solver_mode_;
        PositionSolver::Parameters position_solver_params_;

    private: // Data
        // Agents
        std::unordered_map<core::ID, Agent> agents_;
        std::set<core::ID> A_;
        // Clusters
        std::unordered_map<core::ID, Cluster> clusters_;
        std::set<core::ID> T_;
        // Assignment data
        core::RowVectorXi X_prev_;
        // Assignment solver
        AssignmentSolver::SharedPtr solver_;

    public: // Public methods
        void addAgent(const core::ID& agent_id);
        void addCluster(const core::ID& cluster_id);
        void removeAgent(const core::ID& agent_id);
        void removeCluster(const core::ID& cluster_id);

    private: // Callbacks
        void clusterGeometryCallback(const core::ID& cluster_id, const core::ClusterGeometryMsg::SharedPtr msg);
        void agentStatusCallback(const core::ID& agent_id, const core::AgentStatusMsg::SharedPtr msg);
        void agentPositionCallback(const core::ID& agent_id, const core::PointStampedMsg::SharedPtr msg);

    private: // Assignment management
        void update();

    private: // Utility methods
        PositionSolver::SharedPtr createPositionSolver(const std::string& agent_id, const PositionSolver::Parameters& solver_params, const PositionSolver::SolverMode& solver_mode);
        std::vector<CostFunctions::TrackingUnit> createUnitParameters(const core::TrackingParameters& tracking_params);

    private: // ROS components
        // Timer
        core::TimerPtr update_timer_;
    };

} // namespace flychams::coordination