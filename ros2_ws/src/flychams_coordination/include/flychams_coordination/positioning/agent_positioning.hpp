#pragma once

// Debug message include
#include "flychams_interfaces/msg/solver_debug.hpp"

// Position solver include
#include "flychams_coordination/positioning/cost_functions.hpp"
#include "flychams_coordination/positioning/position_solver.hpp"

// Base module include
#include "flychams_core/base/base_module.hpp"

namespace flychams::coordination
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Agent positioning module
     *
     * @details
     * This class is responsible for computing the position of an agent
     * based on its cluster distribution.
     *
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-03-28
     * ════════════════════════════════════════════════════════════════
     */
    class AgentPositioning : public core::BaseModule
    {
    public: // Constructor/Destructor
        AgentPositioning(const core::ID& agent_id, core::NodePtr node, core::ConfigTools::SharedPtr config_tools, core::FrameworkTools::SharedPtr framework_tools, core::TopicTools::SharedPtr topic_tools, core::TransformTools::SharedPtr transform_tools, core::CallbackGroupPtr module_cb_group)
            : BaseModule(node, config_tools, framework_tools, topic_tools, transform_tools, module_cb_group), agent_id_(agent_id)
        {
            init();
        }

    protected: // Overrides
        void onInit() override;
        void onShutdown() override;

    public: // Types
        using SharedPtr = std::shared_ptr<AgentPositioning>;
        struct Agent
        {
            // Status data
            core::AgentStatus status;
            bool has_status;
            // Position data
            core::PointMsg position;
            bool has_position;
            // Clusters data
            core::AgentClustersMsg clusters;
            bool has_clusters;
            // Setpoint message
            core::PointStampedMsg setpoint;
            // Subscribers
            core::SubscriberPtr<core::AgentStatusMsg> status_sub;
            core::SubscriberPtr<core::PointStampedMsg> position_sub;
            core::SubscriberPtr<core::AgentClustersMsg> clusters_sub;
            // Publisher
            core::PublisherPtr<core::PointStampedMsg> setpoint_pub;
            core::PublisherPtr<flychams_interfaces::msg::SolverDebug> solver_debug_pub;
            // Constructor
            Agent()
                : status(), has_status(false), position(), has_position(false), clusters(),
                has_clusters(false), setpoint(), status_sub(), position_sub(), clusters_sub(),
                setpoint_pub(), solver_debug_pub()
            {
            }
        };

    private: // Parameters
        core::ID agent_id_;
        float update_rate_;
        // Solver modes
        std::vector<PositionSolver::SolverMode> modes_ = {
            PositionSolver::SolverMode::ELLIPSOID_METHOD,
            PositionSolver::SolverMode::PSO_ALGORITHM,
            PositionSolver::SolverMode::ALC_PSO_ALGORITHM,
            PositionSolver::SolverMode::NESTEROV_ALGORITHM,
            PositionSolver::SolverMode::NELDER_MEAD
        };

    private: // Data
        // Agent
        Agent agent_;
        // Position solvers
        std::vector<PositionSolver::SharedPtr>  solvers_;


    private: // Callbacks
        void statusCallback(const core::AgentStatusMsg::SharedPtr msg);
        void positionCallback(const core::PointStampedMsg::SharedPtr msg);
        void clustersCallback(const core::AgentClustersMsg::SharedPtr msg);

    private: // Positioning management
        void update();

    private: // Positioning methods
        CostFunctions::TrackingUnit centralUnitParameters(const core::TrackingParameters& tracking_params);
        std::vector<CostFunctions::TrackingUnit> trackingUnitParameters(const core::TrackingParameters& tracking_params);

    private: // ROS components
        // Timer
        core::TimerPtr update_timer_;
    };

} // namespace flychams::coordination