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
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-03-28
     * ════════════════════════════════════════════════════════════════
     */
    class AgentPositioningExperiment : public core::BaseModule
    {
    public: // Constructor/Destructor
        AgentPositioningExperiment(const core::ID& agent_id, core::NodePtr node, core::ConfigTools::SharedPtr config_tools, core::FrameworkTools::SharedPtr framework_tools, core::TopicTools::SharedPtr topic_tools, core::TransformTools::SharedPtr transform_tools, core::CallbackGroupPtr module_cb_group)
            : BaseModule(node, config_tools, framework_tools, topic_tools, transform_tools, module_cb_group), agent_id_(agent_id)
        {
            init();
        }

    protected: // Overrides
        void onInit() override;
        void onShutdown() override;

    public: // Types
        using SharedPtr = std::shared_ptr<AgentPositioningExperiment>;
        struct Noise
        {
            float mean; // Mean of the noise (typically 0)
            float std;  // Standard deviation of the noise
        };

    private: // Parameters
        core::ID agent_id_;
        float update_rate_;
        // Position solver parameters
        std::vector<PositionSolver::SolverMode> modes_ = {
            PositionSolver::SolverMode::ELLIPSOID_METHOD,
            PositionSolver::SolverMode::PSO_ALGORITHM,
            PositionSolver::SolverMode::ALC_PSO_ALGORITHM,
            PositionSolver::SolverMode::NESTEROV_ALGORITHM,
            PositionSolver::SolverMode::NELDER_MEAD_NLOPT,
            PositionSolver::SolverMode::L_BFGS_NLOPT
        };
        PositionSolver::Parameters solver_params_;

    private: // Data
        // Position solvers
        std::vector<PositionSolver::SharedPtr>  solvers_;

    private: // Positioning management
        void update(const int& n, const float& t);

    private: // Positioning methods
        PositionSolver::SharedPtr createSolver(const std::string& agent_id, const PositionSolver::Parameters& solver_params, const PositionSolver::SolverMode& solver_mode);
        std::vector<CostFunctions::TrackingUnit> createUnitParameters(const core::TrackingParameters& tracking_params);
        float random();
        core::Vector3r randomVector();

    private: // ROS components
        // Publishers
        std::vector<core::PublisherPtr<flychams_interfaces::msg::SolverDebug>> solver_debug_pubs_;

    private: // Experiment parameters
        // General parameters
        const int N_ = 1000; // Number of iterations per cluster distribution and algorithm
        const int K_ = 9;    // Maximum number of clusters
        // Agent parameters
        const core::Vector3r x0_ = { 0.0f, 0.0f, 70.0f };
        // Cluster parameters (up to 9 clusters)
        // Cluster centers
        const std::vector<core::Vector3r> tab_P_ = {
            {30.0f, 20.0f, 0.0f},
            {100.0f, 0.0f, 0.0f},
            {0.0f, 50.0f, 0.0f},
            {100.0f, 50.0f, 0.0f},
            {0.0f, 0.0f, 0.0f},
            {100.0f, 0.0f, 0.0f},
            {0.0f, 100.0f, 0.0f},
            {100.0f, 100.0f, 0.0f},
            {30.0f, 150.0f, 0.0f}
        };
        const std::vector<Noise> tab_P_noise_ = {
            {0.0f, 0.3f},
            {0.0f, 0.0f},
            {0.0f, 0.0f},
            {0.0f, 0.0f},
            {0.0f, 0.0f},
            {0.0f, 0.0f},
            {0.0f, 0.0f},
            {0.0f, 0.0f},
            {0.0f, 0.0f}
        };
        // Cluster radii
        const std::vector<float> tab_r_ = {
            5.5f,
            15.0f,
            4.0f,
            7.0f,
            10.0f,
            10.0f,
            10.0f,
            10.0f,
            10.0f
        };
        const std::vector<Noise> tab_r_noise_ = {
            {0.0f, 0.1f},
            {0.0f, 0.0f},
            {0.0f, 0.0f},
            {0.0f, 0.0f},
            {0.0f, 0.0f},
            {0.0f, 0.0f},
            {0.0f, 0.0f},
            {0.0f, 0.0f},
            {0.0f, 0.0f}
        };
    };

} // namespace flychams::coordination
