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
            PositionSolver::SolverMode::NELDER_MEAD_NLOPT,
            PositionSolver::SolverMode::NESTEROV_ALGORITHM,
            PositionSolver::SolverMode::L_BFGS_NLOPT,
            PositionSolver::SolverMode::PSO_ALGORITHM,
            PositionSolver::SolverMode::ALC_PSO_ALGORITHM,
            PositionSolver::SolverMode::PSO_ALGORITHM_5000P,
            PositionSolver::SolverMode::ALC_PSO_ALGORITHM_5000P,
            PositionSolver::SolverMode::CMA_ES_ALGORITHM
        };
        PositionSolver::Parameters solver_params_;

    private: // Data
        // Position solvers
        std::unordered_map<PositionSolver::SolverMode, PositionSolver::SharedPtr> solvers_;

    private: // Positioning management
        void update(const int& n_step, const float& t_step);

    private: // Positioning methods
        PositionSolver::SharedPtr createSolver(const std::string& agent_id, const PositionSolver::Parameters& solver_params, const PositionSolver::SolverMode& solver_mode);
        std::vector<CostFunctions::TrackingUnit> createUnitParameters(const core::TrackingParameters& tracking_params);
        std::pair<core::Matrix3Xr, core::RowVectorXr> addNoise(const std::vector<core::Vector3r>& centers, const std::vector<float>& radii, const std::vector<Noise>& centers_noise, const std::vector<Noise>& radii_noise);
        std::pair<core::Matrix3Xr, core::RowVectorXr> addCentral(const core::Matrix3Xr& centers, const core::RowVectorXr& radii);
        float random();

    private: // ROS components
        // Publishers
        core::PublisherPtr<flychams_interfaces::msg::SolverDebug> solver_debug_pub_;

    private: // Experiment parameters
        // General parameters
        const int N_ = 1000;  // Number of iterations per cluster distribution and algorithm
        const int K_ = 4;     // Number of clusters
        // Agent parameters (x mean position)
        const core::Vector3r x0_ = { 0.0f, 0.0f, 90.0f };
        // Cluster parameters
        // Cluster centers
        const std::vector<core::Vector3r> tab_P_ = {
            {20.07f, 39.81f, 0.0f},
            {54.33f, 29.44f, 0.0f},
            {14.10f, -35.11f, 0.0f},
            {-18.12f, 29.50f, 0.0f}
        };
        // Cluster centers noise (mean: 0, std: approximately 5% of the distance between the center and the origin)
        const std::vector<Noise> tab_P_noise_ = {
            {0.0f, 1.0f},
            {0.0f, 1.0f},
            {0.0f, 1.0f},
            {0.0f, 1.0f}
        };
        // Cluster radii
        const std::vector<float> tab_r_ = {
            3.34f,
            1.3f,
            4.7f,
            2.45f
        };
        // Cluster radii noise (mean: 0, std: approximately 5% of the radius)
        const std::vector<Noise> tab_r_noise_ = {
            {0.0f, 0.15f},
            {0.0f, 0.15f},
            {0.0f, 0.15f},
            {0.0f, 0.15f}
        };
    };

} // namespace flychams::coordination
