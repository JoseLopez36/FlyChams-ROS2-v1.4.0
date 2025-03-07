#include "flychams_coordination/positioning/position_solver.hpp"

using namespace flychams::core;

namespace flychams::coordination
{
    // ════════════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR: Constructor and destructor
    // ════════════════════════════════════════════════════════════════════════════

    PositionSolver::PositionSolver()
        : function_params_(FunctionParams(0, 0.0f, 300.0f))
    {
        // Initialize solver parameters
        solver_params_.tol = 1e-6f;
        solver_params_.max_iter = 100;
        solver_params_.eps = 1.0f;
    }

    PositionSolver::~PositionSolver()
    {
        destroySolver();
    }

    // ════════════════════════════════════════════════════════════════════════════
    // METHODS: Configuration and runtime methods
    // ════════════════════════════════════════════════════════════════════════════

    void PositionSolver::setSolverParams(const SolverParams& params)
    {
        if (params.eps <= 0.0f || params.tol <= 0.0f)
            throw std::invalid_argument("Tolerances must be positive");

        solver_params_.tol = params.tol;
        solver_params_.max_iter = params.max_iter;
        solver_params_.eps = params.eps;
    }

    void PositionSolver::setFunctionParams(const FunctionParams& params)
    {
        if (params.n <= 0)
            throw std::invalid_argument("Number of cameras must be positive");
        if (params.h_min >= params.h_max)
            throw std::invalid_argument("h_min must be less than h_max");
        if (params.camera_params_.size() != params.n)
            throw std::invalid_argument("Number of camera parameters must match the number of cameras");

        function_params_ = params;
    }

    void PositionSolver::initSolver()
    {
        // Create an NLopt optimizer
        int dim = 3; // Dimension of the problem
        opt_ = nlopt_create(NLOPT_LN_NELDERMEAD, dim);
        if (!opt_)
        {
            throw std::runtime_error("Failed to create NLOpt optimizer");
            return;
        }

        // Define the optimization bounds
        const double lb[3] = { -HUGE_VAL, -HUGE_VAL, static_cast<double>(function_params_.h_min) };
        const double ub[3] = { HUGE_VAL, HUGE_VAL, static_cast<double>(function_params_.h_max) };
        nlopt_set_lower_bounds(opt_, lb);
        nlopt_set_upper_bounds(opt_, ub);

        // Optimization options
        nlopt_set_xtol_rel(opt_, static_cast<double>(solver_params_.tol)); // Set convergence tolerance
        nlopt_set_maxeval(opt_, solver_params_.max_iter);                  // Maximum number of function evaluations
    }

    void PositionSolver::destroySolver()
    {
        if (opt_)
        {
            nlopt_destroy(opt_);
        }
    }

    Vector3r PositionSolver::solve(const Vector3r& x0, const Matrix3Xr& tab_P, const RowVectorXr& tab_r, const PositioningMode& mode)
    {
        // Clip height to limits
        Vector3r x0_clipped = x0;
        x0_clipped(2) = std::clamp(x0(2), static_cast<float>(function_params_.h_min), static_cast<float>(function_params_.h_max));

        // First optimization (solve for initial position)
        Vector3r x_opt_1 = preOptimization(x0_clipped, tab_P, tab_r, mode);

        // Iterative optimization (solve for optimal position)
        return iterativeOptimization(x_opt_1, tab_P, tab_r, solver_params_.eps, mode);
    }

    // ════════════════════════════════════════════════════════════════════════════
    // IMPLEMENTATION: Optimization methods
    // ════════════════════════════════════════════════════════════════════════════

    Vector3r PositionSolver::preOptimization(const Vector3r& x0, const Matrix3Xr& tab_P, const RowVectorXr& tab_r, const PositioningMode& mode)
    {
        // Set the objective cost function J1 (with parameters)
        FunctionData dataJ1(function_params_, tab_P, tab_r, Vector3r::Zero());
        switch (mode)
        {
        case PositioningMode::MultiCameraPositioning:
            nlopt_set_min_objective(opt_, calculateMultiCameraJ1, &dataJ1);
            break;

        case PositioningMode::MultiWindowPositioning:
            nlopt_set_min_objective(opt_, calculateMultiWindowJ1, &dataJ1);
            break;

        default:
            throw std::invalid_argument("Invalid positioning mode");
        }

        // Optimize for J1
        Vector3r x_opt = x0;
        optimize(x_opt);

        return x_opt;
    }

    Vector3r PositionSolver::iterativeOptimization(const Vector3r& x0, const Matrix3Xr& tab_P, const RowVectorXr& tab_r, const float& eps, const PositioningMode& mode)
    {
        // Initialize with the previous solution
        Vector3r x_opt_prev = x0;
        Vector3r x_opt = x_opt_prev;

        // Define the data for the convex relaxation (J2)
        FunctionData dataJ2(function_params_, tab_P, tab_r, Vector3r::Zero());

        // Iteratively call the optimization algorithm that implements the convex relaxation (J2)
        float x_dif_norm = HUGE_VALF;
        while (x_dif_norm > eps)
        {
            // Define the xHat parameter of cost function J2
            dataJ2.x_hat << x_opt_prev[0], x_opt_prev[1], x_opt_prev[2];

            // Set the objective cost function J2 (with parameters)
            switch (mode)
            {
            case PositioningMode::MultiCameraPositioning:
                nlopt_set_min_objective(opt_, calculateMultiCameraJ2, &dataJ2);
                break;

            case PositioningMode::MultiWindowPositioning:
                nlopt_set_min_objective(opt_, calculateMultiWindowJ2, &dataJ2);
                break;

            default:
                throw std::invalid_argument("Invalid positioning mode");
            }

            // Optimize for J2
            optimize(x_opt);

            // Compute the norm of difference
            x_dif_norm = (x_opt - x_opt_prev).norm();

            // Update the previous solution
            x_opt_prev = x_opt;
        }

        return x_opt;
    }

    double PositionSolver::calculateMultiCameraJ1(unsigned n, const double* x, double* grad, void* data)
    {
        // Extract data
        FunctionData* function_data = reinterpret_cast<FunctionData*>(data);
        Vector3r x_vec(static_cast<float>(x[0]), static_cast<float>(x[1]), static_cast<float>(x[2]));

        // Compute the value of the optimization index based on nested intervals
        float J = 0.0f;
        for (int i = 0; i < function_data->params.n; i++)
        {
            // Extract the necessary parameters
            const auto& tracking_params = function_data->params.tracking_params_;
            const float& s_min = tracking_params.s_min;
            const float& s_max = tracking_params.s_max;
            const float& s_ref = tracking_params.s_ref;
            const auto& camera_params = function_data->params.camera_params_[i];
            const float& f_min = camera_params.f_min;
            const float& f_max = camera_params.f_max;
            const float& f_ref = camera_params.f_ref;

            // Target position and distance to its camera (approximated by distance to the vehicle)
            const Vector3r z_vec = function_data->tab_P.col(i);
            const float d = (x_vec - z_vec).norm();

            // Equivalent interest radius of the real target
            const float r = function_data->tab_r(i);

            // Calculate the reference distance to the target
            const float d_ref = r * f_ref / s_ref;

            // Calculate what would be the ideal reference position in the case of a single target (perfect verticallity)
            Vector3r p_ref_vec = z_vec;
            p_ref_vec(2) += d_ref;

            // Determine the nested intervals
            const float L0 = d_ref;
            const float U0 = d_ref;
            const float L1 = r * f_min / s_ref;
            const float U1 = r * f_max / s_ref;
            const float L2 = r * f_min / s_max;
            const float U2 = r * f_max / s_min;

            // Calculate the index terms based on intervals
            const float psi_i = 1.0f * pow((std::max)(0.0f, d - U0), 2) + 2.0f * pow((std::max)(0.0f, d - U1), 2) + 10.0f * pow((std::max)(0.0f, d - U2), 2);
            const float lambda_i = 0.0f; // Non-convex term is not considered

            const float mu_i = 1.0f;
            const float nu_i = 1.0f;
            const float gamma_i = mu_i * (x_vec - p_ref_vec).transpose() * (x_vec - p_ref_vec) + nu_i * pow((d - (x_vec - z_vec).transpose() * Vector3r(0.0f, 0.0f, 1.0f)), 2);

            // Accumulate into the total index
            J += psi_i + lambda_i + gamma_i;
        }

        return static_cast<double>(J);
    }

    double PositionSolver::calculateMultiCameraJ2(unsigned n, const double* x, double* grad, void* data)
    {
        // Extract data
        FunctionData* function_data = reinterpret_cast<FunctionData*>(data);
        Vector3r x_vec(static_cast<float>(x[0]), static_cast<float>(x[1]), static_cast<float>(x[2]));

        // Distance threshold to consider that xHat coincides with zi
        // Considering that hMin is several meters, it should not be reached unless set very high, since zi are at height 0
        float eps_dist = 0.1f;

        // Compute the value of the optimization index based on nested intervals
        float J = 0.0f;
        for (int i = 0; i < function_data->params.n; i++)
        {
            // Extract the necessary parameters
            const auto& tracking_params = function_data->params.tracking_params_;
            const float& s_min = tracking_params.s_min;
            const float& s_max = tracking_params.s_max;
            const float& s_ref = tracking_params.s_ref;
            const auto& camera_params = function_data->params.camera_params_[i];
            const float& f_min = camera_params.f_min;
            const float& f_max = camera_params.f_max;
            const float& f_ref = camera_params.f_ref;

            // Target position and distance to its camera (approximated by distance to the vehicle)
            const Vector3r z_vec = function_data->tab_P.col(i);
            const float d = (x_vec - z_vec).norm();

            // Vector indicating the direction to project
            const Vector3r v_vec = function_data->x_hat - z_vec;
            const float v_norm = v_vec.norm();
            Vector3r eta_vec = Vector3r::Zero();
            if (v_norm > eps_dist)
                eta_vec = v_vec / v_norm;

            // Calculate the projection as a substitute for distance for the non-convex term
            const float d_proj = (x_vec - z_vec).transpose() * eta_vec;

            // Equivalent interest radius of the real target
            const float r = function_data->tab_r(i);

            // Calculate the reference distance to the target
            const float d_ref = r * f_ref / s_ref;

            // Calculate what would be the ideal reference position in the case of a single target (perfect verticallity)
            Vector3r p_ref_vec = z_vec;
            p_ref_vec(2) += d_ref;

            // Determine the nested intervals
            const float L0 = d_ref;
            const float U0 = d_ref;
            const float L1 = r * f_min / s_ref;
            const float U1 = r * f_max / s_ref;
            const float L2 = r * f_min / s_max;
            const float U2 = r * f_max / s_min;

            // Calculate the index terms based on intervals
            const float alpha = 2.0f;
            const float psi_i = 1.0f * pow((std::max)(0.0f, d - U0), alpha) + 2.0f * pow((std::max)(0.0f, d - U1), alpha) + 10.0f * pow((std::max)(0.0f, d - U2), alpha);
            const float lambda_i = 1.0f * pow((std::max)(0.0f, L0 - d_proj), alpha) + 2.0f * pow((std::max)(0.0f, L1 - d_proj), alpha) + 10.0f * pow((std::max)(0.0f, L2 - d_proj), alpha);

            const float mu_i = 1.0f;
            const float nu_i = 1.0f;
            const float gamma_i = mu_i * (x_vec - p_ref_vec).transpose() * (x_vec - p_ref_vec) + nu_i * pow((d - (x_vec - z_vec).transpose() * Vector3r(0.0f, 0.0f, 1.0f)), 2);

            // Accumulate into the total index
            J += psi_i + lambda_i + gamma_i;
        }

        return static_cast<double>(J);
    }

    double PositionSolver::calculateMultiWindowJ1(unsigned n, const double* x, double* grad, void* data)
    {
        // Extract data
        FunctionData* function_data = reinterpret_cast<FunctionData*>(data);
        Vector3r x_vec(static_cast<float>(x[0]), static_cast<float>(x[1]), static_cast<float>(x[2]));

        // Compute the value of the optimization index based on nested intervals
        float J = 0.0f;
        for (int i = 0; i < function_data->params.n; i++)
        {
            // Extract the necessary parameters
            const auto& tracking_params = function_data->params.tracking_params_;
            const float& s_min = tracking_params.s_min;
            const float& s_max = tracking_params.s_max;
            const float& s_ref = tracking_params.s_ref;
            const auto& window_params = function_data->params.window_params_[i];
            const float& f = window_params.f;
            const float& rho = window_params.rho;
            const float& lambda_min = window_params.lambda_min;
            const float& lambda_max = window_params.lambda_max;
            const float& lambda_ref = window_params.lambda_ref;

            // Target position and distance to its camera (approximated by distance to the vehicle)
            const Vector3r z_vec = function_data->tab_P.col(i);
            const float d = (x_vec - z_vec).norm();

            // Equivalent interest radius of the real target
            const float r = function_data->tab_r(i);

            // Calculate the reference distance to the target
            const float d_ref = (r * f * lambda_ref) / (s_ref * rho);

            // Calculate what would be the ideal reference position in the case of a single target (perfect verticallity)
            Vector3r p_ref_vec = z_vec;
            p_ref_vec(2) += d_ref;

            // Determine the nested intervals
            const float L0 = d_ref;
            const float U0 = d_ref;
            const float L1 = (r * f * lambda_min) / (s_ref * rho);
            const float U1 = (r * f * lambda_max) / (s_ref * rho);
            const float L2 = (r * f * lambda_min) / (s_max * rho);
            const float U2 = (r * f * lambda_max) / (s_min * rho);

            // Calculate the index terms based on intervals
            const float psi_i = 1.0f * pow((std::max)(0.0f, d - U0), 2) + 2.0f * pow((std::max)(0.0f, d - U1), 2) + 10.0f * pow((std::max)(0.0f, d - U2), 2);
            const float lambda_i = 0.0f; // Non-convex term is not considered

            const float mu_i = 1.0f;
            const float nu_i = 1.0f;
            const float gamma_i = mu_i * (x_vec - p_ref_vec).transpose() * (x_vec - p_ref_vec) + nu_i * pow((d - (x_vec - z_vec).transpose() * Vector3r(0.0f, 0.0f, 1.0f)), 2);

            // Accumulate into the total index
            J += psi_i + lambda_i + gamma_i;
        }

        return static_cast<double>(J);
    }

    double PositionSolver::calculateMultiWindowJ2(unsigned n, const double* x, double* grad, void* data)
    {
        // Extract data
        FunctionData* function_data = reinterpret_cast<FunctionData*>(data);
        Vector3r x_vec(static_cast<float>(x[0]), static_cast<float>(x[1]), static_cast<float>(x[2]));

        // Distance threshold to consider that xHat coincides with zi
        // Considering that hMin is several meters, it should not be reached unless set very high, since zi are at height 0
        float eps_dist = 0.1f;

        // Compute the value of the optimization index based on nested intervals
        float J = 0.0f;
        for (int i = 0; i < function_data->params.n; i++)
        {
            // Extract the necessary parameters
            const auto& tracking_params = function_data->params.tracking_params_;
            const float& s_min = tracking_params.s_min;
            const float& s_max = tracking_params.s_max;
            const float& s_ref = tracking_params.s_ref;
            const auto& window_params = function_data->params.window_params_[i];
            const float& f = window_params.f;
            const float& rho = window_params.rho;
            const float& lambda_min = window_params.lambda_min;
            const float& lambda_max = window_params.lambda_max;
            const float& lambda_ref = window_params.lambda_ref;

            // Target position and distance to its camera (approximated by distance to the vehicle)
            const Vector3r z_vec = function_data->tab_P.col(i);
            const float d = (x_vec - z_vec).norm();

            // Vector indicating the direction to project
            const Vector3r v_vec = function_data->x_hat - z_vec;
            const float v_norm = v_vec.norm();
            Vector3r eta_vec = Vector3r::Zero();
            if (v_norm > eps_dist)
                eta_vec = v_vec / v_norm;

            // Calculate the projection as a substitute for distance for the non-convex term
            const float d_proj = (x_vec - z_vec).transpose() * eta_vec;

            // Equivalent interest radius of the real target
            const float r = function_data->tab_r(i);

            // Calculate the reference distance to the target
            const float d_ref = (r * f * lambda_ref) / (s_ref * rho);

            // Calculate what would be the ideal reference position in the case of a single target (perfect verticallity)
            Vector3r p_ref_vec = z_vec;
            p_ref_vec(2) += d_ref;

            // Determine the nested intervals
            const float L0 = d_ref;
            const float U0 = d_ref;
            const float L1 = (r * f * lambda_min) / (s_ref * rho);
            const float U1 = (r * f * lambda_max) / (s_ref * rho);
            const float L2 = (r * f * lambda_min) / (s_max * rho);
            const float U2 = (r * f * lambda_max) / (s_min * rho);

            // Calculate the index terms based on intervals
            const float alpha = 2.0f;
            const float psi_i = 1.0f * pow((std::max)(0.0f, d - U0), alpha) + 2.0f * pow((std::max)(0.0f, d - U1), alpha) + 10.0f * pow((std::max)(0.0f, d - U2), alpha);
            // Convex approximation of the non-convex term
            const float lambda_i = 1.0f * pow((std::max)(0.0f, L0 - d_proj), alpha) + 2.0f * pow((std::max)(0.0f, L1 - d_proj), alpha) + 10.0f * pow((std::max)(0.0f, L2 - d_proj), alpha);

            const float mu_i = 1.0f;
            const float nu_i = 1.0f;
            const float gamma_i = mu_i * (x_vec - p_ref_vec).transpose() * (x_vec - p_ref_vec) + nu_i * pow((d - (x_vec - z_vec).transpose() * Vector3r(0.0f, 0.0f, 1.0f)), 2);

            // Accumulate into the total index
            J += psi_i + lambda_i + gamma_i;
        }

        return static_cast<double>(J);
    }

    float PositionSolver::optimize(Vector3r& x_opt)
    {
        double x_opt_nlopt[3] = { static_cast<double>(x_opt(0)), static_cast<double>(x_opt(1)), static_cast<double>(x_opt(2)) };

        // Call the optimization algorithm
        // J: optimal value of the cost function
        // x_opt: value that minimizes the cost function
        double J;
        nlopt_optimize(opt_, x_opt_nlopt, &J);

        // Update the position
        x_opt << static_cast<float>(x_opt_nlopt[0]), static_cast<float>(x_opt_nlopt[1]), static_cast<float>(x_opt_nlopt[2]);

        // Return the optimal value of the cost function
        return static_cast<float>(J);
    }

} // namespace flychams::coordination