#include "flychams_coordination/positioning/position_solver.hpp"

using namespace flychams::core;

namespace flychams::coordination
{
    // ════════════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR: Constructor and destructor
    // ════════════════════════════════════════════════════════════════════════════

    PositionSolver::PositionSolver()
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
        if (params.h_min >= params.h_max)
            throw std::invalid_argument("h_min must be less than h_max");
        if (params.tracking_params.n <= 0)
            throw std::invalid_argument("Number of cameras must be positive");

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

    Vector3r PositionSolver::solve(const Vector3r& x0, const Matrix3Xr& tab_P, const RowVectorXr& tab_r)
    {
        // Clip height to limits
        Vector3r x0_clipped = x0;
        x0_clipped(2) = std::clamp(x0(2), function_params_.h_min, function_params_.h_max);

        // First optimization (solve for initial position)
        Vector3r x_opt_1 = preOptimization(x0_clipped, tab_P, tab_r);

        // Iterative optimization (solve for optimal position)
        return iterativeOptimization(x_opt_1, tab_P, tab_r, solver_params_.eps);
    }

    // ════════════════════════════════════════════════════════════════════════════
    // IMPLEMENTATION: Optimization methods
    // ════════════════════════════════════════════════════════════════════════════

    Vector3r PositionSolver::preOptimization(const Vector3r& x0, const Matrix3Xr& tab_P, const RowVectorXr& tab_r)
    {
        // Set the objective cost function J1 (with parameters)
        FunctionData dataJ1(function_params_, tab_P, tab_r, Vector3r::Zero());
        nlopt_set_min_objective(opt_, funJ1, &dataJ1);

        // Optimize for J1
        Vector3r x_opt = x0;
        optimize(x_opt);

        return x_opt;
    }

    Vector3r PositionSolver::iterativeOptimization(const Vector3r& x0, const Matrix3Xr& tab_P, const RowVectorXr& tab_r, const float& eps)
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
            nlopt_set_min_objective(opt_, funJ2, &dataJ2);

            // Optimize for J2
            optimize(x_opt);

            // Compute the norm of difference
            x_dif_norm = (x_opt - x_opt_prev).norm();

            // Update the previous solution
            x_opt_prev = x_opt;
        }

        return x_opt;
    }

    // ════════════════════════════════════════════════════════════════════════════
    // IMPLEMENTATION: Objective functions
    // ════════════════════════════════════════════════════════════════════════════

    double PositionSolver::funJ1(unsigned n, const double* x, double* grad, void* data)
    {
        // Extract data
        FunctionData* function_data = reinterpret_cast<FunctionData*>(data);
        Vector3r x_vec(static_cast<float>(x[0]), static_cast<float>(x[1]), static_cast<float>(x[2]));

        // Compute the value of the optimization index based on nested intervals (without non-convex term) based on tracking mode
        float J1 = 0.0f;
        switch (function_data->params.tracking_params.mode)
        {
        case TrackingMode::MultiCameraTracking:
            for (int i = 0; i < function_data->params.tracking_params.n; i++)
            {
                J1 += calculateCameraJ1(i, x_vec, function_data);
            }
            break;

        case TrackingMode::MultiWindowTracking:
            for (int i = 0; i < function_data->params.tracking_params.n; i++)
            {
                J1 += calculateWindowJ1(i, x_vec, function_data);
            }
            break;

        default:
            throw std::invalid_argument("Invalid tracking mode");
        }

        // Return the value of J1
        return static_cast<double>(J1);
    }

    double PositionSolver::funJ2(unsigned n, const double* x, double* grad, void* data)
    {
        // Extract data
        FunctionData* function_data = reinterpret_cast<FunctionData*>(data);
        Vector3r x_vec(static_cast<float>(x[0]), static_cast<float>(x[1]), static_cast<float>(x[2]));

        // Compute the value of the optimization index based on nested intervals (with non-convex term) based on tracking mode
        float J2 = 0.0f;
        switch (function_data->params.tracking_params.mode)
        {
        case TrackingMode::MultiCameraTracking:
            for (int i = 0; i < function_data->params.tracking_params.n; i++)
            {
                J2 += calculateCameraJ2(i, x_vec, function_data);
            }
            break;

        case TrackingMode::MultiWindowTracking:
            for (int i = 0; i < function_data->params.tracking_params.n; i++)
            {
                J2 += calculateWindowJ2(i, x_vec, function_data);
            }
            break;

        default:
            throw std::invalid_argument("Invalid tracking mode");
        }

        // Return the value of J2
        return static_cast<double>(J2);
    }

    // ════════════════════════════════════════════════════════════════════════════
    // IMPLEMENTATION: Cost functions
    // ════════════════════════════════════════════════════════════════════════════

    float PositionSolver::calculateCameraJ1(const int& i, const Vector3r& x, const FunctionData* data)
    {
        // Extract the necessary parameters
        const auto& tracking_params = data->params.tracking_params;
        const float& s_min = tracking_params.projection_params[i].s_min;
        const float& s_max = tracking_params.projection_params[i].s_max;
        const float& s_ref = tracking_params.projection_params[i].s_ref;
        const float& f_min = tracking_params.camera_params[i].f_min;
        const float& f_max = tracking_params.camera_params[i].f_max;
        const float& f_ref = tracking_params.camera_params[i].f_ref;

        // Target position and distance to its camera (approximated by distance to the vehicle)
        const Vector3r z = data->tab_P.col(i);
        const float d = (x - z).norm();

        // Equivalent interest radius of the real target
        const float r = data->tab_r(i);

        // Calculate the reference distance to the target
        const float d_ref = r * f_ref / s_ref;

        // Calculate what would be the ideal reference position in the case of a single target (perfect verticallity)
        Vector3r p_ref = z;
        p_ref(2) += d_ref;

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
        const float gamma_i = mu_i * (x - p_ref).transpose() * (x - p_ref) + nu_i * pow((d - (x - z).transpose() * Vector3r(0.0f, 0.0f, 1.0f)), 2);

        // Return the value of Ji
        return psi_i + lambda_i + gamma_i;
    }

    float PositionSolver::calculateCameraJ2(const int& i, const Vector3r& x, const FunctionData* data)
    {
        // Distance threshold to consider that xHat coincides with zi
        // Considering that hMin is several meters, it should not be reached unless set very high, since zi are at height 0
        float eps_dist = 0.1f;

        // Extract the necessary parameters
        const auto& tracking_params = data->params.tracking_params;
        const float& s_min = tracking_params.projection_params[i].s_min;
        const float& s_max = tracking_params.projection_params[i].s_max;
        const float& s_ref = tracking_params.projection_params[i].s_ref;
        const float& f_min = tracking_params.camera_params[i].f_min;
        const float& f_max = tracking_params.camera_params[i].f_max;
        const float& f_ref = tracking_params.camera_params[i].f_ref;

        // Target position and distance to its camera (approximated by distance to the vehicle)
        const Vector3r z = data->tab_P.col(i);
        const float d = (x - z).norm();

        // Vector indicating the direction to project
        const Vector3r v = data->x_hat - z;
        const float v_norm = v.norm();
        Vector3r eta = Vector3r::Zero();
        if (v_norm > eps_dist)
            eta = v / v_norm;

        // Calculate the projection as a substitute for distance for the non-convex term
        const float d_proj = (x - z).transpose() * eta;

        // Equivalent interest radius of the real target
        const float r = data->tab_r(i);

        // Calculate the reference distance to the target
        const float d_ref = r * f_ref / s_ref;

        // Calculate what would be the ideal reference position in the case of a single target (perfect verticallity)
        Vector3r p_ref = z;
        p_ref(2) += d_ref;

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
        const float gamma_i = mu_i * (x - p_ref).transpose() * (x - p_ref) + nu_i * pow((d - (x - z).transpose() * Vector3r(0.0f, 0.0f, 1.0f)), 2);

        // Accumulate into the total index
        return psi_i + lambda_i + gamma_i;
    }

    float PositionSolver::calculateWindowJ1(const int& i, const Vector3r& x, const FunctionData* data)
    {
        // Extract the necessary parameters
        const auto& tracking_params = data->params.tracking_params;
        const float& s_min = tracking_params.projection_params[i].s_min;
        const float& s_max = tracking_params.projection_params[i].s_max;
        const float& s_ref = tracking_params.projection_params[i].s_ref;
        const float& f = tracking_params.window_params[i].camera_params.f_ref;
        const float& lambda_min = tracking_params.window_params[i].lambda_min;
        const float& lambda_max = tracking_params.window_params[i].lambda_max;
        const float& lambda_ref = tracking_params.window_params[i].lambda_ref;

        // Target position and distance to its camera (approximated by distance to the vehicle)
        const Vector3r z = data->tab_P.col(i);
        const float d = (x - z).norm();

        // Equivalent interest radius of the real target
        const float r = data->tab_r(i);

        // Calculate the reference distance to the target
        const float d_ref = (r * f * lambda_ref) / s_ref;

        // Calculate what would be the ideal reference position in the case of a single target (perfect verticallity)
        Vector3r p_ref = z;
        p_ref(2) += d_ref;

        // Determine the nested intervals
        const float L0 = d_ref;
        const float U0 = d_ref;
        const float L1 = (r * f * lambda_min) / s_ref;
        const float U1 = (r * f * lambda_max) / s_ref;
        const float L2 = (r * f * lambda_min) / s_max;
        const float U2 = (r * f * lambda_max) / s_min;

        // Calculate the index terms based on intervals
        const float psi_i = 1.0f * pow((std::max)(0.0f, d - U0), 2) + 2.0f * pow((std::max)(0.0f, d - U1), 2) + 10.0f * pow((std::max)(0.0f, d - U2), 2);
        const float lambda_i = 0.0f; // Non-convex term is not considered

        const float mu_i = 1.0f;
        const float nu_i = 1.0f;
        const float gamma_i = mu_i * (x - p_ref).transpose() * (x - p_ref) + nu_i * pow((d - (x - z).transpose() * Vector3r(0.0f, 0.0f, 1.0f)), 2);

        // Accumulate into the total index
        return psi_i + lambda_i + gamma_i;
    }

    float PositionSolver::calculateWindowJ2(const int& i, const Vector3r& x, const FunctionData* data)
    {
        // Distance threshold to consider that xHat coincides with zi
        // Considering that hMin is several meters, it should not be reached unless set very high, since zi are at height 0
        float eps_dist = 0.1f;

        // Extract the necessary parameters
        const auto& tracking_params = data->params.tracking_params;
        const float& s_min = tracking_params.projection_params[i].s_min;
        const float& s_max = tracking_params.projection_params[i].s_max;
        const float& s_ref = tracking_params.projection_params[i].s_ref;
        const float& f = tracking_params.window_params[i].camera_params.f_ref;
        const float& lambda_min = tracking_params.window_params[i].lambda_min;
        const float& lambda_max = tracking_params.window_params[i].lambda_max;
        const float& lambda_ref = tracking_params.window_params[i].lambda_ref;

        // Target position and distance to its camera (approximated by distance to the vehicle)
        const Vector3r z = data->tab_P.col(i);
        const float d = (x - z).norm();

        // Vector indicating the direction to project
        const Vector3r v = data->x_hat - z;
        const float v_norm = v.norm();
        Vector3r eta = Vector3r::Zero();
        if (v_norm > eps_dist)
            eta = v / v_norm;

        // Calculate the projection as a substitute for distance for the non-convex term
        const float d_proj = (x - z).transpose() * eta;

        // Equivalent interest radius of the real target
        const float r = data->tab_r(i);

        // Calculate the reference distance to the target
        const float d_ref = (r * f * lambda_ref) / s_ref;

        // Calculate what would be the ideal reference position in the case of a single target (perfect verticallity)
        Vector3r p_ref = z;
        p_ref(2) += d_ref;

        // Determine the nested intervals
        const float L0 = d_ref;
        const float U0 = d_ref;
        const float L1 = (r * f * lambda_min) / s_ref;
        const float U1 = (r * f * lambda_max) / s_ref;
        const float L2 = (r * f * lambda_min) / s_max;
        const float U2 = (r * f * lambda_max) / s_min;

        // Calculate the index terms based on intervals
        const float alpha = 2.0f;
        const float psi_i = 1.0f * pow((std::max)(0.0f, d - U0), alpha) + 2.0f * pow((std::max)(0.0f, d - U1), alpha) + 10.0f * pow((std::max)(0.0f, d - U2), alpha);
        // Convex approximation of the non-convex term
        const float lambda_i = 1.0f * pow((std::max)(0.0f, L0 - d_proj), alpha) + 2.0f * pow((std::max)(0.0f, L1 - d_proj), alpha) + 10.0f * pow((std::max)(0.0f, L2 - d_proj), alpha);

        const float mu_i = 1.0f;
        const float nu_i = 1.0f;
        const float gamma_i = mu_i * (x - p_ref).transpose() * (x - p_ref) + nu_i * pow((d - (x - z).transpose() * Vector3r(0.0f, 0.0f, 1.0f)), 2);

        // Accumulate into the total index
        return psi_i + lambda_i + gamma_i;
    }

    // ════════════════════════════════════════════════════════════════════════════
    // IMPLEMENTATION: Optimization utility methods
    // ════════════════════════════════════════════════════════════════════════════

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