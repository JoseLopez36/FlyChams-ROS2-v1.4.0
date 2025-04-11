#include "flychams_coordination/positioning/position_solver.hpp"

using namespace flychams::core;

namespace flychams::coordination
{
    // ════════════════════════════════════════════════════════════════════════════
    // PUBLIC METHODS: Public methods for configuration and control
    // ════════════════════════════════════════════════════════════════════════════

    void PositionSolver::init()
    {
        // Create an NLopt optimizer
        opt_ = nlopt_create(NLOPT_LN_NELDERMEAD, 3); // 3 is the dimension of the problem

        // Optimizer options
        nlopt_set_xtol_rel(opt_, static_cast<double>(tol_)); // Set convergence tolerance
        nlopt_set_maxeval(opt_, max_iter_);                  // Maximum number of function evaluations
    }

    void PositionSolver::destroy()
    {
        if (opt_)
        {
            nlopt_destroy(opt_);
        }
    }

    void PositionSolver::reset()
    {
        // Nothing to do
    }

    void PositionSolver::setMode(const SolverMode& mode)
    {
        mode_ = mode;
    }

    void PositionSolver::setParameters(const float& tol, const int& max_iter, const float& eps)
    {
        // Check parameters
        if (tol <= 0.0f || max_iter <= 0 || eps <= 0.0f)
        {
            throw std::invalid_argument("Invalid parameters");
        }

        // Set parameters
        tol_ = tol;
        max_iter_ = max_iter;
        eps_ = eps;
    }

    Vector3r PositionSolver::run(const Matrix3Xr& tab_P, const RowVectorXr& tab_r, const Vector3r& x0,
        const float& min_h, const float& max_h, const CostParameters& central_params, const std::vector<CostParameters>& tracking_params)
    {
        // Check if NLopt optimizer is initialized
        if (!opt_)
        {
            throw std::runtime_error("NLopt optimizer not initialized");
        }

        // Define the optimization bounds
        const double lb[3] = { -HUGE_VAL, -HUGE_VAL, static_cast<double>(min_h) };
        const double ub[3] = { HUGE_VAL, HUGE_VAL, static_cast<double>(max_h) };
        nlopt_set_lower_bounds(opt_, lb);
        nlopt_set_upper_bounds(opt_, ub);

        // Clip height to limits
        Vector3r x0_clipped = x0;
        x0_clipped(2) = std::min(std::max(x0(2), min_h), max_h);

        // Create data struct with the number of clusters
        int num_clusters = static_cast<int>(tab_P.cols());
        CostData data(num_clusters);

        // Fill data struct
        data.n = num_clusters;
        data.tab_P = tab_P;
        data.tab_r = tab_r;
        data.xHat = Vector3r::Zero();
        data.central_params = central_params;
        data.tracking_params = tracking_params;

        // First optimization (solve for initial position)
        Vector3r xOpt_1 = preOptimization(x0_clipped, data);

        // Iterative optimization (solve for optimal position)
        return iterativeOptimization(xOpt_1, data);
    }

    // ════════════════════════════════════════════════════════════════════════════
    // IMPLEMENTATION: Optimization methods
    // ════════════════════════════════════════════════════════════════════════════

    Vector3r PositionSolver::preOptimization(const Vector3r& x0, CostData& data)
    {
        // Set the objective cost function J1 along with the data
        nlopt_set_min_objective(opt_, funJ1, &data);

        // Optimize for J1
        Vector3r xOpt = x0; // Start from initial position
        optimize(xOpt);

        return xOpt;
    }

    Vector3r PositionSolver::iterativeOptimization(const Vector3r& x0, CostData& data)
    {
        // Initialize with the previous solution
        Vector3r xOpt_prev = x0;
        Vector3r xOpt = xOpt_prev;

        // Iteratively call the optimization algorithm that implements the convex relaxation (J2)
        float xDiff = HUGE_VALF;
        while (xDiff > eps_)
        {
            // Define the xHat parameter of cost function J2
            data.xHat << xOpt_prev[0], xOpt_prev[1], xOpt_prev[2];

            // Set the objective cost function J2 along with the data
            nlopt_set_min_objective(opt_, funJ2, &data);

            // Optimize for J2
            optimize(xOpt);

            // Compute the norm of difference
            xDiff = (xOpt - xOpt_prev).norm();

            // Update the previous solution
            xOpt_prev = xOpt;
        }

        return xOpt;
    }

    // ════════════════════════════════════════════════════════════════════════════
    // IMPLEMENTATION: Objective functions
    // ════════════════════════════════════════════════════════════════════════════

    double PositionSolver::funJ1(unsigned n, const double* x, double* grad, void* data)
    {
        // Extract data
        CostData* cost_data = reinterpret_cast<CostData*>(data);
        Vector3r xVec(static_cast<float>(x[0]), static_cast<float>(x[1]), static_cast<float>(x[2]));

        // Compute the value of the optimization index based on nested intervals (without non-convex term) based on tracking mode
        float J1 = 0.0f;
        for (int i = 0; i < cost_data->n; i++)
        {
            // Get relevant data
            const auto& z = cost_data->tab_P.col(i);
            const auto& r = cost_data->tab_r(i);
            const auto& params = cost_data->tracking_params[i];

            // Compute the value of the index based on tracking mode
            switch (params.mode)
            {
            case TrackingMode::MultiCamera:
                J1 += calculateCameraJ1(z, r, xVec, params);
                break;

            case TrackingMode::MultiWindow:
                J1 += calculateWindowJ1(z, r, xVec, params);
                break;

            default:
                throw std::invalid_argument("Invalid tracking mode");
            }
        }

        // Account for the central window cost to ensure targets are inside the bounds of the central window
        // We use the mean of the centers and the largest possible radius
        Vector3r z_mean = Vector3r::Zero();
        for (int i = 0; i < cost_data->n; i++)
        {
            z_mean += cost_data->tab_P.col(i);
        }
        z_mean /= static_cast<float>(cost_data->n);
        float r_max = 0.0f;
        for (int i = 0; i < cost_data->n; i++)
        {
            r_max = std::max(r_max, (z_mean - cost_data->tab_P.col(i)).norm() + cost_data->tab_r(i));
        }
        J1 += calculateWindowJ1(z_mean, r_max, xVec, cost_data->central_params);

        // Return the value of J1
        return static_cast<double>(J1);
    }

    double PositionSolver::funJ2(unsigned n, const double* x, double* grad, void* data)
    {
        // Extract data
        CostData* cost_data = reinterpret_cast<CostData*>(data);
        Vector3r xVec(static_cast<float>(x[0]), static_cast<float>(x[1]), static_cast<float>(x[2]));

        // Compute the value of the optimization index based on nested intervals (with non-convex term) based on tracking mode
        float J2 = 0.0f;
        for (int i = 0; i < cost_data->n; i++)
        {
            // Get relevant data
            const auto& z = cost_data->tab_P.col(i);
            const auto& r = cost_data->tab_r(i);
            const auto& xHat = cost_data->xHat;
            const auto& params = cost_data->tracking_params[i];

            // Compute the value of the index based on tracking mode
            switch (params.mode)
            {
            case TrackingMode::MultiCamera:
                J2 += calculateCameraJ2(z, r, xVec, xHat, params);
                break;

            case TrackingMode::MultiWindow:
                J2 += calculateWindowJ2(z, r, xVec, xHat, params);
                break;

            default:
                throw std::invalid_argument("Invalid tracking mode");
            }
        }

        // Account for the central window cost to ensure targets are inside the bounds of the central window
        // We use the mean of the centers and the largest possible radius
        Vector3r z_mean = Vector3r::Zero();
        for (int i = 0; i < cost_data->n; i++)
        {
            z_mean += cost_data->tab_P.col(i);
        }
        z_mean /= static_cast<float>(cost_data->n);
        float r_max = 0.0f;
        for (int i = 0; i < cost_data->n; i++)
        {
            r_max = std::max(r_max, (z_mean - cost_data->tab_P.col(i)).norm() + cost_data->tab_r(i));
        }
        J2 += calculateWindowJ2(z_mean, r_max, xVec, cost_data->xHat, cost_data->central_params);

        // Return the value of J2
        return static_cast<double>(J2);
    }

    // ════════════════════════════════════════════════════════════════════════════
    // IMPLEMENTATION: Cost functions
    // ════════════════════════════════════════════════════════════════════════════

    float PositionSolver::calculateCameraJ1(const Vector3r& z, const float& r, const Vector3r& x, const CostParameters& params)
    {
        // Args:
        // z: center of the cluster
        // r: radius of the cluster
        // x: position of the vehicle
        // params: parameters for the cost function

        // Extract cost function parameters
        const auto& s_min = params.s_min;
        const auto& s_max = params.s_max;
        const auto& s_ref = params.s_ref;
        const auto& f_min = params.f_min;
        const auto& f_max = params.f_max;
        const auto& f_ref = params.f_ref;
        const auto& tau0 = params.tau0;
        const auto& tau1 = params.tau1;
        const auto& tau2 = params.tau2;
        const auto& mu = params.mu;
        const auto& nu = params.nu;

        // Target position and distance to its camera (approximated by distance to the vehicle)
        const float d = (x - z).norm();

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
        const float psi_i = tau0 * pow((std::max)(0.0f, d - U0), 2) + tau1 * pow((std::max)(0.0f, d - U1), 2) + tau2 * pow((std::max)(0.0f, d - U2), 2);
        const float lambda_i = 0.0f; // Non-convex term is not considered
        const float gamma_i = mu * (x - p_ref).transpose() * (x - p_ref) + nu * pow((d - (x - z).transpose() * Vector3r(0.0f, 0.0f, 1.0f)), 2);

        // Return the value of Ji
        return psi_i + lambda_i + gamma_i;
    }

    float PositionSolver::calculateCameraJ2(const Vector3r& z, const float& r, const Vector3r& x, const Vector3r& xHat, const CostParameters& params)
    {
        // Args:
        // z: center of the cluster
        // r: radius of the cluster
        // x: position of the vehicle
        // xHat: estimated position of the vehicle
        // params: parameters for the cost function

        // Distance threshold to consider that xHat coincides with zi
        // Considering that hMin is several meters, it should not be reached unless set very high, since zi are at height 0
        float eps_dist = 0.1f;

        // Extract cost function parameters
        const auto& s_min = params.s_min;
        const auto& s_max = params.s_max;
        const auto& s_ref = params.s_ref;
        const auto& f_min = params.f_min;
        const auto& f_max = params.f_max;
        const auto& f_ref = params.f_ref;
        const auto& tau0 = params.tau0;
        const auto& tau1 = params.tau1;
        const auto& tau2 = params.tau2;
        const auto& sigma0 = params.sigma0;
        const auto& sigma1 = params.sigma1;
        const auto& sigma2 = params.sigma2;
        const auto& mu = params.mu;
        const auto& nu = params.nu;

        // Target position and distance to its camera (approximated by distance to the vehicle)
        const float d = (x - z).norm();

        // Vector indicating the direction to project
        const Vector3r v = xHat - z;
        const float v_norm = v.norm();
        Vector3r eta = Vector3r::Zero();
        if (v_norm > eps_dist)
            eta = v / v_norm;

        // Calculate the projection as a substitute for distance for the non-convex term
        const float d_proj = (x - z).transpose() * eta;

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
        const float psi_i = tau0 * pow((std::max)(0.0f, d - U0), alpha) + tau1 * pow((std::max)(0.0f, d - U1), alpha) + tau2 * pow((std::max)(0.0f, d - U2), alpha);
        const float lambda_i = sigma0 * pow((std::max)(0.0f, L0 - d_proj), alpha) + sigma1 * pow((std::max)(0.0f, L1 - d_proj), alpha) + sigma2 * pow((std::max)(0.0f, L2 - d_proj), alpha);
        const float gamma_i = mu * (x - p_ref).transpose() * (x - p_ref) + nu * pow((d - (x - z).transpose() * Vector3r(0.0f, 0.0f, 1.0f)), 2);

        // Return the value of Ji
        return psi_i + lambda_i + gamma_i;
    }

    float PositionSolver::calculateWindowJ1(const Vector3r& z, const float& r, const Vector3r& x, const CostParameters& params)
    {
        // Args:
        // z: center of the cluster
        // r: radius of the cluster
        // x: position of the vehicle
        // params: parameters for the cost function

        // Extract cost function parameters
        const auto& s_min = params.s_min;
        const auto& s_max = params.s_max;
        const auto& s_ref = params.s_ref;
        const auto& lambda_min = params.lambda_min;
        const auto& lambda_max = params.lambda_max;
        const auto& lambda_ref = params.lambda_ref;
        const auto& f = params.central_f;
        const auto& tau0 = params.tau0;
        const auto& tau1 = params.tau1;
        const auto& tau2 = params.tau2;
        const auto& mu = params.mu;
        const auto& nu = params.nu;

        // Target position and distance to its camera (approximated by distance to the vehicle)
        const float d = (x - z).norm();

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
        const float psi_i = tau0 * pow((std::max)(0.0f, d - U0), 2) + tau1 * pow((std::max)(0.0f, d - U1), 2) + tau2 * pow((std::max)(0.0f, d - U2), 2);
        const float lambda_i = 0.0f; // Non-convex term is not considered
        const float gamma_i = mu * (x - p_ref).transpose() * (x - p_ref) + nu * pow((d - (x - z).transpose() * Vector3r(0.0f, 0.0f, 1.0f)), 2);

        // Return the value of Ji
        return psi_i + lambda_i + gamma_i;
    }

    float PositionSolver::calculateWindowJ2(const Vector3r& z, const float& r, const Vector3r& x, const Vector3r& xHat, const CostParameters& params)
    {
        // Args:
        // z: center of the cluster
        // r: radius of the cluster
        // x: position of the vehicle
        // xHat: estimated position of the vehicle
        // params: parameters for the cost function

        // Distance threshold to consider that xHat coincides with zi
        // Considering that hMin is several meters, it should not be reached unless set very high, since zi are at height 0
        float eps_dist = 0.1f;

        // Extract cost function parameters
        const auto& s_min = params.s_min;
        const auto& s_max = params.s_max;
        const auto& s_ref = params.s_ref;
        const auto& lambda_min = params.lambda_min;
        const auto& lambda_max = params.lambda_max;
        const auto& lambda_ref = params.lambda_ref;
        const auto& f = params.central_f;
        const auto& tau0 = params.tau0;
        const auto& tau1 = params.tau1;
        const auto& tau2 = params.tau2;
        const auto& sigma0 = params.sigma0;
        const auto& sigma1 = params.sigma1;
        const auto& sigma2 = params.sigma2;
        const auto& mu = params.mu;
        const auto& nu = params.nu;

        // Target position and distance to its camera (approximated by distance to the vehicle)
        const float d = (x - z).norm();

        // Vector indicating the direction to project
        const Vector3r v = xHat - z;
        const float v_norm = v.norm();
        Vector3r eta = Vector3r::Zero();
        if (v_norm > eps_dist)
            eta = v / v_norm;

        // Calculate the projection as a substitute for distance for the non-convex term
        const float d_proj = (x - z).transpose() * eta;

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
        const float psi_i = tau0 * pow((std::max)(0.0f, d - U0), alpha) + tau1 * pow((std::max)(0.0f, d - U1), alpha) + tau2 * pow((std::max)(0.0f, d - U2), alpha);
        const float lambda_i = sigma0 * pow((std::max)(0.0f, L0 - d_proj), alpha) + sigma1 * pow((std::max)(0.0f, L1 - d_proj), alpha) + sigma2 * pow((std::max)(0.0f, L2 - d_proj), alpha);
        const float gamma_i = mu * (x - p_ref).transpose() * (x - p_ref) + nu * pow((d - (x - z).transpose() * Vector3r(0.0f, 0.0f, 1.0f)), 2);

        // Return the value of Ji
        return psi_i + lambda_i + gamma_i;
    }

    // ════════════════════════════════════════════════════════════════════════════
    // IMPLEMENTATION: Optimization utility methods
    // ════════════════════════════════════════════════════════════════════════════

    float PositionSolver::optimize(Vector3r& xOpt)
    {
        double xOpt_nlopt[3] = { static_cast<double>(xOpt(0)), static_cast<double>(xOpt(1)), static_cast<double>(xOpt(2)) };

        // Call the optimization algorithm
        // J: optimal value of the cost function
        // xOpt: value that minimizes the cost function
        double J;
        nlopt_optimize(opt_, xOpt_nlopt, &J);

        // Update the position
        xOpt << static_cast<float>(xOpt_nlopt[0]), static_cast<float>(xOpt_nlopt[1]), static_cast<float>(xOpt_nlopt[2]);

        // Return the optimal value of the cost function
        return static_cast<float>(J);
    }

} // namespace flychams::coordination