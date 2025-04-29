#pragma once

// Utilities
#include "flychams_core/types/core_types.hpp"

namespace flychams::coordination
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Cost functions for agent positioning
     *
     * @details
     * This class implements the cost functions for agent positioning.
     * It is used to calculate the cost of the agent position.
     *
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-04-17
     * ════════════════════════════════════════════════════════════════
     */
    class CostFunctions
    {
    public: // Types
        struct TrackingUnit // Tracking unit parameters
        {
            // Tracking mode
            core::TrackingMode mode;

            // Camera parameters
            float f_min = 0.0f;
            float f_max = 0.0f;
            float f_ref = 0.0f;

            // Window parameters
            float lambda_min = 0.0f;
            float lambda_max = 0.0f;
            float lambda_ref = 0.0f;
            float central_f = 0.0f;

            // Projection parameters
            float s_min = 0.0f;
            float s_max = 0.0f;
            float s_ref = 0.0f;

            // Cost function weights
            // Psi
            float tau0 = 1.0f;
            float tau1 = 2.0f;
            float tau2 = 10.0f;
            // Lambda
            float sigma0 = 1.0f;
            float sigma1 = 2.0f;
            float sigma2 = 10.0f;
            // Gamma
            float mu = 1.0f;
            float nu = 1.0f;
        };
        struct Parameters // Parameters for the cost function
        {
            int n;
            std::vector<TrackingUnit> units;
        };

    public: // Cost functions without gradient calculation
        static float J0(const core::Matrix3Xr& tab_P, const core::RowVectorXr& tab_r, const core::Vector3r& x, const Parameters& params)
        {
            // Compute the value of the optimization index based on nested intervals 
            // (original cost function, with non-convex term) based on tracking mode
            float J = 0.0f;
            for (int i = 0; i < params.n; i++)
            {
                // Get relevant data
                const auto& z = tab_P.col(i);
                const auto& r = tab_r(i);
                const auto& unit = params.units[i];

                // Compute the value of the index based on tracking mode
                switch (unit.mode)
                {
                case core::TrackingMode::MultiCamera:
                    J += CostFunctions::cameraJ0(z, r, x, unit);
                    break;

                case core::TrackingMode::MultiWindow:
                    J += CostFunctions::windowJ0(z, r, x, unit);
                    break;

                default:
                    throw std::invalid_argument("Invalid tracking mode");
                }
            }

            // Return the value of J
            return J;
        }

        static float J1(const core::Matrix3Xr& tab_P, const core::RowVectorXr& tab_r, const core::Vector3r& x, const Parameters& params)
        {
            // Compute the value of the optimization index based on nested intervals 
            // (without non-convex term) based on tracking mode
            float J = 0.0f;
            for (int i = 0; i < params.n; i++)
            {
                // Get relevant data
                const auto& z = tab_P.col(i);
                const auto& r = tab_r(i);
                const auto& unit = params.units[i];

                // Compute the value of the index based on tracking mode
                switch (unit.mode)
                {
                case core::TrackingMode::MultiCamera:
                    J += CostFunctions::cameraJ1(z, r, x, unit);
                    break;

                case core::TrackingMode::MultiWindow:
                    J += CostFunctions::windowJ1(z, r, x, unit);
                    break;

                default:
                    throw std::invalid_argument("Invalid tracking mode");
                }
            }

            // Return the value of J
            return J;
        }

        static float J2(const core::Matrix3Xr& tab_P, const core::RowVectorXr& tab_r, const core::Vector3r& x, const core::Vector3r& x_hat, const Parameters& params)
        {
            // Compute the value of the optimization index based on nested intervals 
            // (with convex relaxation of the non-convex term) based on tracking mode
            float J = 0.0f;
            for (int i = 0; i < params.n; i++)
            {
                // Get relevant data
                const auto& z = tab_P.col(i);
                const auto& r = tab_r(i);
                const auto& unit = params.units[i];

                // Compute the value of the index based on tracking mode
                switch (unit.mode)
                {
                case core::TrackingMode::MultiCamera:
                    J += CostFunctions::cameraJ2(z, r, x, x_hat, unit);
                    break;

                case core::TrackingMode::MultiWindow:
                    J += CostFunctions::windowJ2(z, r, x, x_hat, unit);
                    break;

                default:
                    throw std::invalid_argument("Invalid tracking mode");
                }
            }

            // Return the value of J
            return J;
        }

    public: // Cost functions with gradient calculation
        static float J1(const core::Matrix3Xr& tab_P, const core::RowVectorXr& tab_r, const core::Vector3r& x, const Parameters& params, core::Vector3r& grad)
        {
            // Initialize the gradient
            grad = core::Vector3r::Zero();

            // Compute the value of the optimization index based on nested intervals 
            // (without non-convex term) based on tracking mode
            float J = 0.0f;
            for (int i = 0; i < params.n; i++)
            {
                // Get relevant data
                const auto& z = tab_P.col(i);
                const auto& r = tab_r(i);
                const auto& unit = params.units[i];

                // Compute the value of the index based on tracking mode
                core::Vector3r grad_i;
                switch (unit.mode)
                {
                case core::TrackingMode::MultiCamera:
                    J += CostFunctions::cameraJ1(z, r, x, unit, grad_i);
                    break;

                case core::TrackingMode::MultiWindow:
                    J += CostFunctions::windowJ1(z, r, x, unit, grad_i);
                    break;

                default:
                    throw std::invalid_argument("Invalid tracking mode");
                }

                // Integrate the gradient
                grad += grad_i;
            }

            // Return the value of J
            return J;
        }

        static float J2(const core::Matrix3Xr& tab_P, const core::RowVectorXr& tab_r, const core::Vector3r& x, const core::Vector3r& x_hat, const Parameters& params, core::Vector3r& grad)
        {
            // Initialize the gradient
            grad = core::Vector3r::Zero();

            // Compute the value of the optimization index based on nested intervals 
            // (with convex relaxation of the non-convex term) based on tracking mode
            float J = 0.0f;
            for (int i = 0; i < params.n; i++)
            {
                // Get relevant data
                const auto& z = tab_P.col(i);
                const auto& r = tab_r(i);
                const auto& unit = params.units[i];

                // Compute the value of the index based on tracking mode
                core::Vector3r grad_i;
                switch (unit.mode)
                {
                case core::TrackingMode::MultiCamera:
                    J += CostFunctions::cameraJ2(z, r, x, x_hat, unit, grad_i);
                    break;

                case core::TrackingMode::MultiWindow:
                    J += CostFunctions::windowJ2(z, r, x, x_hat, unit, grad_i);
                    break;

                default:
                    throw std::invalid_argument("Invalid tracking mode");
                }

                // Integrate the gradient
                grad += grad_i;
            }

            // Return the value of J
            return J;
        }

    public: // Cost for single tracking unit without gradient calculation
        static float cameraJ0(const core::Vector3r& z, const float& r, const core::Vector3r& x, const TrackingUnit& params)
        {
            // Original cost function with non-convex term
            // Not valid for non-global optimization (e.g. Ellipsoid method, Nelder-Mead Simplex...)
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
            const auto& sigma0 = params.sigma0;
            const auto& sigma1 = params.sigma1;
            const auto& sigma2 = params.sigma2;
            const auto& mu = params.mu;
            const auto& nu = params.nu;

            // Target position and distance to its camera (approximated by distance to the vehicle)
            const float d = (x - z).norm();

            // Calculate the reference distance to the target
            const float d_ref = r * f_ref / s_ref;

            // Calculate what would be the ideal reference position in the case of a single target (perfect verticallity)
            core::Vector3r p_ref = z;
            p_ref(2) += d_ref;

            // Determine the nested intervals
            const float L0 = d_ref;
            const float U0 = d_ref;
            const float L1 = r * f_min / s_ref;
            const float U1 = r * f_max / s_ref;
            const float L2 = r * f_min / s_max;
            const float U2 = r * f_max / s_min;

            // Calculate the index terms based on intervals
            const float psi_i = 
                tau0 * pow(max(0.0f, d - U0), 2) + 
                tau1 * pow(max(0.0f, d - U1), 2) + 
                tau2 * pow(max(0.0f, d - U2), 2);
            const float lambda_i = 
                sigma0 * pow(max(0.0f, L0 - d), 2) + 
                sigma1 * pow(max(0.0f, L1 - d), 2) + 
                sigma2 * pow(max(0.0f, L2 - d), 2);
            const float gamma_i = 
                mu * (x - p_ref).transpose() * (x - p_ref) + 
                nu * pow((d - (x - z).transpose() * core::Vector3r(0.0f, 0.0f, 1.0f)), 2);
                            
            // Return the value of Ji
            return psi_i + lambda_i + gamma_i;
        }

        static float cameraJ1(const core::Vector3r& z, const float& r, const core::Vector3r& x, const TrackingUnit& params)
        {
            // Cost function without non-convex term
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
            core::Vector3r p_ref = z;
            p_ref(2) += d_ref;

            // Determine the nested intervals
            const float L0 = d_ref;
            const float U0 = d_ref;
            const float L1 = r * f_min / s_ref;
            const float U1 = r * f_max / s_ref;
            const float L2 = r * f_min / s_max;
            const float U2 = r * f_max / s_min;

            // Calculate the index terms based on intervals
            const float psi_i = 
                tau0 * pow(max(0.0f, d - U0), 2) + 
                tau1 * pow(max(0.0f, d - U1), 2) + 
                tau2 * pow(max(0.0f, d - U2), 2);
            // const float lambda_i = 0.0f; // Non-convex term is not considered
            const float gamma_i = 
                mu * (x - p_ref).transpose() * (x - p_ref) + 
                nu * pow((d - (x - z).transpose() * core::Vector3r(0.0f, 0.0f, 1.0f)), 2);

            // Return the value of Ji
            return psi_i + gamma_i;
        }

        static float cameraJ2(const core::Vector3r& z, const float& r, const core::Vector3r& x, const core::Vector3r& x_hat, const TrackingUnit& params)
        {
            // Cost function with convex relaxation of the non-convex term
            // Args:
            // z: center of the cluster
            // r: radius of the cluster
            // x: position of the vehicle
            // x_hat: estimated position of the vehicle
            // params: parameters for the cost function

            // Distance threshold to consider that x_hat coincides with zi
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
            const core::Vector3r v = x_hat - z;
            const float v_norm = v.norm();
            core::Vector3r eta = core::Vector3r::Zero();
            if (v_norm > eps_dist)
                eta = v / v_norm;

            // Calculate the projection as a substitute for distance for the non-convex term
            const float d_proj = (x - z).transpose() * eta;

            // Calculate the reference distance to the target
            const float d_ref = r * f_ref / s_ref;

            // Calculate what would be the ideal reference position in the case of a single target (perfect verticallity)
            core::Vector3r p_ref = z;
            p_ref(2) += d_ref;

            // Determine the nested intervals
            const float L0 = d_ref;
            const float U0 = d_ref;
            const float L1 = r * f_min / s_ref;
            const float U1 = r * f_max / s_ref;
            const float L2 = r * f_min / s_max;
            const float U2 = r * f_max / s_min;

            // Calculate the index terms based on intervals
            const float psi_i = 
                tau0 * pow(max(0.0f, d - U0), 2) + 
                tau1 * pow(max(0.0f, d - U1), 2) + 
                tau2 * pow(max(0.0f, d - U2), 2);
            const float lambda_i = 
                sigma0 * pow(max(0.0f, L0 - d_proj), 2) + 
                sigma1 * pow(max(0.0f, L1 - d_proj), 2) + 
                sigma2 * pow(max(0.0f, L2 - d_proj), 2);
            const float gamma_i = 
                mu * (x - p_ref).transpose() * (x - p_ref) + 
                nu * pow((d - (x - z).transpose() * core::Vector3r(0.0f, 0.0f, 1.0f)), 2);
                            
            // Return the value of Ji
            return psi_i + lambda_i + gamma_i;
        }

        static float windowJ0(const core::Vector3r& z, const float& r, const core::Vector3r& x, const TrackingUnit& params)
        {
            // Original cost function with non-convex term
            // Not valid for non-global optimization (e.g. Ellipsoid method, Nelder-Mead Simplex...)
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
            const auto& sigma0 = params.sigma0;
            const auto& sigma1 = params.sigma1;
            const auto& sigma2 = params.sigma2;
            const auto& mu = params.mu;
            const auto& nu = params.nu;

            // Target position and distance to its camera (approximated by distance to the vehicle)
            const float d = (x - z).norm();

            // Calculate the reference distance to the target
            const float d_ref = (r * f * lambda_ref) / s_ref;

            // Calculate what would be the ideal reference position in the case of a single target (perfect verticallity)
            core::Vector3r p_ref = z;
            p_ref(2) += d_ref;

            // Determine the nested intervals
            const float L0 = d_ref;
            const float U0 = d_ref;
            const float L1 = (r * f * lambda_min) / s_ref;
            const float U1 = (r * f * lambda_max) / s_ref;
            const float L2 = (r * f * lambda_min) / s_max;
            const float U2 = (r * f * lambda_max) / s_min;

            // Calculate the index terms based on intervals
            const float psi_i = 
                tau0 * pow(max(0.0f, d - U0), 2) + 
                tau1 * pow(max(0.0f, d - U1), 2) + 
                tau2 * pow(max(0.0f, d - U2), 2);
            const float lambda_i = 
                sigma0 * pow(max(0.0f, L0 - d), 2) + 
                sigma1 * pow(max(0.0f, L1 - d), 2) + 
                sigma2 * pow(max(0.0f, L2 - d), 2);
            const float gamma_i = 
                mu * (x - p_ref).transpose() * (x - p_ref) + 
                nu * pow((d - (x - z).transpose() * core::Vector3r(0.0f, 0.0f, 1.0f)), 2);

            // Return the value of Ji
            return psi_i + lambda_i + gamma_i;
        }

        static float windowJ1(const core::Vector3r& z, const float& r, const core::Vector3r& x, const TrackingUnit& params)
        {
            // Cost function without non-convex term
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
            core::Vector3r p_ref = z;
            p_ref(2) += d_ref;

            // Determine the nested intervals
            const float L0 = d_ref;
            const float U0 = d_ref;
            const float L1 = (r * f * lambda_min) / s_ref;
            const float U1 = (r * f * lambda_max) / s_ref;
            const float L2 = (r * f * lambda_min) / s_max;
            const float U2 = (r * f * lambda_max) / s_min;

            // Calculate the index terms based on intervals
            const float psi_i = 
                tau0 * pow(max(0.0f, d - U0), 2) + 
                tau1 * pow(max(0.0f, d - U1), 2) + 
                tau2 * pow(max(0.0f, d - U2), 2);
            // const float lambda_i = 0.0f; // Non-convex term is not considered
            const float gamma_i = 
                mu * (x - p_ref).transpose() * (x - p_ref) + 
                nu * pow((d - (x - z).transpose() * core::Vector3r(0.0f, 0.0f, 1.0f)), 2);

            // Return the value of Ji
            return psi_i + gamma_i;
        }

        static float windowJ2(const core::Vector3r& z, const float& r, const core::Vector3r& x, const core::Vector3r& x_hat, const TrackingUnit& params)
        {
            // Cost function with convex relaxation of the non-convex term
            // Args:
            // z: center of the cluster
            // r: radius of the cluster
            // x: position of the vehicle
            // x_hat: estimated position of the vehicle
            // params: parameters for the cost function

            // Distance threshold to consider that x_hat coincides with zi
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
            const core::Vector3r v = x_hat - z;
            const float v_norm = v.norm();
            core::Vector3r eta = core::Vector3r::Zero();
            if (v_norm > eps_dist)
                eta = v / v_norm;

            // Calculate the projection as a substitute for distance for the non-convex term
            const float d_proj = (x - z).transpose() * eta;

            // Calculate the reference distance to the target
            const float d_ref = (r * f * lambda_ref) / s_ref;

            // Calculate what would be the ideal reference position in the case of a single target (perfect verticallity)
            core::Vector3r p_ref = z;
            p_ref(2) += d_ref;

            // Determine the nested intervals
            const float L0 = d_ref;
            const float U0 = d_ref;
            const float L1 = (r * f * lambda_min) / s_ref;
            const float U1 = (r * f * lambda_max) / s_ref;
            const float L2 = (r * f * lambda_min) / s_max;
            const float U2 = (r * f * lambda_max) / s_min;

            // Calculate the index terms based on intervals
            const float psi_i = 
                tau0 * pow(max(0.0f, d - U0), 2) + 
                tau1 * pow(max(0.0f, d - U1), 2) + 
                tau2 * pow(max(0.0f, d - U2), 2);
            const float lambda_i = 
                sigma0 * pow(max(0.0f, L0 - d_proj), 2) + 
                sigma1 * pow(max(0.0f, L1 - d_proj), 2) + 
                sigma2 * pow(max(0.0f, L2 - d_proj), 2);
            const float gamma_i = 
                mu * (x - p_ref).transpose() * (x - p_ref) + 
                nu * pow((d - (x - z).transpose() * core::Vector3r(0.0f, 0.0f, 1.0f)), 2);

            // Return the value of Ji
            return psi_i + lambda_i + gamma_i;
        }

    public: // Cost for single tracking unit with gradient calculation
        static float cameraJ1(const core::Vector3r& z, const float& r, const core::Vector3r& x, const TrackingUnit& params, core::Vector3r& grad)
        {
            // Cost function without non-convex term
            // Args:
            // z: center of the cluster
            // r: radius of the cluster
            // x: position of the vehicle
            // params: parameters for the cost function
            // grad: gradient of the cost function to be computed

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

            // Initialize the gradient
            grad = core::Vector3r::Zero();

            // Target position and distance to its camera (approximated by distance to the vehicle)
            const float d = (x - z).norm();

            // Vector indicating the direction to project
            const core::Vector3r v_rho = x - z;
            const float v_rho_norm = v_rho.norm();
            core::Vector3r rho = core::Vector3r::Zero();
            if (v_rho_norm > 0.001f)
                rho = v_rho / v_rho_norm;

            // Calculate the reference distance to the target
            const float d_ref = r * f_ref / s_ref;

            // Calculate what would be the ideal reference position in the case of a single target (perfect verticallity)
            core::Vector3r p_ref = z;
            p_ref(2) += d_ref;

            // Determine the nested intervals
            const float L0 = d_ref;
            const float U0 = d_ref;
            const float L1 = r * f_min / s_ref;
            const float U1 = r * f_max / s_ref;
            const float L2 = r * f_min / s_max;
            const float U2 = r * f_max / s_min;

            // Calculate the index terms based on intervals
            const float psi_i = 
                tau0 * pow(max(0.0f, d - U0), 2) + 
                tau1 * pow(max(0.0f, d - U1), 2) + 
                tau2 * pow(max(0.0f, d - U2), 2);
            // const float lambda_i = 0.0f; // Non-convex term is not considered
            const float gamma_i = 
                mu * (x - p_ref).transpose() * (x - p_ref) + 
                nu * pow((d - (x - z).transpose() * core::Vector3r(0.0f, 0.0f, 1.0f)), 2);

            // Compute the gradient of the cost function
            // Psi gradient
            float grad_psi = 2.0f *
                    (tau0 * max(0.0f, d - U0) * heaviside(d, U0) + 
                    tau1 * max(0.0f, d - U1) * heaviside(d, U1) + 
                    tau2 * max(0.0f, d - U2) * heaviside(d, U2));
            // Gamma gradient
            core::Vector3r grad_gamma;
            grad_gamma(0) = 2.0f * (x(0) - p_ref(0)) + 2.0f * (d - (x(2) - z(2))) * (x(0) - z(0)) / d;
            grad_gamma(1) = 2.0f * (x(1) - p_ref(1)) + 2.0f * (d - (x(2) - z(2))) * (x(1) - z(1)) / d;
            grad_gamma(2) = 2.0f * (x(2) - p_ref(2)) + 2.0f * (d - (x(2) - z(2))) * ((x(2) - z(2)) / d - 1.0f);
            // Integrate all the gradients
            grad += grad_psi * rho + grad_gamma;

            // Return the value of Ji
            return psi_i + gamma_i;
        }

        static float cameraJ2(const core::Vector3r& z, const float& r, const core::Vector3r& x, const core::Vector3r& x_hat, const TrackingUnit& params, core::Vector3r& grad)
        {
            // Cost function with convex relaxation of the non-convex term
            // Args:
            // z: center of the cluster
            // r: radius of the cluster
            // x: position of the vehicle
            // x_hat: estimated position of the vehicle
            // params: parameters for the cost function
            // grad: gradient of the cost function to be computed

            // Distance threshold to consider that x_hat coincides with zi
            // Considering that hMin is several meters, it should not be reached unless set very high, since zi are at height 0
            float eps_dist = 0.1f;

            // Initialize the gradient
            grad = core::Vector3r::Zero();

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
            const core::Vector3r v = x_hat - z;
            const float v_norm = v.norm();
            core::Vector3r eta = core::Vector3r::Zero();
            if (v_norm > eps_dist)
                eta = v / v_norm;

            const core::Vector3r v_rho = x - z;
            const float v_rho_norm = v_rho.norm();
            core::Vector3r rho = core::Vector3r::Zero();
            if (v_rho_norm > 0.001f)
                rho = v_rho / v_rho_norm;

            // Calculate the projection as a substitute for distance for the non-convex term
            const float d_proj = (x - z).transpose() * eta;

            // Calculate the reference distance to the target
            const float d_ref = r * f_ref / s_ref;

            // Calculate what would be the ideal reference position in the case of a single target (perfect verticallity)
            core::Vector3r p_ref = z;
            p_ref(2) += d_ref;

            // Determine the nested intervals
            const float L0 = d_ref;
            const float U0 = d_ref;
            const float L1 = r * f_min / s_ref;
            const float U1 = r * f_max / s_ref;
            const float L2 = r * f_min / s_max;
            const float U2 = r * f_max / s_min;

            // Calculate the index terms based on intervals
            const float psi_i = 
                tau0 * pow(max(0.0f, d - U0), 2) + 
                tau1 * pow(max(0.0f, d - U1), 2) + 
                tau2 * pow(max(0.0f, d - U2), 2);
            const float lambda_i = 
                sigma0 * pow(max(0.0f, L0 - d_proj), 2) + 
                sigma1 * pow(max(0.0f, L1 - d_proj), 2) + 
                sigma2 * pow(max(0.0f, L2 - d_proj), 2);
            const float gamma_i = 
                mu * (x - p_ref).transpose() * (x - p_ref) + 
                nu * pow((d - (x - z).transpose() * core::Vector3r(0.0f, 0.0f, 1.0f)), 2);

            // Compute the gradient of the cost function
            // Psi gradient
            float grad_psi = 2.0f *
                    (tau0 * max(0.0f, d - U0) * heaviside(d, U0) + 
                    tau1 * max(0.0f, d - U1) * heaviside(d, U1) + 
                    tau2 * max(0.0f, d - U2) * heaviside(d, U2));
            // Lambda gradient
            float grad_lambda = -2.0f *
                    (sigma0 * max(0.0f, L0 - d_proj) * heaviside(L0, d_proj) + 
                    sigma1 * max(0.0f, L1 - d_proj) * heaviside(L1, d_proj) + 
                    sigma2 * max(0.0f, L2 - d_proj) * heaviside(L2, d_proj));
            // Gamma gradient
            core::Vector3r grad_gamma;
            grad_gamma(0) = 2.0f * (x(0) - p_ref(0)) + 2.0f * (d - (x(2) - z(2))) * (x(0) - z(0)) / d;
            grad_gamma(1) = 2.0f * (x(1) - p_ref(1)) + 2.0f * (d - (x(2) - z(2))) * (x(1) - z(1)) / d;
            grad_gamma(2) = 2.0f * (x(2) - p_ref(2)) + 2.0f * (d - (x(2) - z(2))) * ((x(2) - z(2)) / d - 1.0f);
            // Integrate all the gradients
            grad += grad_psi * rho + grad_lambda * eta + grad_gamma;

            // Return the value of Ji
            return psi_i + lambda_i + gamma_i;
        }

        static float windowJ1(const core::Vector3r& z, const float& r, const core::Vector3r& x, const TrackingUnit& params, core::Vector3r& grad)
        {
            // Cost function without non-convex term
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

            // Initialize the gradient
            grad = core::Vector3r::Zero();

            // Target position and distance to its camera (approximated by distance to the vehicle)
            const float d = (x - z).norm();

            // Vector indicating the direction to project
            const core::Vector3r v_rho = x - z;
            const float v_rho_norm = v_rho.norm();
            core::Vector3r rho = core::Vector3r::Zero();
            if (v_rho_norm > 0.001f)
                rho = v_rho / v_rho_norm;

            // Calculate the reference distance to the target
            const float d_ref = (r * f * lambda_ref) / s_ref;

            // Calculate what would be the ideal reference position in the case of a single target (perfect verticallity)
            core::Vector3r p_ref = z;
            p_ref(2) += d_ref;

            // Determine the nested intervals
            const float L0 = d_ref;
            const float U0 = d_ref;
            const float L1 = (r * f * lambda_min) / s_ref;
            const float U1 = (r * f * lambda_max) / s_ref;
            const float L2 = (r * f * lambda_min) / s_max;
            const float U2 = (r * f * lambda_max) / s_min;

            // Calculate the index terms based on intervals
            const float psi_i = 
                tau0 * pow(max(0.0f, d - U0), 2) + 
                tau1 * pow(max(0.0f, d - U1), 2) + 
                tau2 * pow(max(0.0f, d - U2), 2);
            // const float lambda_i = 0.0f; // Non-convex term is not considered
            const float gamma_i = 
                mu * (x - p_ref).transpose() * (x - p_ref) + 
                nu * pow((d - (x - z).transpose() * core::Vector3r(0.0f, 0.0f, 1.0f)), 2);

            // Compute the gradient of the cost function
            // Psi gradient
            float grad_psi = 2.0f *
                    (tau0 * max(0.0f, d - U0) * heaviside(d, U0) + 
                    tau1 * max(0.0f, d - U1) * heaviside(d, U1) + 
                    tau2 * max(0.0f, d - U2) * heaviside(d, U2));
            // Gamma gradient
            core::Vector3r grad_gamma;
            grad_gamma(0) = 2.0f * (x(0) - p_ref(0)) + 2.0f * (d - (x(2) - z(2))) * (x(0) - z(0)) / d;
            grad_gamma(1) = 2.0f * (x(1) - p_ref(1)) + 2.0f * (d - (x(2) - z(2))) * (x(1) - z(1)) / d;
            grad_gamma(2) = 2.0f * (x(2) - p_ref(2)) + 2.0f * (d - (x(2) - z(2))) * ((x(2) - z(2)) / d - 1.0f);
            // Integrate all the gradients
            grad += grad_psi * rho + grad_gamma;

            // Return the value of Ji
            return psi_i + gamma_i;
        }

        static float windowJ2(const core::Vector3r& z, const float& r, const core::Vector3r& x, const core::Vector3r& x_hat, const TrackingUnit& params, core::Vector3r& grad)
        {
            // Cost function with convex relaxation of the non-convex term
            // Args:
            // z: center of the cluster
            // r: radius of the cluster
            // x: position of the vehicle
            // x_hat: estimated position of the vehicle
            // params: parameters for the cost function

            // Distance threshold to consider that x_hat coincides with zi
            // Considering that hMin is several meters, it should not be reached unless set very high, since zi are at height 0
            float eps_dist = 0.1f;

            // Initialize the gradient
            grad = core::Vector3r::Zero();

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
            const core::Vector3r v = x_hat - z;
            const float v_norm = v.norm();
            core::Vector3r eta = core::Vector3r::Zero();
            if (v_norm > eps_dist)
                eta = v / v_norm;

            const core::Vector3r v_rho = x - z;
            const float v_rho_norm = v_rho.norm();
            core::Vector3r rho = core::Vector3r::Zero();
            if (v_rho_norm > 0.001f)
                rho = v_rho / v_rho_norm;

            // Calculate the projection as a substitute for distance for the non-convex term
            const float d_proj = (x - z).transpose() * eta;

            // Calculate the reference distance to the target
            const float d_ref = (r * f * lambda_ref) / s_ref;

            // Calculate what would be the ideal reference position in the case of a single target (perfect verticallity)
            core::Vector3r p_ref = z;
            p_ref(2) += d_ref;

            // Determine the nested intervals
            const float L0 = d_ref;
            const float U0 = d_ref;
            const float L1 = (r * f * lambda_min) / s_ref;
            const float U1 = (r * f * lambda_max) / s_ref;
            const float L2 = (r * f * lambda_min) / s_max;
            const float U2 = (r * f * lambda_max) / s_min;

            // Calculate the index terms based on intervals
            const float psi_i = 
                tau0 * pow(max(0.0f, d - U0), 2) + 
                tau1 * pow(max(0.0f, d - U1), 2) + 
                tau2 * pow(max(0.0f, d - U2), 2);
            const float lambda_i = 
                sigma0 * pow(max(0.0f, L0 - d_proj), 2) + 
                sigma1 * pow(max(0.0f, L1 - d_proj), 2) + 
                sigma2 * pow(max(0.0f, L2 - d_proj), 2);
            const float gamma_i = 
                mu * (x - p_ref).transpose() * (x - p_ref) + 
                nu * pow((d - (x - z).transpose() * core::Vector3r(0.0f, 0.0f, 1.0f)), 2);

            // Compute the gradient of the cost function
            // Psi gradient
            float grad_psi = 2.0f *
                    (tau0 * max(0.0f, d - U0) * heaviside(d, U0) + 
                    tau1 * max(0.0f, d - U1) * heaviside(d, U1) + 
                    tau2 * max(0.0f, d - U2) * heaviside(d, U2));
            // Lambda gradient
            float grad_lambda = -2.0f *
                    (sigma0 * max(0.0f, L0 - d_proj) * heaviside(L0, d_proj) + 
                    sigma1 * max(0.0f, L1 - d_proj) * heaviside(L1, d_proj) + 
                    sigma2 * max(0.0f, L2 - d_proj) * heaviside(L2, d_proj));
            // Gamma gradient
            core::Vector3r grad_gamma;
            grad_gamma(0) = 2.0f * (x(0) - p_ref(0)) + 2.0f * (d - (x(2) - z(2))) * (x(0) - z(0)) / d;
            grad_gamma(1) = 2.0f * (x(1) - p_ref(1)) + 2.0f * (d - (x(2) - z(2))) * (x(1) - z(1)) / d;
            grad_gamma(2) = 2.0f * (x(2) - p_ref(2)) + 2.0f * (d - (x(2) - z(2))) * ((x(2) - z(2)) / d - 1.0f);
            // Integrate all the gradients
            grad += grad_psi * rho + grad_lambda * eta + grad_gamma;

            // Return the value of Ji
            return psi_i + lambda_i + gamma_i;
        }

    private: // Utils
        static float max(const float& a, const float& b) 
        { 
            return std::max(a, b);
        }

        static float heaviside(const float& d, const float& U)
        {
            if (d < U) 
                return 0.0f;
            else if (d > U) 
                return 1.0f;
            else 
                return 0.5f;
        }
    };

} // namespace flychams::coordination