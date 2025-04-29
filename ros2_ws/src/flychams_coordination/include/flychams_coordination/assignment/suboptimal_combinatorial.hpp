#pragma once

// Standard includes
#include <vector>
#include <algorithm>
#include <iostream>
#include <functional>

// Position solver
#include "flychams_coordination/positioning/position_solver.hpp"

// Utilities
#include "flychams_core/types/core_types.hpp"

namespace flychams::coordination
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Solver for agent assignment using sub-optimal combinatorial 
     * optimization
     *
     * @details
     * This class implements a sub-optimal combinatorial optimization
     * algorithm for solving the agent assignment problem.
     *
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-04-22
     * ════════════════════════════════════════════════════════════════
     */
    class SuboptimalCombinatorial
    {
    public: // Types
        // Parameters
        struct Parameters
        {
            // Optimization weights
            float observation_weight;
            float distance_weight;
            float switch_weight;
        };
        // Data
        struct Agent
        {
            int k;
            core::Vector3r x;       // Current position
            core::RowVectorXi X;    // Current assignment
            core::VectorXr J_hist;  // Assignment cost history
            core::MatrixXi X_hist;  // Assignment history
        };
        struct Cluster
        {
            int i;
            core::Vector3r C;
            float r;
        };
    
    private: // Parameters
        Parameters params_;

    public: // Public methods
        void init(const Parameters& params)
        {
            // Store parameters
            params_ = params;
        }
        void destroy()
        {
            // Nothing to destroy
        }
        core::RowVectorXi run(const core::Matrix3Xr& tab_x, const core::Matrix3Xr& tab_P, const core::RowVectorXr& tab_r, 
            const core::RowVectorXi& X_prev, std::vector<PositionSolver::SharedPtr>& solvers)
        {
            // Get number of agents and tracking units
            int m = tab_x.cols();
            core::RowVectorXi nk = core::RowVectorXi::Zero(m);
            for (int k = 0; k < m; k++)
            {
                nk(k) = solvers[k]->n();
            }

            // Create agent vector
            std::vector<Agent> A(m);
            int t = 0;
            for (int k = 0; k < m; k++)
            {
                A[k].k = k;
                A[k].x = tab_x.col(k);
                A[k].X.resize(nk(k));
                for (int u = 0; u < nk(k); u++)
                {
                    A[k].X(u) = X_prev(t);
                    t++;
                }
                A[k].J_hist = core::VectorXr::Zero(0);
                A[k].X_hist = core::MatrixXi::Zero(0, nk(k));
            }
            
            // Create cluster vector
            int n = tab_P.cols();
            std::vector<Cluster> T(n);
            for (int i = 0; i < n; i++)
            {
                T[i].i = i;
                T[i].C = tab_P.col(i);
                T[i].r = tab_r(i);
            }

            // Compute minimum global cost using recursive function
            float J = 0.0f, J_min = HUGE_VALF;
            core::RowVectorXi X = core::RowVectorXi::Zero(0), X_min = core::RowVectorXi::Zero(0);
            globalCost(A, T, J, X, J_min, X_min, nk, solvers);

            // Return assignment vector
            return X_min;
        }

    private: // Recursive cost calculation. Branch and bound
        void globalCost(std::vector<Agent>& A, const std::vector<Cluster>& T, 
            const float& J, const core::RowVectorXi& X,
            float& J_min, core::RowVectorXi& X_min, 
            const core::RowVectorXi& nk, std::vector<PositionSolver::SharedPtr>& solvers)
        {
            //std::cout << "---------------> Global cost: " << J << std::endl;

            // Get number of agents and clusters
            int m = static_cast<int>(A.size());
            int n = static_cast<int>(T.size());
            
            // If there are no remaining agents, it is understood that we are at the end of a branch,
            // so the cost is considered as a final cost. We check if J is lower than the minimum found so far,
            // in which case we update the value.
            if (m == 0)
            {
                if (J < J_min)
                {
                    J_min = J;
                    X_min = X;
                    //std::cout << "New minimum cost found: " << J_min << std::endl;
                    //std::cout << "New minimum assignment: " << X_min << std::endl;
                    //std::cout << "----------------" << std::endl;
                }
                return;
            }

            // Iterate through all agents
            for (int a = 0; a < m; a++)
            {
                // Get current agent
                auto& Ak = A[a];
                int k = Ak.k;

                // Calculate all possible permutations with available clusters
                core::RowVectorXi T_array(n);
                for (int i = 0; i < n; i++)
                {
                    T_array(i) = T[i].i;
                }
                //std::cout << "Calculating permutations for: " << T_array << " taking " << nk(k) << " clusters" << std::endl;
                const core::MatrixXi P = calculatePermutations(T_array, nk(k));
                int n_perms = P.rows();
                //std::cout << "Calculated " << n_perms << " permutations:" << std::endl;
                //std::cout << P << std::endl;

                // Iterate through all permutations
                for (int p = 0; p < n_perms; p++)
                {
                    // Get current permutation
                    std::vector<Cluster> Tk(nk(k));
                    for (int t = 0; t < nk(k); t++)
                    {
                        // Find the cluster in T that matches the current permutation
                        for (const auto& Ti : T)
                        {
                            if (P(p, t) == Ti.i)
                            {
                                Tk[t] = Ti;
                                break;
                            }
                        }
                    }
                    
                    // Calculate agent cost with current permutation
                    float Jk;
                    core::RowVectorXi Xk;
                    //std::cout << "Calculating agent cost for agent " << k << std::endl;
                    agentCost(Ak, Tk, Jk, Xk, nk(k), solvers[k]);
                    //std::cout << "Agent cost: " << Jk << std::endl;
                    //std::cout << "Agent assignment: " << Xk << std::endl;
                                       
                    // If current cost plus new cost is less than minimum found so far, continue down this branch,
                    // else, break and backtrack to avoid losing time.
                    if (J + Jk < J_min)
                    {
                        //std::cout << "Continuing down branch" << std::endl;

                        // Create a copy of the agent vector without the current agent
                        std::vector<Agent> An;
                        for (const auto& Akn : A)
                        {
                            // Find the agents that are not the current agent
                            if (Akn.k != k)
                            {
                                An.push_back(Akn);
                            }
                        }
                        
                        // Create a copy of the cluster vector without the assigned clusters
                        std::vector<Cluster> Tn;
                        for (const auto& Tin : T)
                        {
                            // Check if Tin.i is not in Xk
                            bool is_assigned = false;
                            for (int j = 0; j < nk(k); j++)
                            {
                                if (Tin.i == Xk(j))
                                {
                                    is_assigned = true;
                                    break;
                                }
                            }
                            
                            if (!is_assigned)
                            {
                                Tn.push_back(Tin);
                            }
                        }
                        
                        // Concatenate assignment vectors
                        core::RowVectorXi Xn;
                        if (X.size() > 0)
                        {
                            Xn.resize(X.size() + nk(k));
                            Xn.head(X.size()) = X;
                            Xn.tail(nk(k)) = Xk;
                        }
                        else
                        {
                            Xn = Xk;
                        }
                        
                        // Recursive call to continue branch exploration
                        globalCost(An, Tn, J + Jk, Xn, J_min, X_min, nk, solvers);
                    }
                }
            }

            // The process is finished
            //std::cout << "----------------" << std::endl;
        }

        void agentCost(Agent& Ak, const std::vector<Cluster>& Tk, 
            float& Jk, core::RowVectorXi& Xk, 
            const int& nk,
            PositionSolver::SharedPtr solver)
        {
            // Check if solver is valid
            if (!solver)
            {
                throw std::invalid_argument("Solver is not valid");
            }

            // Weights for each cost
            const float& Wo = params_.observation_weight;
            const float& Wd = params_.distance_weight;
            const float& Ws = params_.switch_weight;

            // Current position and assignment
            const core::Vector3r& x = Ak.x;
            const core::RowVectorXi& X_prev = Ak.X;

            // Get target array
            core::RowVectorXi Tk_array(nk);
            for (int i = 0; i < nk; i++)
            {
                Tk_array(i) = Tk[i].i;
            }

            // Check if the assignment has been calculated previously,
            // by searching if it is in the list of calculated assignments
            bool calculated = false;
            float J_calculated = 0.0f;
            core::RowVectorXi X_calculated;
            
            if (Ak.X_hist.rows() > 0)
            {
                for (int row = 0; row < Ak.J_hist.size(); row++)
                {
                    bool match = true;
                    for (int col = 0; col < nk; col++)
                    {
                        if (Tk_array(col) != Ak.X_hist(row, col))
                        {
                            match = false;
                            break;
                        }
                    }
                    
                    if (match)
                    {
                        calculated = true;
                        J_calculated = Ak.J_hist(row);
                        X_calculated = Ak.X_hist.row(row);
                        break;
                    }
                }
            }

            // If the assignment has been calculated previously,
            // use the stored value
            if (calculated)
            {
                Jk = J_calculated;
                Xk = X_calculated;
                return;
            }

            // If the assignment has not been calculated previously:
            // Calculate observation cost
            // Get cluster centers and radii matrices for all units (accounting for the central unit)
            core::Matrix3Xr tab_P = core::Matrix3Xr::Zero(3, nk + 1);
            core::RowVectorXr tab_r = core::RowVectorXr::Zero(nk + 1);
            // Tracking units
            for (int t = 1; t < nk + 1; t++)
            {
                tab_P.col(t) = Tk[t].C;
                tab_r(t) = Tk[t].r;
            }
            // Central unit (mean of all selected clusters and maximum radius)
            const auto& [central_P, central_r] = computeCentralCluster(tab_P, tab_r);
            tab_P.col(0) = central_P;
            tab_r(0) = central_r;
            // Run solver to get optimal position
            float Jo;
            core::Vector3r x_opt = solver->run(tab_P, tab_r, x, Jo);

            // Calculate distance cost
            float Jd = (x - x_opt).norm();

            // Calculate switch cost
            int Js = 0; // Number of swaps
            if ((X_prev.array() >= 0).all())
            {
                Js = (X_prev.array() != Tk_array.array()).sum();
            }

            // Calculate total cost
            Jk = Wo * Jo + Wd * Jd + Ws * Js;
            Xk = Tk_array;

            // Store calculated assignment and cost
            int n_hist = Ak.J_hist.size();
            
            // First allocation
            if (n_hist == 0)
            {
                Ak.X_hist.resize(1, nk);
                Ak.J_hist.resize(1);
                Ak.X_hist.row(0) = Xk;
                Ak.J_hist(0) = Jk;
            }
            else // Append to existing history
            {
                core::MatrixXi X_hist_new(n_hist + 1, nk);
                core::VectorXr J_hist_new(n_hist + 1);
                
                // Copy existing data
                X_hist_new.topRows(n_hist) = Ak.X_hist;
                J_hist_new.head(n_hist) = Ak.J_hist;
                
                // Add new data
                X_hist_new.row(n_hist) = Xk;
                J_hist_new(n_hist) = Jk;
                
                // Replace old with new
                Ak.X_hist = X_hist_new;
                Ak.J_hist = J_hist_new;
            }
        }

    private: // Utility methods
        core::MatrixXi calculatePermutations(const core::RowVectorXi& V, const int& n)
        {
            const int m = V.size();

            // Check if n is valid
            if (n > m)
            {
                throw std::invalid_argument("n is greater than the number of clusters");
            }
            
            // Calculate number of permutations: P(m,n) = m!/(m-n)!
            int K = 1;
            for (int i = m; i > m - n; i--)
                K *= i;
            
            // Initialize matrix to store permutations (K combinations of n)
            core::MatrixXi P(K, n);
            
            // For cases where n = m:
            if (n == m)
            {
                std::vector<int> I(m);
                for (int i = 0; i < m; i++)
                    I[i] = i;
                
                int row = 0;
                do {
                    for (int col = 0; col < m; col++)
                        P(row, col) = V(I[col]);
                    row++;
                } while (std::next_permutation(I.begin(), I.end()));
                
                return P;
            }
            
            // For cases where n < m:
            if (n < m)
            {
                std::vector<bool> used(m, false);
                std::vector<int> current(n);
                int row = 0;

                // Recursive function to generate permutations taking n elements from m
                std::function<void(int)> backtrack = [&](int depth) 
                {
                    // depth = how many have we chosen so far
                    if (depth == n) 
                    {
                        // Write one permutation into P
                        for (int j = 0; j < n; j++)
                            P(row, j) = V(current[j]);
                        row++;
                        return;
                    }
                    for (int i = 0; i < m; i++) 
                    {
                        if (!used[i]) 
                        {
                            used[i] = true;
                            current[depth] = i;
                            backtrack(depth + 1);
                            used[i] = false;
                        }
                    }
                };

                backtrack(0);
            }

            return P;
        }

        std::pair<core::Vector3r, float> computeCentralCluster(const core::Matrix3Xr& tab_P, const core::RowVectorXr& tab_r)
        {
            // Get number of tracking units
            int n = tab_P.cols();
            
            // Compute mean of all available clusters
            core::Vector3r z_mean = core::Vector3r::Zero();
            for (int i = 0; i < n; i++)
            {
                z_mean += tab_P.col(i);
            }
            z_mean /= static_cast<float>(n);

            // Get the largest possible radius
            float r_max = 0.0f;
            for (int i = 0; i < n; i++)
            {
                r_max = std::max(r_max, (z_mean - tab_P.col(i)).norm() + tab_r(i));
            }

            // Return central cluster and radius
            return std::make_pair(z_mean, r_max);
        }
    };

} // namespace flychams::coordination