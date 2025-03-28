#include "flychams_coordination/assignment/assignment_solver.hpp"

using namespace flychams::core;

namespace flychams::coordination
{
    // ════════════════════════════════════════════════════════════════════════════
    // PUBLIC METHODS: Public methods for configuration and control
    // ════════════════════════════════════════════════════════════════════════════

    void AssignmentSolver::reset()
    {
        // Nothing to do
    }

    void AssignmentSolver::setMode(const AssignmentMode& mode)
    {
        mode_ = mode;
    }

    AssignmentSolver::Assignments AssignmentSolver::runGreedy(const Clusters& C, const Agents& A)
    {
        // Initialize empty assignment
        Assignments assignments;

        // If no clusters, return empty assignment
        if (C.empty())
        {
            return assignments;
        }

        // Create a list of unassigned clusters
        Clusters U;
        for (const auto& Cj : C)
        {
            U.insert(Cj);
        }

        // Create a map to track how many clusters are assigned to each agent
        std::unordered_map<core::ID, int> n;
        for (const auto& [Ai_id, _] : A)
        {
            n[Ai_id] = 0;
        }

        // Assign clusters to agents greedily based on distance
        while (!U.empty())
        {
            float min_dist = HUGE_VALF;
            core::ID best_A_id = "";
            core::ID best_U_id = "";

            // Find the best agent-cluster pair based on distance
            for (const auto& [Uj_id, Uj] : U)
            {
                for (const auto& [Ai_id, Ai] : A)
                {
                    // Skip if agent has reached maximum assignments
                    if (n[Ai_id] >= Ai.second)
                        continue;

                    // Calculate distance
                    float dist = (Ai.first - Uj.first).norm();

                    // Update best assignment if this is better
                    if (dist < min_dist)
                    {
                        min_dist = dist;
                        best_A_id = Ai_id;
                        best_U_id = Uj_id;
                    }
                }
            }

            // If we found a valid assignment
            if (min_dist < HUGE_VALF)
            {
                // Add to assignments
                assignments[best_U_id] = best_A_id;

                // Increment agent assignment count
                n[best_A_id]++;

                // Remove the assigned cluster from unassigned list
                U.erase(best_U_id);
            }
            else
            {
                // No valid assignment found for remaining clusters
                break;
            }
        }

        return assignments;
    }

} // namespace flychams::coordination