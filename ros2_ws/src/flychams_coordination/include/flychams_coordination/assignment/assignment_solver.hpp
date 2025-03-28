#pragma once

// Standard includes
#include <vector>
#include <unordered_map>
#include <unordered_set>

// Utilities
#include "flychams_core/types/core_types.hpp"
#include "flychams_core/utils/math_utils.hpp"

namespace flychams::coordination
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Solver for cluster-agent assignment problems
     *
     * @details
     * This class implements different algorithms for solving the
     * cluster-agent assignment problem. It supports multiple assignment
     * modes including greedy coordination and sub-optimal coordination,
     * with configurable parameters for optimization.
     *
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-01-31
     * ════════════════════════════════════════════════════════════════
     * Assignment coordination modes:
     * - GreedyCoordination: Assigns clusters to agents based on distance
     * - SubOptimalCoordination: Advanced coordination algorithm (TODO)
     * ════════════════════════════════════════════════════════════════
     */
    class AssignmentSolver
    {
    public: // Types
        // Modes
        enum class AssignmentMode
        {
            GREEDY,     // Distance-based greedy assignment. Default
            SUBOPTIMAL  // Advanced coordination (TODO)
        };
        // Data
        using Agents = std::unordered_map<core::ID, std::pair<core::Vector3r, int>>;        // Agent ID -> (Position, Max assignments)
        using Clusters = std::unordered_map<core::ID, std::pair<core::Vector3r, float>>;    // Cluster ID -> (Center, Radius)
        using Assignments = std::unordered_map<core::ID, core::ID>; 	                    // Agent ID -> Cluster ID

    private: // Parameters
        AssignmentMode mode_;

    public: // Public methods
        // Configuration
        void reset();
        void setMode(const AssignmentMode& mode);
        // Control
        Assignments runGreedy(const Clusters& C, const Agents& A);
    };

} // namespace flychams::coordination