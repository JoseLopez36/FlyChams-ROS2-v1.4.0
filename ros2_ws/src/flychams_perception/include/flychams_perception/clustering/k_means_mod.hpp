#pragma once

// Core includes
#include "flychams_core/types/core_types.hpp"
#include "flychams_core/utils/math_utils.hpp"

namespace flychams::perception
{
	/**
	 * ════════════════════════════════════════════════════════════════
	 * @brief K-Means modified implementation
	 *
	 * @details
	 * This class implements a K-Means modified algorithm.
	 * It provides methods for clustering targets.
	 * The modifications include:
	 * - Inter-iteration cluster consistency.
	 * - Time persistence of clusters.
	 *
	 * ════════════════════════════════════════════════════════════════
	 * @author Jose Francisco Lopez Ruiz
	 * @date 2025-01-29
	 * ════════════════════════════════════════════════════════════════
	 */
	class KMeansMod
	{
	public: // Types
		using Clusters = std::unordered_set<core::ID>;					// Cluster IDs	
		using Points = std::unordered_map<core::ID, core::Vector3r>;    // Point ID -> Position
		using Assignments = std::unordered_map<core::ID, core::ID>; 	// Point ID -> Cluster ID

	private: // Types
		using PointMatrix = core::Matrix3Xr;
		using IndexVector = core::RowVectorXi;

	private: // Parameters
		// Persistence parameters
		float ini_bonding_coef_;
		float max_bonding_coef_;
		float bonding_coef_time_to_max_;
		float max_hysteresis_ratio_;
		float min_hysteresis_ratio_;

	private: // Data
		// Previous assignments
		IndexVector prev_assignments_;
		// Bonding coefficients	
		core::RowVectorXr bonding_coefs_;
		// First time flag
		bool first_update_;

	public: // Public methods
		// Configuration
		void reset();
		void setParameters(float ini_bonding_coef, float max_bonding_coef, float bonding_coef_time_to_max, float max_hysteresis_ratio, float min_hysteresis_ratio);
		// Control
		Assignments run(const Points& points, const Clusters& clusters, const float& dt);

	private: // Implementation
		// Base K-means implementation. Initial assignments
		IndexVector computeInitialAssignments(const PointMatrix& tab_P, int n, int K);
		IndexVector selectFarthestPoints(const PointMatrix& tab_P, int n, int K);
		core::MatrixXr computeDistMatrix(const PointMatrix& tab_P, int n);
		PointMatrix computeCentroids(const PointMatrix& tab_P, const IndexVector& assignments, int n, int K);
		IndexVector assignClusters(const PointMatrix& tab_P, const PointMatrix& centroids, int n, int K);

		// Consistent clustering implementation. Consistent centroids
		IndexVector reorderCentroidsConsistently(const PointMatrix& prev_centroids, const PointMatrix& curr_centroids, const IndexVector& prev_assignments, int n, int K);
		IndexVector associateCentroids(const PointMatrix& prev_centroids, const PointMatrix& curr_centroids, int K);
		core::MatrixXr computeDistMatrixTwoGroups(const PointMatrix& prev_centroids, const PointMatrix& curr_centroids, int K);

		// Clustering persistence implementation. Cluster persistence
		IndexVector ensureClusterPersistence(const PointMatrix& tab_P, const IndexVector& curr_assignments, const PointMatrix& curr_centroids, const IndexVector& prev_assignments, int n, int K, float dt);
		core::Vector3r calculateCentroidWithInclusion(const PointMatrix& tab_P, const IndexVector& assignments, int cluster_idx, int point_idx);
	};

} // namespace flychams::perception
