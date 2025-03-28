#include "flychams_perception/clustering/k_means_mod.hpp"

/* Hungarian algorithm: https://github.com/mcximing/hungarian-algorithm-cpp */
#include "Hungarian.h"

using namespace flychams::core;

namespace flychams::perception
{
	// ════════════════════════════════════════════════════════════════════════════
	// PUBLIC METHODS: Public methods for configuration and control
	// ════════════════════════════════════════════════════════════════════════════

	void KMeansMod::reset()
	{
		// Reset data
		prev_assignments_.resize(0);
		bonding_coefs_.resize(0);
		first_update_ = true;
	}

	void KMeansMod::setParameters(float ini_bonding_coef, float max_bonding_coef, float bonding_coef_time_to_max, float max_hysteresis_ratio, float min_hysteresis_ratio)
	{
		// Set parameters
		ini_bonding_coef_ = ini_bonding_coef;
		max_bonding_coef_ = max_bonding_coef;
		bonding_coef_time_to_max_ = bonding_coef_time_to_max;
		max_hysteresis_ratio_ = max_hysteresis_ratio;
		min_hysteresis_ratio_ = min_hysteresis_ratio;
	}

	KMeansMod::Assignments KMeansMod::run(const Points& points, const Clusters& clusters, const float& dt)
	{
		// Get number of points and clusters
		int n = static_cast<int>(points.size());
		int K = static_cast<int>(clusters.size());

		// Check edge cases
		if (K <= 0 || n < K)
			return Assignments(); // Return empty map

		// Create ID to index LUTs and point matrix
		std::unordered_map<int, core::ID> point_idx_to_id;
		std::unordered_map<core::ID, int> point_id_to_idx;
		std::unordered_map<int, core::ID> cluster_idx_to_id;
		std::unordered_map<core::ID, int> cluster_id_to_idx;
		PointMatrix tab_P(3, n);

		// Convert point IDs to indices and fill point matrix
		int point_idx = 0;
		for (const auto& [point_id, point_pos] : points)
		{
			// Fill LUTs
			point_idx_to_id[point_idx] = point_id;
			point_id_to_idx[point_id] = point_idx;

			// Fill point matrix
			tab_P.col(point_idx) = point_pos;

			// Increment point index
			point_idx++;
		}

		// Convert cluster IDs to indices
		int cluster_idx = 0;
		for (const auto& cluster_id : clusters)
		{
			// Fill LUTs
			cluster_idx_to_id[cluster_idx] = cluster_id;
			cluster_id_to_idx[cluster_id] = cluster_idx;

			// Increment cluster index
			cluster_idx++;
		}

		// Perform clustering with different methods, depending on first update flag
		IndexVector curr_assignments;
		if (first_update_)
		{
			// Perform clustering with base K-means
			curr_assignments = computeInitialAssignments(tab_P, n, K);
			first_update_ = false;
		}
		else
		{
			// Perform clustering with modified K-means:

			// 1. Perform clustering with base K-means
			curr_assignments = computeInitialAssignments(tab_P, n, K);

			// 2. Compute current and previous centroids
			PointMatrix curr_centroids = computeCentroids(tab_P, curr_assignments, n, K);
			PointMatrix prev_centroids = computeCentroids(tab_P, prev_assignments_, n, K);

			// 3. Ensure consistency in cluster numbering
			curr_assignments = reorderCentroidsConsistently(curr_centroids, prev_centroids, prev_assignments_, n, K);

			// 4. Ensure persistence of centroids
			curr_assignments = ensureClusterPersistence(tab_P, curr_assignments, curr_centroids, prev_assignments_, n, K, dt);
		}

		// Update previous assignments
		prev_assignments_.resize(curr_assignments.size());
		prev_assignments_ = curr_assignments;

		// Return assignments
		Assignments assignments_map;
		for (int i = 0; i < n; i++)
		{
			assignments_map[point_idx_to_id.at(i)] = cluster_idx_to_id.at(curr_assignments(i));
		}

		return assignments_map;
	}

	// ════════════════════════════════════════════════════════════════════════════
	// BASE K-MEANS: Compute initial assignments
	// ════════════════════════════════════════════════════════════════════════════

	KMeansMod::IndexVector KMeansMod::computeInitialAssignments(const PointMatrix& tab_P, int n, int K)
	{
		// Function that computes the initial assignments for the base K-means algorithm
		IndexVector assignments(n);

		// Initialize clusters with the K farthest points
		IndexVector farthest_list = selectFarthestPoints(tab_P, n, K);
		for (int i = 0; i < K; i++)
		{
			assignments(farthest_list(i)) = i;
		}

		// K-means Iteration
		bool changed = true;
		IndexVector prev_iter_assignments(n);
		while (changed)
		{
			// Compute new centroids
			PointMatrix centroids = computeCentroids(tab_P, assignments, n, K);

			// Store calculated assignments to check for changes
			prev_iter_assignments = assignments;

			// Assign clusters based on the updated centroids
			assignments = assignClusters(tab_P, centroids, n, K);

			// Check if any assignments have changed
			changed = (assignments.array() != prev_iter_assignments.array()).any();

			// If it hasn't changed, return the last result
		}

		return assignments;
	}

	KMeansMod::IndexVector KMeansMod::selectFarthestPoints(const PointMatrix& tab_P, int n, int K)
	{
		// Function that selects the 'K' farthest points from a given set of points provided by the
		// columns of 'points'. If 'K' is 1, it simply selects the first point.
		IndexVector selected_list = IndexVector::Zero(1);

		if (K == 1 || n == 1)
			return selected_list;

		// Calculate the distance matrix between points
		MatrixXr dist_matrix = computeDistMatrix(tab_P, n);

		// Find the indices of the two most separated points
		RowVectorXr max_vals(n);
		IndexVector max_indices(n);
		for (int i = 0; i < n; i++)
		{
			max_vals(i) = dist_matrix.col(i).maxCoeff(&max_indices(i));
		}

		int max_col;
		max_vals.maxCoeff(&max_col);
		int idx1 = max_indices(max_col);
		int idx2 = max_col;

		// Initially, the selected list contains the two most distant points
		selected_list.resize(2);
		selected_list << idx1, idx2;
		int selected_count = 2;

		int max_min_index = 0;
		while (selected_count < K)
		{
			float current_max_min_dist = 0.0f;
			for (int i = 0; i < n; i++)
			{
				// If the i-th point is not already selected
				if (!((selected_list.array() == i).any()))
				{
					float min_dist = HUGE_VALF;
					for (int j = 0; j < selected_count; j++)
					{
						int current_cluster = selected_list(j);
						if (dist_matrix(i, current_cluster) < min_dist)
						{
							min_dist = dist_matrix(i, current_cluster);
						}
					}
					if (min_dist > current_max_min_dist)
					{
						current_max_min_dist = min_dist;
						max_min_index = i;
					}
				}
			}

			selected_list.conservativeResize(selected_count + 1);
			selected_list(selected_count) = max_min_index;
			selected_count++;
		}

		return selected_list;
	}

	MatrixXr KMeansMod::computeDistMatrix(const PointMatrix& tab_P, int n)
	{
		// Function that computes the distance matrix between each pair of points in a set.
		MatrixXr dist_matrix = MatrixXr::Zero(n, n);

		float dist;
		for (int i = 0; i < n; i++)
		{
			for (int j = i + 1; j < n; j++)
			{
				dist = (tab_P.col(i) - tab_P.col(j)).norm();
				dist_matrix(i, j) = dist;
				dist_matrix(j, i) = dist;
			}
		}

		return dist_matrix;
	}

	KMeansMod::PointMatrix KMeansMod::computeCentroids(const PointMatrix& tab_P, const IndexVector& assignments, int n, int K)
	{
		PointMatrix centroids(3, K);
		centroids.setZero();
		IndexVector num_points(K);
		num_points.setZero();

		// Sum all points for each cluster
		for (int j = 0; j < n; j++)
		{
			int cluster_idx = assignments(j);
			centroids.col(cluster_idx) += tab_P.col(j);
			num_points(cluster_idx)++;
		}

		// Compute average for each centroid
		for (int i = 0; i < K; i++)
		{
			const float& n_points = static_cast<float>(num_points(i));
			if (n_points > 0.0f)
			{
				centroids.col(i) /= n_points;
			}
		}

		return centroids;
	}

	KMeansMod::IndexVector KMeansMod::assignClusters(const PointMatrix& tab_P, const PointMatrix& centroids, int n, int K)
	{
		IndexVector assignments = IndexVector::Zero(n);

		for (int i = 0; i < n; i++)
		{
			float min_dist = HUGE_VALF;
			int best_cluster = 0;

			for (int j = 0; j < K; j++)
			{
				float dist = (tab_P.col(i) - centroids.col(j)).norm();
				if (dist < min_dist)
				{
					min_dist = dist;
					best_cluster = j;
				}
			}

			assignments(i) = best_cluster;
		}

		return assignments;
	}

	// ════════════════════════════════════════════════════════════════════════════
	// CONSISTENT CLUSTERING: Compute consistent centroids
	// ════════════════════════════════════════════════════════════════════════════

	KMeansMod::IndexVector KMeansMod::reorderCentroidsConsistently(const PointMatrix& curr_centroids, const PointMatrix& prev_centroids, const IndexVector& prev_assignments, int n, int K)
	{
		// Function that reorders the centroids to ensure consistency in cluster numbering.
		IndexVector assignments(n);

		// Ensure consistency in cluster numbering
		IndexVector associations = associateCentroids(prev_centroids, curr_centroids, K);

		// Update point assignments based on reordered centroids
		for (int i = 0; i < n; i++)
		{
			assignments(i) = associations(prev_assignments(i));
		}

		return assignments;
	}

	KMeansMod::IndexVector KMeansMod::associateCentroids(const PointMatrix& prev_centroids, const PointMatrix& curr_centroids, int K)
	{
		IndexVector associations(K);

		// Calculate the distance matrix between elements of both groups
		MatrixXr dist_matrix = computeDistMatrixTwoGroups(prev_centroids, curr_centroids, K);

		// Hungarian algorithm
		// Convert the Eigen distance matrix to a vector of vectors for the Hungarian Algorithm
		std::vector<std::vector<double>> cost_matrix(K, std::vector<double>(K));
		for (int i = 0; i < K; i++)
		{
			for (int j = 0; j < K; j++)
			{
				cost_matrix[i][j] = static_cast<double>(dist_matrix(i, j));
			}
		}

		// Execute the Hungarian Algorithm
		HungarianAlgorithm hung_algo_obj;
		std::vector<int> assigned_idx;
		hung_algo_obj.Solve(cost_matrix, assigned_idx);

		// Convert the assignment result to a RowVectorXi
		for (int i = 0; i < K; i++)
		{
			associations(i) = assigned_idx[i];
		}

		return associations;
	}

	MatrixXr KMeansMod::computeDistMatrixTwoGroups(const PointMatrix& prev_centroids, const PointMatrix& curr_centroids, int K)
	{
		// Function that provides the distance matrix between points of two groups.
		// Unlike 'ComputeDistMatrix()', which calculated distances between all points in a single set
		MatrixXr dist_matrix = MatrixXr::Zero(K, K);

		// Calculate squared Euclidean distances
		for (int i = 0; i < K; i++)
		{
			for (int j = 0; j < K; j++)
			{
				dist_matrix(i, j) = std::pow((prev_centroids.col(i) - curr_centroids.col(j)).norm(), 2);
			}
		}
		return dist_matrix;
	}

	// ════════════════════════════════════════════════════════════════════════════
	// PERSISTENCE: Ensure persistence of centroids
	// ════════════════════════════════════════════════════════════════════════════

	KMeansMod::IndexVector KMeansMod::ensureClusterPersistence(const PointMatrix& tab_P, const IndexVector& curr_assignments, const PointMatrix& curr_centroids, const IndexVector& prev_assignments, int n, int K, float dt)
	{
		// Function that ensures the persistence of centroids over time
		IndexVector assignments(n);
		assignments = curr_assignments; // Start with input assignments

		// Check and fix parameters
		if (ini_bonding_coef_ > max_bonding_coef_)
		{
			ini_bonding_coef_ = max_bonding_coef_;
		}
		if (min_hysteresis_ratio_ > max_hysteresis_ratio_)
		{
			min_hysteresis_ratio_ = max_hysteresis_ratio_ / 2.0f;
		}

		// Calculate the slope for incrementing the bonding coefficient
		float bonding_coef_incr_slope = max_bonding_coef_ / bonding_coef_time_to_max_;

		// Initialize bonding coefficients if empty
		if (bonding_coefs_.size() == 0)
		{
			bonding_coefs_ = core::RowVectorXr::Constant(n, ini_bonding_coef_);
		}

		// Flag to track if hysteresis has been applied (bonding mechanism active)
		bool bonding_active = false;

		// For each point, check if its assignment has changed
		for (int i = 0; i < n; i++)
		{
			int prev_cluster = prev_assignments(i);
			int curr_cluster = assignments(i);

			// If the point remains in the same cluster
			if (curr_cluster == prev_cluster)
			{
				// Increase the bonding coefficient
				bonding_coefs_(i) += bonding_coef_incr_slope * dt;

				// Enforce maximum bonding coefficient
				bonding_coefs_(i) = std::min(bonding_coefs_(i), max_bonding_coef_);
			}
			else
			{
				// Decrease the bonding coefficient
				bonding_coefs_(i) -= bonding_coef_incr_slope * dt;

				// Enforce minimum bonding coefficient
				bonding_coefs_(i) = std::max(bonding_coefs_(i), 0.0f);

				// Calculate dynamic hysteresis threshold for distance measurements
				float hysteresis_ratio = min_hysteresis_ratio_ + bonding_coefs_(i) * (max_hysteresis_ratio_ - min_hysteresis_ratio_) / max_bonding_coef_;

				// Calculate where the centroid of the previous cluster would be 
				// if this point continued to be associated with it
				core::Vector3r prev_cluster_centroid_with_point = calculateCentroidWithInclusion(tab_P, assignments, prev_cluster, i);

				// Calculate distances
				float dist_to_prev_cluster = (tab_P.col(i) - prev_cluster_centroid_with_point).norm();
				float dist_to_curr_cluster = (tab_P.col(i) - curr_centroids.col(curr_cluster)).norm();
				float dist_between_clusters = (prev_cluster_centroid_with_point - curr_centroids.col(curr_cluster)).norm();

				// If the difference doesn't exceed the current hysteresis threshold,
				// revert the point to its previous cluster assignment
				if ((dist_to_prev_cluster - dist_to_curr_cluster) / dist_between_clusters <= hysteresis_ratio)
				{
					assignments(i) = prev_cluster;
					bonding_active = true; // Indicate that hysteresis has been applied
				}
				else
				{
					// If the point should change to the new cluster, reset its bonding coefficient
					bonding_coefs_(i) = ini_bonding_coef_;
				}
			}
		}

		return assignments;
	}

	// Helper function to calculate a centroid including a specific point
	core::Vector3r KMeansMod::calculateCentroidWithInclusion(const PointMatrix& tab_P, const IndexVector& assignments, int cluster_idx, int point_idx)
	{
		core::Vector3r centroid = core::Vector3r::Zero();
		int count = 0;

		// Sum all points belonging to the cluster
		for (int i = 0; i < tab_P.cols(); i++)
		{
			if (assignments(i) == cluster_idx || i == point_idx)
			{
				centroid += tab_P.col(i);
				count++;
			}
		}

		// Calculate average
		if (count > 0)
		{
			centroid /= static_cast<float>(count);
		}

		return centroid;
	}

} // namespace flychams::perception
