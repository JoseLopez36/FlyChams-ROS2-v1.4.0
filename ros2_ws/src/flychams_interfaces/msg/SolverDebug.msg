# Global data
# Number of iterations
int32 n_iterations
# Number of clusters
int32 n_clusters
# Agent position (m)
geometry_msgs/Point x0

# Per-iteration data
# Iteration index
int32 n
# Elapsed time (ms)
float32 t
# Cluster positions and radii values (m)
geometry_msgs/Point[] tab_p
float32[] tab_r
# Optimal cost values
float32 j_ellipsoid
float32 j_pso
float32 j_alc_pso
float32 j_nesterov
float32 j_nelder_mead
float32 j_l_bfgs
# Optimal position values (m)
geometry_msgs/Point x_ellipsoid
geometry_msgs/Point x_pso
geometry_msgs/Point x_alc_pso
geometry_msgs/Point x_nesterov
geometry_msgs/Point x_nelder_mead
geometry_msgs/Point x_l_bfgs
# Execution time values (ms)
float32 t_ellipsoid
float32 t_pso
float32 t_alc_pso
float32 t_nesterov
float32 t_nelder_mead
float32 t_l_bfgs
