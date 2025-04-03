std_msgs/Header header

# Agent data
geometry_msgs/Point position
geometry_msgs/Point setpoint

# Tracking data
float32[] focals                     # Associated tracking focal length for this cluster (only used if tracking_mode == 0)
float32[] resolution_factors         # Associated tracking resolution factor for this cluster (only used if tracking_mode == 1)
float32[] projected_sizes            # Associated tracking projected size for this cluster

# Position and movement metrics
float32 distance_traveled            # Total distance covered by the UAV (m)
float32 speed                        # Current speed of the UAV (m/s)

# Goal-related metrics
float32 distance_to_goal             # Current distance to assigned goal (m)

# Energy metrics
#float32 battery_level               # TODO: Remaining battery percentage (0-1) 
#float32 power_consumption           # TODO: Current power consumption rate (W)

# Mission metrics
#float32 average_distance_to_targets # TODO: Average distance to assigned targets (m)
#int32 waypoints_reached             # TODO: Number of waypoints successfully reached
float32 time_elapsed                 # Time elapsed since mission start (s)

# Performance metrics
float32 average_speed                # Average speed during mission (m/s)
#float32 tracking_error              # TODO: Error in following planned trajectory (m)
#float32 stability_metric            # Measure of flight stability (0-1)