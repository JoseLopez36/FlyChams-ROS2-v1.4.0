std_msgs/Header header             
string[] head_ids                           # IDs of the heads to be commanded
geometry_msgs/Vector3[] rpy_setpoints       # Head goal orientation (roll, pitch, yaw) (rad)
float32[] focal_setpoints                   # Head goal focal length (m)
float32[] projected_sizes                   # Target projected size in pixels (pix)