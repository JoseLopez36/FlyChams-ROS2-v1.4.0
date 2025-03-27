std_msgs/Header header             
string[] head_ids                           # IDs of the heads to be commanded
geometry_msgs/Quaternion[] quat_setpoints   # Head goal orientation
float32[] fov_setpoints                     # Head goal fov (rad)