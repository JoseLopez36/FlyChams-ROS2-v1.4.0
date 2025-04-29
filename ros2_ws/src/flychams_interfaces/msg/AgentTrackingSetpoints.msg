std_msgs/Header header       

# Head data
string[] head_ids                           # IDs of the heads to be commanded
geometry_msgs/Vector3[] angles              # Head goal roll, pitch and yaw (rad)
float32[] focals                            # Head goal focal length (m)

# Window data
float32[] resolution_factors                # Resolution factor for windows (0-1)

# Common data
float32[] projected_sizes                   # Target projected size in pixels (pix)
