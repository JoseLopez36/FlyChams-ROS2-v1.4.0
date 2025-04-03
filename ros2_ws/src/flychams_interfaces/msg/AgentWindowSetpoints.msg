std_msgs/Header header             
string camera_id                            # Source camera ID
flychams_interfaces/Crop[] crop_setpoints   # Crop parameters
float32[] resolution_factors                # Resolution factor for this window (0-1)
float32[] projected_sizes                   # Target projected size in pixels (pix)