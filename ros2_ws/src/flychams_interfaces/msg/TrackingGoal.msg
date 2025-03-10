std_msgs/Header header

# IDs of the windows where the tracking image will be displayed
string[] window_ids                

# Tracking unit types
uint8[] unit_types                     # Tracking unit types (0: phys-zoom, 1: dig-zoom)

# Physical zoom goals
string[] head_ids                       # Source of the tracking images (one per each physical unit)
geometry_msgs/Quaternion[] orientations # Unit goal orientation
float32[] fovs                          # Unit goal fov (rad)

# Digital zoom goals
string camera_id                       # Source of the tracking images (one for every digital unit, typically the central camera)
flychams_interfaces/Crop[] crops       # Crop parameters (one per each digital unit)
float32[] resolution_factors           # Resolution factor (for debugging purposes)