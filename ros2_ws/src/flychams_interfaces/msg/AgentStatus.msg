std_msgs/Header header
uint8 status

# Status values
# IDLE: 0        Initial state, UAV is inactive
# DISARMED: 1    UAV is disarmed, safe state
# ARMED: 2       UAV is armed, ready for takeoff
# TAKING_OFF: 3  UAV is taking off
# TAKEN_OFF: 4   UAV has taken off
# HOVERING: 5    UAV is hovering
# HOVERED: 6     UAV has hovered
# TRACKING: 7    UAV is tracking targets
# LANDING: 8     UAV is landing
# LANDED: 9      UAV has landed
# ERROR: 10      Error state, requires reset