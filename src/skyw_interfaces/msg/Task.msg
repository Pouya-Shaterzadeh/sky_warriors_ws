uint32 task_id
uint32 task_version
uint16 task_type            # enum (see below)
uint8 priority              # 0..255
float64 created_time        # seconds (epoch or ROS time)
float64 deadline_time       # 0 if none

# Optional target (use NaN if not used)
float64 target_x
float64 target_y
float64 target_z
float64 target_yaw

# Task parameters
string formation_type       # "v", "line", "okbasi", etc
float32 formation_spacing
float32 formation_altitude
float32 formation_rotation_deg

uint8 required_agents       # how many agents needed
uint16 required_capability  # bitmask