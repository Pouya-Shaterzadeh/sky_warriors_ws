uint8 agent_id

# Pose
float64 x
float64 y
float64 z
float64 yaw

# Motion
float64 vx
float64 vy
float64 vz

# Health
float32 battery_pct         # 0..100
uint8 health_state          # 0 ok, 1 degraded, 2 critical
uint8 availability          # 0 idle, 1 executing, 2 unavailable
uint32 current_task_id      # 0 if none

# Capabilities bitmask
uint16 capability_mask      # see below

float64 last_update_time