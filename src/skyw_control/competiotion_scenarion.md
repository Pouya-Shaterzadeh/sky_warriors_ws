## Competition Scenario: 3 drones + mono camera to QR wall

### Recommended architecture

Use `mission_sequencer.py` (in `skyw_swarm`) as the high-level state machine and keep `px4_offboard_bridge.py` as the low-level PX4 interface.

- `mission_sequencer.py` publishes `/droneX/setpoint_position`
- `px4_offboard_bridge.py` streams PX4 offboard topics and auto-sends ARM + OFFBOARD commands
- `qr_code_detector_node.py` publishes `/qr_decoded`, which advances mission logic

This is the clean split:
- **High-level decisions**: `mission_sequencer.py`
- **Low-level vehicle protocol**: `px4_offboard_bridge.py`

### Mission implemented now

1. Take off all 3 drones to a safe hold altitude (`takeoff_z`, NED frame).
2. Keep drones 2 and 3 hovering near reference.
3. Send drone 1 (`x500_mono`) to `wall_1` target:
   - wall pose reference: `(x=5, y=0, z=-1, yaw=1.57)` in PX4 local frame
4. Hold and scan QR until decode or timeout.

### Run

```bash
colcon build --packages-select skyw_swarm
source install/setup.bash
ros2 launch skyw_swarm swarm_mission_scenario.launch.py use_sim_time:=true
```

### Useful launch overrides

```bash
ros2 launch skyw_swarm swarm_mission_scenario.launch.py \
  use_sim_time:=true \
  takeoff_z:=-2.5 \
  wall_x:=5.0 wall_y:=0.0 wall_z:=-1.0 wall_yaw:=1.57
```

### Important note about frames

PX4 local setpoints are NED, so **up is negative Z**.  
If your world reference is ENU `(5, 0, +1)`, the equivalent NED setpoint is typically `(5, 0, -1)` for altitude.
