# Sky Warrior Simulation Scenario

This document describes the end-to-end mission that should be demonstrated in simulation.
The scenario is built around a world with:

- A start area (blue point)
- Six QR walls (hexagon layout)
- Red and blue landing pads

## Mission Objective

Run a complete autonomous swarm mission where perception drives task allocation and control:

1. Detect QR mission data.
2. Reconfigure formation.
3. Navigate to decoded goal.
4. Land all drones on the requested pad color.

## Full Mission Flow

### Phase 1: Takeoff and Stabilization

1. Arm and take off all three drones from the start area.
2. Hold a safe altitude and stable initial formation.

Expected outcome:
- All drones are airborne and stable.
- No waypoint transitions start before takeoff is complete.

### Phase 2: QR Scanning Pass

1. Send Drone 1 (mono camera platform) to perform a short scan in front of the six QR walls.
2. Keep Drones 2 and 3 in safe hover or follower behavior.

Expected outcome:
- Camera feed is active.
- QR detector publishes decoded mission payload (for example: formation type, target pad color, goal point).

### Phase 3: Mission Decode and Task Allocation

1. Parse decoded QR string into mission fields.
2. Feed mission fields into the task allocator (CBBA).
3. Assign swarm objective and role updates.

Expected outcome:
- A valid task is produced from perception data.
- The swarm receives a synchronized objective update.

### Phase 4: Formation Reconfiguration

1. Transition swarm geometry from current formation to decoded formation (for example, line to diamond or triangle).
2. Maintain collision-safe spacing during the transition.

Expected outcome:
- Formation change occurs without losing control.
- Inter-vehicle distance constraints are respected.

### Phase 5: Goal Navigation

1. Command swarm motion to the decoded goal point.
2. Maintain formation coherence while moving.

Expected outcome:
- Swarm reaches the goal region.
- Position error is within mission tolerance.

### Phase 6: Color-Based Landing

1. Use color detector to identify target landing pad (red or blue).
2. Execute precise descent and touchdown logic for all drones.

Expected outcome:
- All drones land on the requested color pad.
- Landing is stable and mission completes without manual intervention.

## Required Data Interfaces

At minimum, the scenario should include these logical interfaces:

- Camera image input for QR and color detectors
- QR decoded string topic
- Task allocation input/output (mission fields to assignments)
- Swarm controller command interface (formation and goal updates)
- Pad detection output for landing guidance

## Demonstration Checklist

Use this checklist to verify the run:

- [ ] All 3 drones take off successfully.
- [ ] Drone 1 performs QR scan path.
- [ ] At least one QR message is decoded and published.
- [ ] Task allocator updates swarm objective.
- [ ] Formation changes to decoded shape.
- [ ] Swarm reaches decoded goal point.
- [ ] Color detector identifies target pad.
- [ ] All drones land on correct pad color.

## Notes

- Keep coordinate frame conventions consistent across perception, planning, and control.
- Ensure timeout/fallback behavior exists if QR is not detected.
- Log decoded payload, selected formation, and landing target for post-run validation.


