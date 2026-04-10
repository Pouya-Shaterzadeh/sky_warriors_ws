# State Manager Node

This node implements a finite state machine (FSM) for UAV swarm management. It listens to QR detection, manual commands, and UAV status, and publishes state changes and commands to the swarm.

## Key Features
- Standalone FSM logic (no dependency on CBBA)
- Easily extensible for new states, triggers, and outputs
- ROS2 topic-based communication

## Example FSM States
- FORMATION
- DETACHING
- WAITING
- RECONFIGURING
- IDLE

## Example Topics
- Input: `/qr_decoded`, `/manual_command`, `/uav_status`
- Output: `/swarm/state_change`, `/swarm/detach_command`, `/swarm/formation_command`

See the main README for more details.