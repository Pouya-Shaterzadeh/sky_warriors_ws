# state_manager

A ROS2 package for centralized swarm state management using a finite state machine (FSM).

## Purpose
- Manages swarm states (e.g., FORMATION, DETACHING, WAITING, RECONFIGURING, IDLE).
- Listens to QR detection, manual commands, and UAV status topics.
- Publishes commands to control UAVs (detach, formation, etc.).
- Can operate independently of CBBA or any other allocation logic.

## Key Features
- Modular and extensible FSM logic.
- ROS2 topic-based communication for easy integration.
- Easily add new states, triggers, and outputs as needed.

## Structure
- `state_manager_node.py`: Main FSM node implementation.
- `launch/`: Launch files for easy startup.
- `resource/`: Package resource files.
- `README.md`: This file.

## Example Topics
- Input: `/qr_decoded`, `/manual_command`, `/uav_status`
- Output: `/swarm/state_change`, `/swarm/detach_command`, `/swarm/formation_command`

## How to Extend
- Add new subscriptions for more triggers (e.g., emergencies).
- Add new publishers for new commands (e.g., land, return home).
- Implement more complex state logic as needed.

---

This package was created based on a design discussion to provide a flexible, standalone FSM for UAV swarm management, with or without CBBA integration.