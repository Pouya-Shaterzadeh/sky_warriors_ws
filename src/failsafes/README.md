# Failsafe Testing for Sky Warrior

This directory supports both manual and automated PX4/MAVSDK failure injection testing.

## Scripts

- `scripts/failsafe_manager.py`: core MAVSDK helper class used by automated tests.
- `scripts/mavsdkrunner.py`: **manual** PX4 shell command runner.
- `scripts/mavsdkrunner_automated.py`: **automated** mission-based failure scenario runner.

## 1) Launch SITL (required for both modes)

From repository root:

```bash
cd /home/skywarrior/Projects/mavsdk-api-failsafe
./src/launch_classic_3_iris_lz.sh
```

This launches 3 PX4 SITL instances and Gazebo Classic.

## 2) Manual Failure Injection Testing

Open a second terminal and run:

```bash
cd /home/skywarrior/Projects/mavsdk-api-failsafe
python3 src/failsafes/scripts/mavsdkrunner.py --ports 14541,14542,14543
```

Inside `px4-manual>` run:

```text
:targets
param set SYS_FAILURE_EN 1
param show SYS_FAILURE_EN
failure gps off
failure gps ok
```

### Manual runner control commands

- `:targets` list connected drones.
- `:use drone0` / `:use drone1` / `:use drone2` target one drone.
- `:use all` broadcast to all connected drones.
- `:help` print runner help.
- `:exit` exit the runner.

## 3) Automated Failure Injection Testing

Open another terminal and run:

```bash
cd /home/skywarrior/Projects/mavsdk-api-failsafe/src/failsafes/scripts
python3 mavsdkrunner_automated.py
```

Then choose a scenario from the menu (GPS, baro, RC link, etc.). The runner uses MAVSDK Failure API and validates expected failsafe mode changes.

## Notes

- Manual runner default ports are `14541,14542,14543` (matching the current 3-drone SITL launch).
- Automated runner default MAVSDK connection is `udpin://0.0.0.0:14541`.
- `SYS_FAILURE_EN` must be `1` for both manual `failure ...` commands and MAVSDK failure injection.
- Some failure types may be unsupported by specific PX4 builds/simulator combinations.

## Multi-drone RTL landing zones

Landing zone mapping in this setup:

- Drone `0` -> `(0, 0)`
- Drone `1` -> `(4, 3)`
- Drone `2` -> `(8, 0)`
