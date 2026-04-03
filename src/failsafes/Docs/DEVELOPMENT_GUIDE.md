# Failsafe Development Guide (Simple Version)

This project now keeps a small failsafe test setup.

## Scripts you should use

- `scripts/failsafe_manager.py`
- `scripts/mavsdkrunner.py`
- `scripts/mavsdkrunner_automated.py`

## Basic workflow

1. Start Gazebo + PX4 SITL.
2. Open terminal:

```bash
cd ~/sky_warriors_ws/src/failsafes/scripts
```

3. For manual/global PX4 shell commands:

```bash
python3 mavsdkrunner.py
```

4. Run commands such as:

```bash
param set SYS_FAILURE_EN 1
failure gps off
```

5. For automated mission + failure test scenarios:

```bash
python3 mavsdkrunner_automated.py
```

6. Choose a failure from the menu and watch how PX4 reacts (`RETURN_TO_LAUNCH`, `LAND`, etc.).

## What the manager handles for you

- Connection to PX4
- Enabling `SYS_FAILURE_EN`
- Arming and takeoff
- Failure injection and restore
- Mode checks and safe teardown

## If injection times out

This usually means `SYS_FAILURE_EN` is still `0`.

Set it in PX4 shell:

```bash
param set SYS_FAILURE_EN 1
param save
```

Then restart SITL and run again.

## Multi-drone RTL landing zones

In `skyw_multi_lz_world`, drone-to-LZ mapping is:

- Drone0 -> `(0,0)`
- Drone1 -> `(4,3)`
- Drone2 -> `(8,0)`

Spawn scripts use the same mapping, so RTL returns each drone to its own LZ.
