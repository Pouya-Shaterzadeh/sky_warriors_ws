# Failsafe Testing for Sky Warrior (Gazebo Classic 11)
# ====================================================

This folder contains the automated end-to-end failsafe testing architecture for the Sky Warrior drone swarm.
It uses the MAVSDK Python Failure API interacting with PX4 SITL and Gazebo Classic 11.

## Architecture

The failsafe architecture consists of three layers:

1. **`failsafe_manager.py`**: A core class (`FailsafeManager`) handling MAVSDK connection, parameter configuration (enabling `SYS_FAILURE_EN`), failure injection via `drone.failure.inject()`, health validation, taking off, and telemetry monitoring for transition states (like entering `RETURN_TO_LAUNCH`).
2. **`failsafe_tests.py`**: A pytest-based suite containing specific test scenarios (GPS failure, Battery critical, RC Loss, sensor degraded states). Each test relies on the `FailsafeManager` to automate the process (Setup -> Inject -> Detect -> Verify -> Recover).
3. **`run_failsafe_tests.py`**: A CLI runner to trigger all or specific tests without needing to invoke `pytest` manually.

### Auxiliary Components
- **`parameters/failsafe_config.yaml`**: Centralized configuration handling PX4 failure parameters and default test timeouts.
- **`failsafe_monitor_ros2.py`**: A passive ROS2 node that listens to PX4's topics (like `/fmu/out/vehicle_status`) to log structured JSON events when failsafe state transitions occur.

## Quick Start (Automated Testing)

### 1. Prerequisites
- `pip install mavsdk pytest pyyaml`
- Ensure Gazebo Classic 11 (`gazebo`) and PX4 SITL (`make px4_sitl gazebo-classic`) are launched, and a drone is spawned.

### 2. Run Tests
Use the provided CLI tool inside `scripts/`:

```bash
cd ~/sky_warriors_ws/src/failsafes/scripts
python3 run_failsafe_tests.py --all
```

To run a specific test:
```bash
python3 run_failsafe_tests.py --test gps_loss
```

To list tests:
```bash
python3 run_failsafe_tests.py --list
```

## Important Considerations (Gazebo Classic 11)
- The scripts connect by default to `udpin://0.0.0.0:14540`.
- The `SYS_FAILURE_EN` parameter **must** be set to `1` in PX4. The `FailsafeManager` attempts to set this automatically during connection setup.
- MAVSDK's `failure.inject(FailureUnit, FailureType)` is a blocking call. If `SYS_FAILURE_EN` is 0, the injection call will timeout. The test suite handles this gracefully by wrapping the call in an `asyncio.timeout`.
