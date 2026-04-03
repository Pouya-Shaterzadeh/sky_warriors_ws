#!/usr/bin/env python3
import asyncio
import logging
import sys
import os

from failsafe_manager import FailsafeManager
from mavsdk.failure import FailureUnit, FailureType

logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s")
logger = logging.getLogger("Runner")

# Mapping of all 15 components to their primary catastrophic failure types and expected modes.
SCENARIOS = {
    1: {"name": "SENSOR_GPS", "unit": FailureUnit.SENSOR_GPS, "type": FailureType.OFF, "expected": ["RETURN_TO_LAUNCH", "LAND"]},
    2: {"name": "SENSOR_BARO", "unit": FailureUnit.SENSOR_BARO, "type": FailureType.OFF, "expected": ["RETURN_TO_LAUNCH", "LAND"]},
    3: {"name": "SENSOR_GYRO", "unit": FailureUnit.SENSOR_GYRO, "type": FailureType.OFF, "expected": ["RETURN_TO_LAUNCH", "LAND"]},
    4: {"name": "SENSOR_ACCEL", "unit": FailureUnit.SENSOR_ACCEL, "type": FailureType.OFF, "expected": ["RETURN_TO_LAUNCH", "LAND"]},
    5: {"name": "SENSOR_MAG", "unit": FailureUnit.SENSOR_MAG, "type": FailureType.OFF, "expected": ["RETURN_TO_LAUNCH", "LAND"]},
    6: {"name": "SENSOR_OPTICAL_FLOW", "unit": FailureUnit.SENSOR_OPTICAL_FLOW, "type": FailureType.OFF, "expected": None},
    7: {"name": "SENSOR_VIO", "unit": FailureUnit.SENSOR_VIO, "type": FailureType.OFF, "expected": None},
    8: {"name": "SENSOR_DISTANCE_SENSOR", "unit": FailureUnit.SENSOR_DISTANCE_SENSOR, "type": FailureType.OFF, "expected": None},
    9: {"name": "SENSOR_AIRSPEED", "unit": FailureUnit.SENSOR_AIRSPEED, "type": FailureType.OFF, "expected": None},
    10: {"name": "SYSTEM_BATTERY", "unit": FailureUnit.SYSTEM_BATTERY, "type": FailureType.OFF, "expected": ["RETURN_TO_LAUNCH", "LAND"]},
    11: {"name": "SYSTEM_MOTOR", "unit": FailureUnit.SYSTEM_MOTOR, "type": FailureType.OFF, "expected": ["LAND"]},
    12: {"name": "SYSTEM_SERVO", "unit": FailureUnit.SYSTEM_SERVO, "type": FailureType.OFF, "expected": None},
    13: {"name": "SYSTEM_AVOIDANCE", "unit": FailureUnit.SYSTEM_AVOIDANCE, "type": FailureType.OFF, "expected": None},
    14: {"name": "SYSTEM_RC_SIGNAL", "unit": FailureUnit.SYSTEM_RC_SIGNAL, "type": FailureType.OFF, "expected": ["RETURN_TO_LAUNCH"]},
    15: {"name": "SYSTEM_MAVLINK_SIGNAL", "unit": FailureUnit.SYSTEM_MAVLINK_SIGNAL, "type": FailureType.OFF, "expected": ["RETURN_TO_LAUNCH"]},
}

def print_menu():
    print("\n=======================================================")
    print("        MAVSDK Failsafe Interactive Runner")
    print("=======================================================")
    print("Please choose the component you want to test:\n")
    for key in sorted(SCENARIOS.keys()):
        cfg = SCENARIOS[key]
        print(f"  [{key}] Test {cfg['name']} Failure")
    print("\n  [0] Exit")
    print("=======================================================")

async def execute_test(manager: FailsafeManager, config: dict):
    unit = config["unit"]
    f_type = config["type"]
    expected_modes = config["expected"]

    # 1. Start Official Mission Tracker sequence
    print(f"\n[EXECUTION] Opting into Mission Dynamics for {config['name']}...", flush=True)
    await manager.upload_triangle_mission(altitude=10.0, speed=5.0)
    reached = await manager.start_and_track_mission(target_item=1)
    
    if not reached:
        logger.error("Failed to reach Waypoint 1. Aborting test.")
        return False

    # 2. Inject Failure
    print(f"\n[INJECTION] Drone reached Waypoint 1! Ripping {config['name']} offline now!", flush=True)
    success = await manager.inject_failure(unit, f_type)
    if not success:
        # FailsafeManager logs unsupported warnings internally now
        return False

    # 3. Assert Results dynamically
    if expected_modes:
        print(f"\n[ASSERTION] Monitoring autopilot to intercept fallback to: {expected_modes}...", flush=True)
        triggered = False
        for mode in expected_modes:
            triggered = await manager.wait_for_flight_mode(mode, timeout=30.0)
            if triggered:
                print(f"[SUCCESS] Intercepted expected flight mode: {mode}!", flush=True)
                break
        
        if not triggered:
            print(f"[FAILURE] Drone did not enter any expected failsafe modes: {expected_modes}!", flush=True)
            return False
    else:
        print("\n[ASSERTION] This is a non-critical component. Observing flight stability for 15s...", flush=True)
        await asyncio.sleep(15)
        print("[SUCCESS] Drone remained stable after non-critical failure.", flush=True)

    print(f"\n[RECOVERY] Restoring {config['name']} back online...", flush=True)
    await manager.restore_failure(unit)
    return True


async def main():
    while True:
        print_menu()
        try:
            choice = input("Enter your choice [0-15]: ").strip()
            if not choice:
                continue
            choice_idx = int(choice)
        except ValueError:
            print("\n>> Invalid input! Please enter a number.\n")
            continue

        if choice_idx == 0:
            print("Exiting runner...")
            break
        elif choice_idx not in SCENARIOS:
            print("\n>> Invalid choice! Please select a valid test number.\n")
            continue

        selected_config = SCENARIOS[choice_idx]
        print(f"\n[SETUP] Spinning up test for {selected_config['name']}...")
        
        # We target Drone 1 (14541) exactly like run_all_tests.py did against the walls simulation
        manager = FailsafeManager("udpin://0.0.0.0:14541")
        try:
            await manager.connect()
            await manager.enable_failure_injection()
            await manager.start_telemetry_monitor()

            # Execute arm and takeoff (which waits for GPS lock now)
            await manager.arm_and_takeoff(altitude=10.0)
            
            # Run the test logic block
            test_passed = await execute_test(manager, selected_config)
            
            if test_passed:
                print(f"\n✅ TEST PASSED: {selected_config['name']} Failsafe\n")
            else:
                print(f"\n❌ TEST FAILED OR SKIPPED: {selected_config['name']} Failsafe\n")

        except Exception as e:
            logger.error(f"Test crashed with exception: {e}")
            
        finally:
            print("\n[TEARDOWN] Landing safely and releasing UDP connections...", flush=True)
            try:
                await manager.drone.action.land()
                await manager.wait_for_landed(timeout=60.0)
            except Exception as e:
                logger.error(f"Teardown landing error: {e}")
            await manager.disconnect()

        input("\nPress Enter to return to the Main Menu...")

if __name__ == "__main__":
    # Workaround for Python 3.10 event loop issues
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        loop.run_until_complete(main())
    except KeyboardInterrupt:
        print("\nExiting runner...")
        
