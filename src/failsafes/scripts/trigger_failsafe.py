#!/usr/bin/env python3
import asyncio
import argparse
import sys
from mavsdk import System
from mavsdk.failure import FailureUnit, FailureType

async def run():
    parser = argparse.ArgumentParser(description="Trigger failures manually on a running PX4 SITL.")
    parser.add_argument("--unit", required=True, choices=["gps", "battery", "rc", "baro", "mag", "motor"],
                        help="The sensor/system to fail.")
    parser.add_argument("--type", default="off", choices=["ok", "off", "stuck", "garbage", "wrong", "slow", "delayed", "intermittent"],
                        help="The type of failure. Use 'ok' to restore.")
    parser.add_argument("--url", default="udpin://0.0.0.0:14540", help="Connection URL (default: udpin://0.0.0.0:14540)")

    args = parser.parse_args()

    # Map generic string arguments to MAVSDK Enums
    unit_map = {
        "gps": FailureUnit.SENSOR_GPS,
        "battery": FailureUnit.SYSTEM_BATTERY,
        "rc": FailureUnit.SYSTEM_RC_SIGNAL,
        "baro": FailureUnit.SENSOR_BARO,
        "mag": FailureUnit.SENSOR_MAG,
        "motor": FailureUnit.SYSTEM_MOTOR
    }

    type_map = {
        "ok": FailureType.OK,
        "off": FailureType.OFF,
        "stuck": FailureType.STUCK,
        "garbage": FailureType.GARBAGE,
        "wrong": FailureType.WRONG,
        "slow": FailureType.SLOW,
        "delayed": FailureType.DELAYED,
        "intermittent": FailureType.INTERMITTENT
    }

    unit_enum = unit_map[args.unit.lower()]
    type_enum = type_map[args.type.lower()]

    print(f"Connecting to PX4 at {args.url}...")
    drone = System()
    await drone.connect(system_address=args.url)

    # Wait for connection
    is_connected = False
    try:
        async with asyncio.timeout(10.0):
            async for state in drone.core.connection_state():
                if state.is_connected:
                    is_connected = True
                    print("Connected!")
                    break
    except asyncio.TimeoutError:
        print("Error: Could not connect to drone. Check if PX4 is running.")
        sys.exit(1)

    # Make sure SYS_FAILURE_EN is on
    try:
        sys_failure_en = await drone.param.get_param_int("SYS_FAILURE_EN")
        if sys_failure_en == 0:
            print("Enabling SYS_FAILURE_EN parameter...")
            await drone.param.set_param_int("SYS_FAILURE_EN", 1)
    except Exception as e:
        print(f"Warning: Could not check/set SYS_FAILURE_EN parameter: {e}")

    # Inject
    if type_enum == FailureType.OK:
        print(f"Restoring {unit_enum.name} back to normal (OK)...")
    else:
        print(f"Injecting failure: {unit_enum.name} -> {type_enum.name}...")
        
    try:
        async with asyncio.timeout(5.0):
            await drone.failure.inject(unit_enum, type_enum, 0)
        print("Success! Check QGC or Gazebo to observe the drone's behavior.")
    except asyncio.TimeoutError:
        print("Error: Injection timed out. Ensure SYS_FAILURE_EN is 1 in PX4.")
    except Exception as e:
        print(f"Error during injection: {e}")

if __name__ == "__main__":
    asyncio.run(run())
