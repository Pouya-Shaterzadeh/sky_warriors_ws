import asyncio
import logging
from typing import Dict, Any, Optional

from mavsdk import System
from mavsdk.failure import FailureUnit, FailureType

logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(levelname)s] %(message)s')
logger = logging.getLogger("FailsafeManager")

class FailsafeManager:
    """Core class for managing MAVSDK failsafe testing operations."""
    
    def __init__(self, system_address: str = "udpin://0.0.0.0:14540"):
        self.drone = System()
        self.system_address = system_address
        self.is_connected = False
        self._telemetry_task = None

    async def connect(self, timeout: float = 30.0):
        """Connects to the drone and waits for a global position estimate."""
        logger.info(f"Connecting to PX4 at {self.system_address}...")
        await self.drone.connect(system_address=self.system_address)

        try:
            async with asyncio.timeout(timeout):
                logger.info("Waiting for drone to connect...")
                async for state in self.drone.core.connection_state():
                    if state.is_connected:
                        logger.info("Drone connected!")
                        self.is_connected = True
                        break

                logger.info("Waiting for drone to have a global position estimate...")
                async for health in self.drone.telemetry.health():
                    if health.is_global_position_ok and health.is_home_position_ok:
                        logger.info("Global position estimate OK.")
                        break
        except asyncio.TimeoutError:
            logger.error("Connection or GPS health check timed out.")
            raise

    async def configure_parameters(self, params: Dict[str, Any]):
        """Sets multiple PX4 parameters."""
        logger.info(f"Configuring {len(params)} failsafe parameters...")
        for key, value in params.items():
            try:
                if isinstance(value, float):
                    await self.drone.param.set_param_float(key, value)
                elif isinstance(value, int):
                    await self.drone.param.set_param_int(key, value)
                logger.debug(f"Parameter Set: {key} = {value}")
            except Exception as e:
                logger.error(f"Failed to set parameter {key}: {e}")

    async def enable_failure_injection(self):
        """Checks and enables the SYS_FAILURE_EN parameter."""
        logger.info("Checking SYS_FAILURE_EN parameter...")
        try:
            result = await self.drone.param.get_param_int("SYS_FAILURE_EN")
            if result == 0:
                logger.warning("SYS_FAILURE_EN is disabled. Enabling it...")
                await self.drone.param.set_param_int("SYS_FAILURE_EN", 1)
                logger.info("SYS_FAILURE_EN enabled.")
            else:
                logger.info("SYS_FAILURE_EN is already enabled.")
        except Exception as e:
             logger.error(f"Error checking/setting SYS_FAILURE_EN parameter: {e}")
             raise

    async def arm_and_takeoff(self, altitude: float = 10.0):
        """Arms the drone and takes off to a specific altitude."""
        logger.info("Arming drone...")
        await self.drone.action.arm()
        
        logger.info(f"Taking off to {altitude}m...")
        await self.drone.action.set_takeoff_altitude(altitude)
        await self.drone.action.takeoff()

        # Wait to reach altitude
        logger.info(f"Waiting to reach {altitude}m altitude...")
        async for position in self.drone.telemetry.position():
            if position.relative_altitude_m >= (altitude - 1.0):
                logger.info(f"Reached altitude: {position.relative_altitude_m:.2f}m")
                break
            await asyncio.sleep(0.5)
            
    async def inject_failure(self, unit: FailureUnit, failure_type: FailureType, instance: int = 0):
        """Injects a specific component failure."""
        logger.info(f"INJECTING FAILURE: Unit={unit.name}, Type={failure_type.name}, Instance={instance}")
        try:
            # We timeout because if SYS_FAILURE_EN is false, this call might block forever
            async with asyncio.timeout(5.0):
                await self.drone.failure.inject(unit, failure_type, instance)
            logger.info("Failure injected successfully.")
        except asyncio.TimeoutError:
             logger.error("Failure injection TIMED OUT. Check if SYS_FAILURE_EN is 1.")
             raise
        except Exception as e:
             logger.error(f"Failure injection error: {e}")
             raise

    async def restore_failure(self, unit: FailureUnit, instance: int = 0):
        """Restores a component back to OK status."""
        logger.info(f"RESTORING FAILURE for Unit={unit.name}")
        await self.inject_failure(unit, FailureType.OK, instance)

    async def wait_for_flight_mode(self, target_mode_name: str, timeout: float = 30.0) -> bool:
        """Waits for a specific flight mode (e.g., 'RETURN_TO_LAUNCH', 'LAND')."""
        logger.info(f"Waiting for flight mode: {target_mode_name} (Timeout: {timeout}s)...")
        try:
            async with asyncio.timeout(timeout):
                async for flight_mode in self.drone.telemetry.flight_mode():
                    logger.debug(f"Current Flight Mode: {flight_mode.name}")
                    if flight_mode.name.upper() == target_mode_name.upper():
                        logger.info(f"Target flight mode '{target_mode_name}' reached!")
                        return True
        except asyncio.TimeoutError:
            logger.error(f"Timed out waiting for flight mode: {target_mode_name}")
            return False
        return False

    async def wait_for_landed(self, timeout: float = 60.0) -> bool:
        """Waits until the drone detects it is landed."""
        logger.info(f"Waiting for drone to land (Timeout: {timeout}s)...")
        try:
            async with asyncio.timeout(timeout):
                 async for landed_state in self.drone.telemetry.landed_state():
                      logger.debug(f"Landed State: {landed_state.name}")
                      if landed_state.name == "ON_GROUND":
                          logger.info("Drone is landed.")
                          return True
        except asyncio.TimeoutError:
            logger.warning("Timed out waiting for landing.")
            return False
        return False
        
    async def start_telemetry_monitor(self):
        """Starts a background task to log vital telemetry."""
        async def _log_telemetry():
            try:
                async for position in self.drone.telemetry.position():
                     # Only log every few seconds or as needed, simplified for testing
                     await asyncio.sleep(2)
            except asyncio.CancelledError:
                 pass
                 
        if not self._telemetry_task:
            self._telemetry_task = asyncio.create_task(_log_telemetry())

    async def stop_telemetry_monitor(self):
         if self._telemetry_task:
             self._telemetry_task.cancel()
             self._telemetry_task = None

    async def disconnect(self):
        """Cleanup."""
        await self.stop_telemetry_monitor()
        # No explicit disconnect method in contemporary python MAVSDK, but clean up references if needed
        self.is_connected = False
        logger.info("Disconnected from FailsafeManager.")
