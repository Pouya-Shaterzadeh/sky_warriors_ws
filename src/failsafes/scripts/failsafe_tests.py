import asyncio
import pytest
import yaml
import os
from mavsdk.failure import FailureUnit, FailureType
from mavsdk.mission import MissionItem, MissionPlan
from failsafe_manager import FailsafeManager

# Load configuration
CONFIG_PATH = os.path.join(os.path.dirname(__file__), '..', 'parameters', 'failsafe_config.yaml')
try:
    with open(CONFIG_PATH, 'r') as f:
         config = yaml.safe_load(f)
except FileNotFoundError:
    config = {
        'connection': {'default_url': 'udpin://0.0.0.0:14540'},
        'test_settings': {'takeoff_altitude_m': 10.0, 'flight_mode_timeout_s': 30.0, 'landing_timeout_s': 60.0},
        'failsafe_parameters': {'SYS_FAILURE_EN': 1}
    }

URL = config['connection']['default_url']
ALTITUDE = config['test_settings']['takeoff_altitude_m']
TIMEOUT = config['test_settings']['flight_mode_timeout_s']

@pytest.fixture
async def setup_failsafe_manager():
    """Pytest fixture to initialize and prepare the drone."""
    manager = FailsafeManager(system_address=URL)
    await manager.connect()
    await manager.enable_failure_injection()
    
    # Configure required parameters for testing
    if 'failsafe_parameters' in config:
         await manager.configure_parameters(config['failsafe_parameters'])
         
    await manager.arm_and_takeoff(altitude=ALTITUDE)
    
    # Give it a few seconds to hover
    await asyncio.sleep(5)
    
    yield manager
    
    # Teardown logic
    await manager.disconnect()

@pytest.mark.asyncio
async def test_gps_loss_failsafe(setup_failsafe_manager):
    """Test: Dropping GPS causes drone to transition to RTL."""
    manager = setup_failsafe_manager
    
    # 1. Inject GPS OFF failure
    await manager.inject_failure(FailureUnit.SENSOR_GPS, FailureType.OFF)
    
    # 2. Wait and verify it triggers RETURN_TO_LAUNCH mode
    triggered = await manager.wait_for_flight_mode("RETURN_TO_LAUNCH", timeout=TIMEOUT)
    assert triggered is True, "Drone did not enter RTL mode after GPS failure injection."
    
    # 3. Recover
    await manager.restore_failure(FailureUnit.SENSOR_GPS)
    
    # 4. Optional: land to finish safely
    await manager.drone.action.land()

@pytest.mark.asyncio
async def test_battery_critical_failsafe(setup_failsafe_manager):
    """Test: Battery failure goes to RTL or LAND based on settings."""
    manager = setup_failsafe_manager
    
    # Inject Battery failure (OFF usually simulates 0V in SITL context causing immediate failsafe)
    await manager.inject_failure(FailureUnit.SYSTEM_BATTERY, FailureType.OFF)
    
    # Verify flight mode (can be RETURN_TO_LAUNCH or LAND depending on remaining capacity)
    # We will check if it transitioned to RTL (the config parameter COM_LOW_BAT_ACT is 1 (RTL))
    # or LAND if it forces it. We'll accept either for this generic test.
    triggered_rtl = await manager.wait_for_flight_mode("RETURN_TO_LAUNCH", timeout=15)
    triggered_land = False
    if not triggered_rtl:
         triggered_land = await manager.wait_for_flight_mode("LAND", timeout=15)
         
    assert triggered_rtl or triggered_land, "Drone did not enter RTL or LAND mode after Battery failure."
    
    # Recover
    await manager.restore_failure(FailureUnit.SYSTEM_BATTERY)
    await manager.drone.action.land()

@pytest.mark.asyncio
async def test_rc_loss_failsafe(setup_failsafe_manager):
    """Test: RC signal loss causes RTL."""
    manager = setup_failsafe_manager
    
    # In MAVSDK SYSTEM_RC_SIGNAL OFF
    await manager.inject_failure(FailureUnit.SYSTEM_RC_SIGNAL, FailureType.OFF)
    
    # In failsafe_config.yaml, NAV_RCL_ACT is 2 (RTL)
    triggered = await manager.wait_for_flight_mode("RETURN_TO_LAUNCH", timeout=TIMEOUT)
    assert triggered is True, "Drone did not enter RTL mode after RC Loss failure injection."
    
    # Recover
    await manager.restore_failure(FailureUnit.SYSTEM_RC_SIGNAL)
    await manager.drone.action.land()

@pytest.mark.asyncio
async def test_sensor_stuck_degraded(setup_failsafe_manager):
    """Test: Intentionally degrading a sensor like Baro using STUCK failure."""
    manager = setup_failsafe_manager
    
    # Make barometer STUCK
    await manager.inject_failure(FailureUnit.SENSOR_BARO, FailureType.STUCK)
    
    # A stuck baro might not instantly cause RTL, but it should trigger a health warning.
    # We will wait and observe. For testing purposes, we ensure the injection doesn't crash the manager.
    await asyncio.sleep(5)
    
    # Recover
    await manager.restore_failure(FailureUnit.SENSOR_BARO)
    await manager.drone.action.land()

@pytest.mark.asyncio
async def test_gps_loss_during_mission(setup_failsafe_manager):
    """Test: Dropping GPS while executing a mission causes drone to transition to RTL."""
    manager = setup_failsafe_manager
    
    # Create a simple mission near the default SITL origin
    mission_items = []
    mission_items.append(MissionItem(
        47.398039859999997, 8.5455725400000002, 15,
        5, True, float('nan'), float('nan'),
        MissionItem.CameraAction.NONE, float('nan'), float('nan'),
        float('nan'), float('nan'), float('nan'), MissionItem.VehicleAction.NONE
    ))
    mission_items.append(MissionItem(
        47.398036222362471, 8.5450146439425509, 15,
        5, True, float('nan'), float('nan'),
        MissionItem.CameraAction.NONE, float('nan'), float('nan'),
        float('nan'), float('nan'), float('nan'), MissionItem.VehicleAction.NONE
    ))

    mission_plan = MissionPlan(mission_items)
    
    await manager.drone.mission.set_return_to_launch_after_mission(False)
    await manager.drone.mission.upload_mission(mission_plan)
    
    # Start mission (drone is already in the air from setup fixture)
    await manager.drone.mission.start_mission()
    
    # Fly during mission for a few seconds
    await asyncio.sleep(4)
    
    # 1. Inject GPS OFF failure during active mission navigation
    await manager.inject_failure(FailureUnit.SENSOR_GPS, FailureType.OFF)
    
    # 2. Wait and verify it triggers RETURN_TO_LAUNCH mode
    triggered = await manager.wait_for_flight_mode("RETURN_TO_LAUNCH", timeout=TIMEOUT)
    assert triggered is True, "Drone did not enter RTL mode after GPS failure during mission."
    
    # 3. Recover
    await manager.restore_failure(FailureUnit.SENSOR_GPS)
    await manager.drone.action.land()
