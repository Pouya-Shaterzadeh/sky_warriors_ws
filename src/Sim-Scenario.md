# Sky Warrior Simulation Scenario 

The Scenario You Should Demonstrate (Exactly Matches the “Long Task”)
Use your current world.sdf {src/skyw_bringup/world} (hexagon with 6 QR walls + red/blue/start landing pads). The mission flow:

All 3 drones takeoff from starting area (blue point).
Drone 1 (the one with mono_cam) autonomously flies a short scanning path in front of the 6 QR walls.
QR detector reads one (or more) QR codes → publishes e.g. "formation: diamond" + "target_pad: blue" + "goal: 10, -5".
Task allocator (your CBBA) assigns the task to the swarm.
Swarm instantly changes formation (e.g. line → diamond or triangle).
Swarm flies to the decoded goal point.
All drones land on the correct colored pad (red or blue) using your color detector + precise landing logic.



for launch file i get
ros2 launch skyw_swarm swarm_mission_scenario.launch.py \
  use_sim_time:=true \
  takeoff_z:=-2.5 \
  wall_x:=5.0 wall_y:=0.0 wall_z:=-1.0 wall_yaw:=1.57
[INFO] [launch]: All log files can be found below /home/roboticistprogrammer/.ros/log/2026-04-03-15-35-49-942444-amir-ykz-34406
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [px4_pose_bridge.py-1]: process started with pid [34407]
[INFO] [mission_sequencer.py-2]: process started with pid [34409]
[INFO] [px4_offboard_bridge.py-3]: process started with pid [34411]
[mission_sequencer.py-2] [INFO] [1775219750.579273286] [mission_sequencer]: Mission sequencer started for QR wall mission.
[px4_pose_bridge.py-1] [INFO] [1775219750.659459931] [px4_pose_bridge]: Subscribed to /px4_1/fmu/out/vehicle_local_position_v1 for drone1
[px4_pose_bridge.py-1] [INFO] [1775219750.711827885] [px4_pose_bridge]: Subscribed to /px4_2/fmu/out/vehicle_local_position_v1 for drone2
[px4_pose_bridge.py-1] [INFO] [1775219750.713185587] [px4_pose_bridge]: Subscribed to /px4_3/fmu/out/vehicle_local_position_v1 for drone3
[px4_pose_bridge.py-1] [INFO] [1775219750.714167576] [px4_pose_bridge]: PX4 Pose Bridge started for 3 drones
[px4_offboard_bridge.py-3] [INFO] [1775219750.717198435] [px4_offboard_bridge]: PX4 offboard bridge ready.


but drones do not arm and take off to move, i am running simulation now
check avalable nodes and topics to see what is missing
by the way i cant see offboard in qgc modes
