For the simulation, you need all 4 things running simultaneously in separate host terminals:

Terminal 1 — PX4:
cd ~/Projects/AeroSAR/AeroSAR-Autonomous-Search-and-Rescue-Drone/sim/PX4-Autopilot
make px4_sitl none_iris

Terminal 2 — AirSim:
cd ~/Projects/AeroSAR/AirSim_Blocks/Blocks_packaged_Linux_55_33/Linux
./Blocks.sh -windowed -ResX=1280 -ResY=720

Terminal 3 — MAVROS:
cd ~/Projects/AeroSAR/AeroSAR-Autonomous-Search-and-Rescue-Drone/docker
docker compose up mavros

Terminal 4 — Your node (VS Code devcontainer terminal):
bashsource /opt/ros/humble/setup.bash && source install/setup.bash
python3 src/2_control/px4_interface/px4_interface/offboard_hover_mavros.py