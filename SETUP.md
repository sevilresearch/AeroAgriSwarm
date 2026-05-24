# Setup Guide 

-----------------------------------------------------------------------------------------------------

#### Motive Opitrack Setting

![Alt text](image.png)

-----------------------------------------------------------------------------------------------------

#### File Modification

1. `src/crazyflie_missions/config/grid.yaml`: Flight altitude, agent names, agent ID numbers, ROS topic
   
2. `src/crazyflie_missions/crazyflie_missions/waypoint_tests/crazyflie_bridge_multi.py`: Agent names, radio URIs,
flight altitude (line 76)

3. `src/crazyflie_missions/crazyflie_missions/waypoint_tests/waypoint_manager_multi.py`: Agent names
> <b>Note</b>: You do not have to change the mode (waypoint vs. agriswarm algorithm), because it will automatically determine upon running `src/crazyflie_missions/crazyflie_missions/agriswarm/agriswarm_node.py`. 

4. `src/crazyflie_missions/crazyflie_missions/agriswarm/agriswarm_node.py`: Agent names, ROS topic, grid size (line 66), agent starting position (line 72)

5. `src/crazyflie_missions/crazyflie_missions/agriswarm/main.py`: Grid size (line 12, 58, 68), number of agents (line 36), agent starting position (line 38)

6. `src/mocap_optitrack/config/mocap.yaml`: Multicast address, IP address, agent ID numbers, agent names

7. `src/mocap_optitrack/include/mocap_optitrack/socket.h`: Multicast address

8. `src/mocap_optitrack/launch/mocap.launch.py`: Agent names (if you decide to use urdfs in rviz)

9. `src/mocap_optitrack/src/mocap_config.cpp`: Multicast address, port number

10. `src/mocap_optitrack/urdf`: Add or remove urdfs
