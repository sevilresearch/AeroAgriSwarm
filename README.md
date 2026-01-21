# Crazyflie 2.1 ROS 2 (Humble) Workspace
# cflib version 0.1.28

------------------------------------------------------------------------------------------------------

## 1. Source all terminal:

    source install/setup.bash 

## 2. GUI for multi-agent uav radio connection, firware updates, etc. Type this in terminal:
    
    cfclient or python3 -m cfclient.gui

## 3. Launch Rviz and mocap_optitrack package for visualization:

    ros2 launch mocap_optitrack mocap.launch.py

## 4. Run takeoff script:

    python3 src/crazyflie_missions/crazyflie_missions/waypoint_tests/takeoff.py

## 5. Run waypoint navigation without Motive Mocap Data:

    Terminal 1: ros2 run crazyflie_missions crazyflie_bridge
    Terminal 2: ros2 run crazyflie_missions waypoint_manager

## 6. Run waypoint navigation with Motive Mocap Data:

    Terminal 1: ros2 run crazyflie_missions crazyflie_bridge
    Terminal 2: ros2 run crazyflie_missions waypoint_manager_motive

## 7. Run parallel UAVs waypoint navigation using Motive Mocap Data:

    Terminal 1: ros2 run crazyflie_missions crazyflie_bridge_multi
    Terminal 2: ros2 run crazyflie_missions waypoint_manager_multi

## 8. Run parallel UAVs waypoint navigation using Lighthouse Position Data:

    Terminal 1: ros2 run crazyflie_missions crazyflie_bridge_lh
    Terminal 2: ros2 run crazyflie_missions waypoint_manager_multi
