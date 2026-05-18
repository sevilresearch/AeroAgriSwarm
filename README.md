# AeroAgriSwarm

-----------------------------------------------------------------------------------------------------
<p align="justify"><b>Project Overview:</b> This project is a pipeline for automous mobile application. 
The purpose of this research is to integrate decentralized crazyflie agents and agriculture simulation 
with real-time synchronization between waypoint data and error-handling updates. This application will 
allow future engineers to capture plant and soil information in a farm field with the help of A-star search 
algorithm and a local planner for a effcient cost-path planning exploration.</p>

-----------------------------------------------------------------------------------------------------
<b>File Overview:</b> Files to view before Quick Start
1. README.md: General outline of entire research
2. COMMAND.md: Terminal Commands for different flight programs
3. TROUBLELSHOOTING.md: Remove warnings and errors, and modify `.bashrc` file  
4. <mark>SETUP.md</mark>: Setup of Motive Optitrack streaming setting and locations of files to change
   crazyflie agent variables, flight altitude, radio URIs, IDs, and multicast IP address
   
-----------------------------------------------------------------------------------------------------
#### Prerequisites
  
  * Ubuntu 22.04 LTS (Jammy Jellyfish)
  * ROS 2 Humble
  * Python 3.10
     * Python libraries installation: `src/crazyflie_missions/setup.py`
  * Microsoft Visual Studio 2022
  * Motive Optitrack 3.0.0
  * Bitcraze Crazyflie 2.1 Bundle
  
-----------------------------------------------------------------------------------------------------
#### Quick Start
1. Clone this repository into your workspace
   
   ```
   git clone https://github.com/sevilresearch/AeroAgriSwarm.git
   ```

2. Install emoji icons for simulation (if necessary)

   To enable emoji-like visuals in simulation icons, install the Twitter Color Emoji SVG font:

   🔗 [Download Twemoji TTF](https://sourceforge.net/projects/twitt-c-emoji-svg-font.mirror/)

   Once downloaded, install the TTF file based on your operating system:

   * Windows: Right-click the .ttf file → Install
   * Mac: Double-click the .ttf file → Install Font
   * Linux: Copy the .ttf to ~/.fonts/ and run fc-cache -fv

   There already should be a TTF file under `src/crazyflie_missions/crazyflie_missions/agriswarm/` called                    `TwitterColorEmoji-SVGinOT.ttf`

3. Build Workspace

   ```
   colcon build
   ```
   
4. Source Workspace

   ```
   source install/setup.bash
   ```

5. Run Program

   Note: Make sure to look at the terminal path. 

   Terminal 1:

   ```
   ros2 run crazyflie_missions crazyflie_bridge_multi
   ```

   Terminal 2:

   ```
   ros2 run crazyflie_missions waypoint_manager_multi
   ```

   Terminal 3:

   ```
   ros2 run crazyflie_missions agriswarm_node
   ```

   Terminal 4:

   ```
   python3 main.py 
   ```
   
-----------------------------------------------------------------------------------------------------
 #### Demo

-----------------------------------------------------------------------------------------------------
 #### Related Work

 
 
