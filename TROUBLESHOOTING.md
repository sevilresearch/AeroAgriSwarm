# Troubleshooting Guide 

-----------------------------------------------------------------------------------------------------

#### Warning Issues:

1. If you see a warning that refers to unused variables in c++ file like `src/mocap_optitrack/src/natnet/natnet_messages.cpp`,
   then suprress the unused variables and ignore the warnings by adding `(void)variable_name;` beneath it.

2. If you see a warning that refers to qos_overrides, determining default communication behavior, in file like
   `src/mocap_optitrack/src/mocap_node.cpp`, then ignore the warning by writing the following code:

  ```
  // Ignore any other parameters (like qos_overrides)
  RCLCPP_INFO(node->get_logger(),
              "Ignoring non-dynamic parameter: %s",
              param.get_name().c_str());
  ```

-----------------------------------------------------------------------------------------------------

#### `~/.bashrc` File Modification:
1. nano ~/.bashrc:  (Source ~/.bashrc after updating it)
2. Set up ROS 2 Humble Environment
    * source /opt/ros/humble/setup.bash
3. Gets rid of the Cache path issue (Could not save cache, no writable directory)
    * export CFLIB_CACHE=~/.cache/cflib
    * mkdir -p ~/.cache/cflib
    * chmod 777 ~/.cache/cflib
4. Suppressed the “RTPS_TRANSPORT_SHM Error”
    * sudo apt install ros-humble-rmw-cyclonedds-cpp
    * export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    * echo ‘export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp’ >> ~/.bashrc 
