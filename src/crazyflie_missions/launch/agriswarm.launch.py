from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    mocap_launch = os.path.join(
        get_package_share_directory('mocap_optitrack'),
        'launch',
        'mocap.launch.py'
    )

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mocap_launch)
        ),

        Node(
            package='crazyflie_missions',
            executable='crazyflie_bridge_multi',
            output='screen'
        ),

        Node(
            package='crazyflie_missions',
            executable='waypoint_manager_multi',
            output='screen'
        ),

        Node(
            package='crazyflie_missions',
            executable='agriswarm_grid_adapter',
            output='screen'
        ),

        Node(
            package='crazyflie_missions',
            executable='agriswarm_node',
            output='screen'
        ),
    ])
