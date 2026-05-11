# Copyright (c) 2019, Samsung Electronics Inc., Vinnam Kim
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('mocap_optitrack')
    config_file = os.path.join(pkg_share, 'config', 'mocap.yaml')
    rviz_config_file = os.path.join(pkg_share, 'config', 'mocap_view.rviz')

    # Mocap node (one instance handles all rigid bodies)
    mocap_node = Node(
        package='mocap_optitrack',
        executable='mocap_node',
        name='mocap_node',
        parameters=[config_file],
        output='screen',
        arguments=['--ros-args', '--log-level', 'error'],
    )

    # Rviz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
    )

    # Robot_1 (Change names 8 times)
    urdf_robot_1 = os.path.join(pkg_share, 'urdf', 'crazyflie_description_1.urdf')
    with open(urdf_robot_1, 'r') as f:
        robot_1_desc = f.read()

    robot_1_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_robot_1',
        parameters=[
            {'robot_description': robot_1_desc},
            {'frame_prefix': 'Robot_1/'}
        ],
        output='screen',
    )

    # Robot_2 (Change names 8 times)
    urdf_robot_2 = os.path.join(pkg_share, 'urdf', 'crazyflie_description_2.urdf')
    with open(urdf_robot_2, 'r') as f:
        robot_2_desc = f.read()

    robot_2_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_robot_2',
        parameters=[
            {'robot_description': robot_2_desc},
            {'frame_prefix': 'Robot_2/'}
        ],
        output='screen',
    )

    # Robot_3 (Change names 8 times)
    urdf_robot_3 = os.path.join(pkg_share, 'urdf', 'crazyflie_description_3.urdf')
    with open(urdf_robot_3, 'r') as f:
        robot_3_desc = f.read()

    robot_3_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_robot_3',
        parameters=[
            {'robot_description': robot_3_desc},
            {'frame_prefix': 'Robot_3/'}
        ],
        output='screen',
    )

    return LaunchDescription([
        mocap_node,
        rviz_node,
        robot_1_state_pub,
        robot_2_state_pub,
        robot_3_state_pub
    ])


generate_launch_description()



