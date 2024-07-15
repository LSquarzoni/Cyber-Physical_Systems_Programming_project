#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

__author__ = "Braden Wagstaff"
__contact__ = "braden@arkelectron.com"

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_dir = get_package_share_directory('px4_offboard')
    urdf_file_name = 'urdf/iris_depth_camera.urdf'
    pack_path = os.path.expanduser('~/drone_ws/src/px4_offboard/')
    # urdf_default_path = os.path.join(package_dir, urdf_file_name)

    urdf_default_path = os.path.join(pack_path, urdf_file_name)

    declare_urdf_path_cmd = DeclareLaunchArgument(
        name='urdf_path',
        default_value=urdf_default_path,
        description='Absolute path to robot urdf file'
    )

    robot_description_content = Command(
        ['xacro ', LaunchConfiguration('urdf_path')]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    drone_pose_package_path = FindPackageShare(package='drone_pose').find('drone_pose')
    drone_pose_launch_file_path = os.path.join(drone_pose_package_path, 'launch', 'drone_pose.launch.py')

    drone_pose_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(drone_pose_launch_file_path)
    )
    # bash_script_path = os.path.join(package_dir, 'scripts', 'TerminatorScript.sh')
    # ExecuteProcess(cmd=['bash', bash_script_path], output='screen'),
    visualize = Node(
        package='px4_offboard',
        namespace='px4_offboard',
        executable='visualizer',
        name='visualizer'
    )
    processes = Node(
        package='px4_offboard',
        namespace='px4_offboard',
        executable='processes',
        name='processes',
        prefix='gnome-terminal --'
    )
    control = Node(
        package='px4_offboard',
        namespace='px4_offboard',
        executable='control',
        name='control',
        prefix='gnome-terminal --',
    )
    vel_control = Node(
        package='px4_offboard',
        namespace='px4_offboard',
        executable='velocity_control',
        name='velocity'
    )
    rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', [os.path.join(package_dir, 'visualize.rviz')]]
    )

    map_publisher = Node(
        package='map_frame_publisher',
        namespace='map_frame_publisher',
        executable='map_frame_publisher',
        name='map_publisher'
    )
    return LaunchDescription([
        declare_urdf_path_cmd,
        robot_state_publisher_node,
        joint_state_publisher_node,
        drone_pose_launch,
        visualize,
        processes,
        control,
        vel_control,
        rviz,
        map_publisher,
    ])
