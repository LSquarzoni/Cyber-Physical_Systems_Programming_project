#!/usr/bin/env python3

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
    # Get the package directory
    package_dir = get_package_share_directory('px4_offboard')

    with open('/home/cpsuser/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris_depth_camera/iris_depth_camera.urdf', 'r') as infp:
        robot_desc = infp.read()
    params = {'robot_description': robot_desc}
    
    # Define the URDF file name and its default path
    urdf_file_name = 'urdf/iris_depth_camera.urdf'
    urdf_default_path = os.path.join(package_dir, urdf_file_name)

    # Declare the launch argument for URDF path
    declare_urdf_path_cmd = DeclareLaunchArgument(
        name='urdf_path',
        default_value=urdf_default_path,
        description='Absolute path to robot URDF file'
    )

    # Command to parse the URDF file using xacro
    robot_description_content = Command(
        ['xacro ', LaunchConfiguration('urdf_path')]
    )

    # Node for robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Node for joint state publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # Nodes for various functionalities
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
        prefix='gnome-terminal --'
    )

    vel_control = Node(
        package='px4_offboard',
        namespace='px4_offboard',
        executable='velocity_control',
        name='velocity_control'
    )

    rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(package_dir, 'visualize.rviz')]
    )

    map_publisher = Node(
        package='map_frame_publisher',
        executable='map_frame_publisher',
        name='map_publisher'
    )
    
    map_base_link_publisher = Node(
        package='map_base_link_publ',
        executable='map_base_link_publ',
        name='map_base_link_publisher'
    )

    # Return the LaunchDescription with all nodes and launch arguments
    return LaunchDescription([
        declare_urdf_path_cmd,
        robot_state_publisher_node,
        joint_state_publisher_node,
        visualize,
        processes,
        control,
        vel_control,
        rviz,
        map_publisher,
        map_base_link_publisher
    ])

