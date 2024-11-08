import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    sim = LaunchConfiguration('sim')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    
    launch_description = [
        DeclareLaunchArgument(
            name='sim', 
            default_value='true', 
            description='Simulation mode'),
        DeclareLaunchArgument(
            name='rviz_config_file', 
            default_value=PathJoinSubstitution([FindPackageShare("minimal_moveit_thread_class"), "rviz", 'view_robot.rviz']), 
            description='Path to the RViz config file'),
    ]
    
    # Launch MoveIt
    # we are using a custom moveit launch file just because we want to use a custom rviz config file if we use
    # the original moveit launch file, it will still work but we will have to manually add the RvizVisualTools panel
    launch_description.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource( 
                PathJoinSubstitution([FindPackageShare("minimal_moveit_thread_class"), "launch", "launch_custom_ur_moveit.launch.py"])),
            launch_arguments={
                "ur_type": "ur10e",
                "use_sim_time": sim,
                "launch_rviz": "true",
                "rviz_config_file": rviz_config_file
            }.items()
        )
    )

    # UR Control - Real Robot or Simulation
    if sim == 'false':
        launch_description.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"])),
                launch_arguments={
                    "ur_type": "ur10e",
                    "robot_ip": "192.168.1.10",
                    "use_fake_hardware": "false",
                    "initial_joint_controller": "scaled_joint_trajectory_controller"
                }.items()
            )
        )
    else:
        launch_description.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("ur_simulation_gz"), "launch", "ur_sim_control.launch.py"])),
                launch_arguments={
                    "ur_type": "ur10e",
                    "safety_limits": "true",
                    "launch_rviz" : "false",
                }.items()
            )
        )

    # Launch moveit routine
    launch_description.append(
        Node(
            package='minimal_moveit_thread_class',
            executable='moveit_thread_node',
            name='moveit_thread_node',
            parameters=[{'use_sim_time': sim}],
            output='screen'
        )
    )

    return LaunchDescription(launch_description)
