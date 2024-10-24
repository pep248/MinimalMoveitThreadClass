import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    

    return LaunchDescription([
        Node(
            package='minimal_moveit_thread_class',
            executable='moveit_node',
            name='moveit_node',
            output='screen'
        )
    ])
    
    
