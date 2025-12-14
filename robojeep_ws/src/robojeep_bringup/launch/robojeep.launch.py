#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('robojeep_bringup')
    
    # Single config file path
    config_file = os.path.join(pkg_dir, 'config', 'robojeep.yaml')
    
    return LaunchDescription([
        # Joystick node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[config_file],
            output='screen'
        ),
        
        # Teleop twist joy
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            parameters=[config_file],
            output='screen'
        ),
        
        # Base node (differential steering)
        Node(
            package='robojeep_base',
            executable='base_node_differential',
            name='robojeep_base_differential',
            parameters=[config_file],
            output='screen'
        ),
        
        # Arduino bridge (calibrated)
        Node(
            package='robojeep_arduino_bridge',
            executable='arduino_bridge_calibrated',
            name='arduino_bridge_calibrated',
            parameters=[config_file],
            output='screen'
        ),
    ])
