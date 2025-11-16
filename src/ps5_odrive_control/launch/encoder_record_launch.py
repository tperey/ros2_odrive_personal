#!/usr/bin/env python3
import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():

    """ Nodes """
    encoder = Node(
        package='ps5_odrive_control',  # your package name
        executable='encoder_node',     # your node entrypoint
        name='encoder_node',        # optional ROS node name
        output='screen',               # print logs to screen
        parameters=[{
            # optional
        }],
        remappings=[
            # optional
        ]
    )

    """ Record """
    # Create a timestamp string
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    # Define folder where the bag will be saved
    bag_folder = f"/home/trevorperey/rosbags/ros2bag_{timestamp}"
    
    # Topics to record (adjust as needed)
    topics = ["/encoder_furata"]  # Example topic

    # Build the command for ros2 bag record
    record_cmd = ["ros2", "bag", "record", "-o", bag_folder] + topics

    return LaunchDescription([
        encoder,
        ExecuteProcess(
            cmd=record_cmd,
            output="screen"
        )
    ])
