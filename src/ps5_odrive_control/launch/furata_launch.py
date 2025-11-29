from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction

import os
from datetime import datetime

def generate_launch_description():

    """ Node """
    controller = Node(
        package='ps5_odrive_control',  # your package name
        executable='furata_controller',  # your node entrypoint
        name='furata_controller',        # optional ROS node name
        output='screen',               # print logs to screen
        parameters=[{
            'doCal': False,
            'doErase': False,
            'doRecon': False
        }],
        remappings=[
            # optional: topic remapping if needed
            # ('left_stick_y', '/joy/left_stick_y')
        ]
    )

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

    delayctrl = TimerAction(period = 2.0, actions = [controller])

    """ Record """
    # Create a timestamp string
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    # Define folder where the bag will be saved
    bag_folder = f"/home/trevorperey/rosbags/ros2bag_{timestamp}"
    
    # Topics to record (adjust as needed)
    topics = ["/encoder_furata"]  # Example topic

    # Build the command for ros2 bag record
    #record_cmd = ["ros2", "bag", "record", "-o", bag_folder] + topics

    return LaunchDescription([
        encoder,
        delayctrl,
        # ExecuteProcess(
        #     cmd=record_cmd,
        #     output="screen"
        # )
    ])

