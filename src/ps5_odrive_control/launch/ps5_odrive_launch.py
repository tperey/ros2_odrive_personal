from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    odrive = Node(
        package='ps5_odrive_control',  # your package name
        executable='ps5_odrive_node',  # your node entrypoint
        name='ps5_odrive_node',        # optional ROS node name
        output='screen',               # print logs to screen
        parameters=[{
            # optional: any ROS 2 parameters you want to pass
            # 'V_MAX': 10.0,
            # 'some_other_param': 5
        }],
        remappings=[
            # optional: topic remapping if needed
            # ('left_stick_y', '/joy/left_stick_y')
        ]
    )
    ctrlnode = Node(
        package='ps5_odrive_control',  # your package name
        executable='ps5_controller_node',  # your node entrypoint
        name='ps5_odrive_node',        # optional ROS node name
        output='screen',               # print logs to screen
        parameters=[{
            # optional: any ROS 2 parameters you want to pass
            # 'V_MAX': 10.0,
            # 'some_other_param': 5
        }],
        remappings=[
            # optional: topic remapping if needed
            # ('left_stick_y', '/joy/left_stick_y')
        ]
    )

    delayctrl = TimerAction(period = 5.0, actions = [ctrlnode])
    
    return LaunchDescription([
        odrive,
        delayctrl
    ])
