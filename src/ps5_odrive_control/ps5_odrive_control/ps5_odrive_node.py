import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from inputs import get_gamepad
from enum import Enum
import time

import odrive
from odrive.enums import * # Get them all
from ps5_odrive_control.common.playing_with_odrive import get_Odrive_init
from odrive.utils import request_state

class OdriveState(Enum):
    VELO_MODE = 0
    POS_MODE = 1

V_MAX = 4.0

class PS5OdriveNode(Node):
    def __init__(self):

        # General init
        super().__init__('ps5_odrive_node')
        self._odrive_state = OdriveState.VELO_MODE

        # Odrive
        self._odrv = get_Odrive_init(Vmax = V_MAX, ConType = ControlMode.VELOCITY_CONTROL)
        request_state(self._odrv.axis0, AxisState.CLOSED_LOOP_CONTROL)

        # Subscribers
        #self.button_sub = self.create_subscription(Bool, 'button_o', self.button_callback, 10)
        self.stick_sub= self.create_subscription(Float32, 'left_stick_y', self.joy_callback, 10)
    
    def joy_callback(self, msg):

        # Map joystick to velocity
        if self._odrive_state == OdriveState.VELO_MODE:
            if self._odrv.axis0.current_state == AxisState.CLOSED_LOOP_CONTROL: # Confirm in control mode

                setpoint = 0.75*V_MAX* ( ((msg.data) - 127.0) / 127.0) # Map -1 to 1

                self._odrv.axis0.controller.input_vel = setpoint
    
    def shutdown_odrive(self):
        # Stop motor safely
        self.get_logger().info("Stopping ODrive motors...")

        self._odrv.axis0.controller.input_vel = 0.0

        request_state(self._odrv.axis0, AxisState.IDLE)

        time.sleep(1.0)

def main():
    rclpy.init()
    node = PS5OdriveNode()
    try:
        rclpy.spin(node)  # Keep node alive and handle callbacks
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down PS5 Odrive node...")
    finally:
        node.shutdown_odrive() 
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
