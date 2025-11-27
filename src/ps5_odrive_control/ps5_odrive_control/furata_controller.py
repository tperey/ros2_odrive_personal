import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float32MultiArray
from inputs import get_gamepad
from enum import Enum
import time

import odrive
from odrive.enums import * # Get them all
from ps5_odrive_control.common.playing_with_odrive import get_Odrive_init
from odrive.utils import request_state

import numpy as np

REV_TO_RAD = 2*np.pi  # Converts rev to radians
SMALL_ANGLE = 0.6  # Radians about t2 = pi where controller actually kicks in

class OControlState(Enum):
    # For preventing crazy motions when pendulum in weird states
    OFF = 0
    ON = 1

V_MAX = 2.0

class FurataController(Node):
    def __init__(self):

        # General init
        super().__init__('furata_controller')

        # Params
        self.declare_parameter('doCal', False)
        self.declare_parameter('doErase', False)
        self.declare_parameter('doRecon', False)
        self._doCal = self.get_parameter('doCal').get_parameter_value().bool_value
        self._doErase = self.get_parameter('doErase').get_parameter_value().bool_value
        self._doRecon = self.get_parameter('doRecon').get_parameter_value().bool_value

        # Odrive
        self.get_logger().info("!!!STARTING ODrive motors...")
        self._odrv = get_Odrive_init(Vmax = V_MAX, ConType = ControlMode.TORQUE_CONTROL,
                                     shouldCalibrate = self._doCal, shouldErase = self._doErase, shouldReconfig = self._doRecon)
        self.get_logger().info("Odrive prepped :)")
        self._odrv.clear_errors()  # Clear errors on start
        self._odrv.axis0.pos_estimate = 0.0  # 0 the position on start
        self._odrv.axis0.controller.config.input_mode = InputMode.PASSTHROUGH #InputMode.VEL_RAMP
        
        self._control_state = OControlState.OFF  # Start out off

        # Subscribers
        self.encoder_sub = self.create_subscription(Float32MultiArray, 'encoder_furata', self.encoder_callback, 10)
    
        # Controller
        #self._lqr_K = np.array([-0.31622777, 9.16100057, -0.8506804, 0.84424739])  # Copied from simulator, with mega Link 1 damping
        self._lqr_K = np.array([-0.31622777, 7.21298925, -0.40204377, 0.69358956])
        self._q_equ = np.array([0.0, np.pi, 0.0, 0.0])

    def encoder_callback(self, msg):

        """ State machine for LQR encoder """
        t1 = (self._odrv.axis0.pos_estimate)*REV_TO_RAD
        t1d = (self._odrv.axis0.vel_estimate)*REV_TO_RAD
        t2 = msg.data[0]
        t2d = msg.data[1]  # Parse from encoder
        
        q = np.array([t1, t2, t1d, t2d])

        if self._control_state == OControlState.OFF:

            # Hit linear region, so re-engage
            if (t2 < (np.pi + SMALL_ANGLE)) and (t2 > (np.pi - SMALL_ANGLE)):
                request_state(self._odrv.axis0, AxisState.CLOSED_LOOP_CONTROL)  # Enable O-drive
                self._control_state = OControlState.ON
                self.get_logger().info("---Enabling motor")
            else:
                # Stay in IDLE
                pass 
            
            
        elif self._control_state == OControlState.ON:

            # Still in linear region, so run LQR controller
            if (t2 < (np.pi + SMALL_ANGLE)) and (t2 > (np.pi - SMALL_ANGLE)):
                tau = ((-self._lqr_K) @ (q - self._q_equ))
                tau = -tau*0.5 # For some reason, WAYYYY too large
                # Also, I think sign at motor level is opposite
                self.get_logger().info(f"Commanding torque = {tau}")
                self._odrv.axis0.controller.input_torque = tau
            else:
                # Stay in IDLE
                self._odrv.axis0.controller.input_vel = 0.0
                request_state(self._odrv.axis0, AxisState.IDLE)
                self._control_state = OControlState.OFF
                self.get_logger().info("~~~DISabling motor")

        else:
            self.get_logger().error("Bad control state")
    
    def shutdown_odrive(self):
        # Stop motor safely
        self.get_logger().info("Stopping ODrive motors...")

        self._odrv.axis0.controller.input_vel = 0.0

        request_state(self._odrv.axis0, AxisState.IDLE)

        time.sleep(1.0)

# ---- MAIN -----
def main():
    rclpy.init()
    node = FurataController()
    try:
        rclpy.spin(node)  # Keep node alive and handle callbacks
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Odrive in FurataController node...")
    finally:
        node.shutdown_odrive() 
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
