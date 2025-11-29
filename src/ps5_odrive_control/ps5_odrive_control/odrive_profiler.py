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

from msgs_furata.msg import TelemetryFurata

REV_TO_RAD = 2*np.pi  # Converts rev to radians
SMALL_ANGLE = 0.5  # Radians about t2 = pi where controller actually kicks in
TORQUE_CONSTANT = 0.083 # [N-m/A]

SCALE_TORQUE = 0.15 
V_MAX = 1.0

class ProfileState(Enum):
    # For preventing crazy motions when pendulum in weird states
    OFF = 0
    START = 1
    MAX = 2

class OdriveProfiler(Node):
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
        self._odrv = get_Odrive_init(Vmax = V_MAX, Kv = 0.333, ConType = ControlMode.TORQUE_CONTROL,
                                     shouldCalibrate = self._doCal, shouldErase = self._doErase, shouldReconfig = self._doRecon)
        self.get_logger().info("Odrive prepped :)")
        self._odrv.clear_errors()  # Clear errors on start
        self._odrv.axis0.pos_estimate = 0.0  # 0 the position on start

        # Profile-specific
        self._odrv.axis0.controller.config.input_mode = InputMode.TRAP_TRAJ
        self._odrv.axis0.controller.config.trap_traj_vel_limit = V_MAX/4.0 # Max cruise speed for TRAP_TRAJ
        self._odrv.axis0.controller.config.trap_traj_accel_limit = 10.0*(V_MAX/4.0)
        self._odrv.axis0.controller.config.trap_traj_decel_limit = 10.0*(V_MAX/4.0)
        self._counter = 0
        self._control_state = ProfileState.OFF
    
        # Controller
        self._dt = 0.002 # [ms]. So 500 Hz
        self.ctrl_timer = self.create_timer(self._dt, self.control_callback)

        # Telemetry
        self._tel_t = 0.005 # [ms]. So 200 Hz
        self._tau_des = 0.0  # [N-m]
        self.tel_timer = self.create_timer(self._tel_t, self.telemetry_callback)
        self.tel_pub = self.create_publisher(TelemetryFurata, 'telemetry_furata', 10)
    
    def telemetry_callback(self):
        # Publish servo telemetry
        t1 = -1*((self._odrv.axis0.pos_estimate)*REV_TO_RAD)
        t1d = -1*((self._odrv.axis0.vel_estimate)*REV_TO_RAD)

        current_cmd = self._odrv.axis0.motor.foc.Iq_setpoint
        current_act = self._odrv.axis0.motor.foc.Iq_measured

        tau_cmd = self._tau_des
        tau_act = current_act*TORQUE_CONSTANT

        if self._odrv.axis0.current_state == AxisState.CLOSED_LOOP_CONTROL:
            enabled = 1.0
        else:
            enabled = 0.0

        # Msg
        outgoing = TelemetryFurata()
        outgoing.name = ["enabled", "motor_pos", "motor_vel", "I_cmd", "I_actual", "torq_cmd", "torq_actual"]
        outgoing.data = [enabled, t1, t1d, current_cmd, current_act, tau_cmd, tau_act]
        self.tel_pub.publish(outgoing)

    def control_callback(self):

        # State machine
        if self._control_state == ProfileState.OFF:
            # Put in control mode
            if self._odrv.axis0.current_state != AxisState.CLOSED_LOOP_CONTROL:
                request_state(self._odrv.axis0, AxisState.CLOSED_LOOP_CONTROL)
                self._odrv.axis0.pos_estimate = 0.0  # 0 the position on start
                self.get_logger().info("---Enabling motor")
            
            # TRANSITION
            self._control_state = ProfileState.ON
            self._odrv.axis0.controller.input_pos = 0.25  # [rev]
        
        elif self._control_state == ProfileState.START:

            # Wait 4 seconds, then move
            self._counter += 1

            if self._counter == 2000:  # 500 Hz, so 2000 counts for 4 s
                 # TRANSITION
                self._control_state = ProfileState.MAX
                self._odrv.axis0.controller.input_pos = 0.75  # [rev]
                self._counter = 0
        
        elif self._control_state == ProfileState.MAX:

            # Wait 4 seconds, then move
            self._counter += 1

            if self._counter == 2000:  # 500 Hz, so 2000 counts for 4 s
                 # TRANSITION
                self._control_state = ProfileState.start
                self._odrv.axis0.controller.input_pos = 0.25  # [rev]
                self._counter = 0
                
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
    node = OdriveProfiler()
    try:
        rclpy.spin(node)  # Keep node alive and handle callbacks
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Odrive in OdriveProfiler node...")
    finally:
        node.shutdown_odrive() 
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
