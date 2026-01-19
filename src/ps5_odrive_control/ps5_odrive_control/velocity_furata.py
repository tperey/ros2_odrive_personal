import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float32MultiArray
from inputs import get_gamepad
from enum import Enum
import time
from time import perf_counter

import odrive
from odrive.enums import * # Get them all
from ps5_odrive_control.common.playing_with_odrive import get_Odrive_init
from ps5_odrive_control.common.ReaderMCU import ReaderMCU
from odrive.utils import request_state

import numpy as np

from msgs_furata.msg import TelemetryFurata

REV_TO_RAD = 2*np.pi  # Converts rev to radians
SMALL_ANGLE = 0.5  # Radians about t2 = pi where controller actually kicks in
TORQUE_CONSTANT = 0.083 # [N-m/A]

MAX_ALLOWABLE_VELOCITY = 4.0 # [rev/s]

class SimpleSpeedFilter():
    def __init__(self, dt, cutoff):
        self._dt = dt
        self._alpha = np.exp(-2*np.pi*dt*cutoff)
        self._tau = 1.0 / (2.0*np.pi*cutoff)

        self._prev_velo = 0.0

        self._cutoff = cutoff
    
    def update(self, cur_velo, new_dt = 0.0):

        # Allow dynamic dt
        if new_dt > 0.0:
            self._dt = new_dt
            self._alpha = np.exp(-2*np.pi*self._dt*self._cutoff)

        # Passed in a pos, compute velo
        new_velo = self._alpha*self._prev_velo + (1.0 - self._alpha)*cur_velo
        self._prev_velo = new_velo  # Update previous
        return new_velo
    
    def var_update(self, cur_velo, dt):
        # If dt varies
        dt = max(dt, 1e-6)  # avoid division by zero

        # Compute alpha dynamically
        self._alpha = dt / (self._tau + dt)

        # Filter velocity
        new_velo = self._prev_velo + self._alpha * (cur_velo - self._prev_velo)

        # Save previous values
        self._prev_velo = new_velo

        return new_velo


class OControlState(Enum):
    # For preventing crazy motions when pendulum in weird states
    OFF = 0
    ON = 1

V_MAX = 6.0

""" FurataIntegrated: single node which (1) gets pendulum/encoder info
    AND (2) controls Odrive motor """
class VelocityFurata(Node):
    def __init__(self,
                 port='/dev/ttyACM0',
                 baud=115200,
                 topic='encoder_furata',
                 alpha = 0.0,
                 doLog = False):

        # General init
        super().__init__('furata_integrated')
        self._logTime = doLog
        self._last_t = None
        self._printer = 0

        # Params
        self.declare_parameter('doCal', False)
        self.declare_parameter('doErase', False)
        self.declare_parameter('doRecon', False)
        self._doCal = self.get_parameter('doCal').get_parameter_value().bool_value
        self._doErase = self.get_parameter('doErase').get_parameter_value().bool_value
        self._doRecon = self.get_parameter('doRecon').get_parameter_value().bool_value

        # Odrive
        self.get_logger().info("!!!STARTING ODrive motors...")
        self._odrv = get_Odrive_init(Vmax = V_MAX, Kv = 1.0, KvI = 3.0, ConType = ControlMode.VELOCITY_CONTROL,
                                     shouldCalibrate = self._doCal, shouldErase = self._doErase, shouldReconfig = self._doRecon)
        self.get_logger().info("Odrive prepped :)")
        self._odrv.clear_errors()  # Clear errors on start
        self._odrv.axis0.pos_estimate = 0.0  # 0 the position on start
        self._odrv.axis0.controller.config.input_mode = InputMode.PASSTHROUGH
        self._odrv.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
        self._odrv.axis0.controller.config.vel_gain = 1.0
        self._odrv.axis0.controller.config.vel_integrator_gain = 3.0
        self._control_state = OControlState.OFF  # Start out off

        # MCU Reader
        self.mcu_reader = ReaderMCU(
            port='/dev/ttyACM1',   # change to your MCU port
            baud=115200,
            logTime = False
        )
    
        # Controller
        # MANUAL K -  scale torque of 1.0
        # [ (rev/s)/rad,  (rev/s)/rad, (rev/s)/(rev/s), (rev/s)/(rad/s)]
        #self._lqr_K = np.array([0.0, 5.0, 0.0, 0.0])
        self._lqr_K = np.array([0.0, 10.0, 0.0, 1.0])

        self.get_logger().info(f"K = {self._lqr_K}")
        self._q_equ = np.array([0.0, np.pi, 0.0, 0.0])
        self.q = np.array([0.0, np.pi, 0.0, 0.0])

        self._dt = 0.001 # [ms]. So 500 Hz
        self.ctrl_timer = self.create_timer(self._dt, self.control_callback)
        self._ctrl_filt_t1d = SimpleSpeedFilter(self._dt, cutoff = 4.0)  # Cutoff in [Hz]
        self._ctrl_filt_tau = SimpleSpeedFilter(self._dt, cutoff = 4.0)
        self._cur_dt = self._dt

        self._alpha = alpha

        self._tau_des = 0.0  # [N-m]

        # Odrive filtering, and telemetry
        self._t1 = None
        self._filt_t1d = 0.0
        self._filtdt = 0.01
        self._vel_filter = SimpleSpeedFilter(self._filtdt, cutoff = 4.0)  # Cutoff in [Hz]
        #self.tel_timer = self.create_timer(self._filtdt, self._telemetry_callback)

        self.tel_pub = self.create_publisher(TelemetryFurata, 'telemetry_furata', 10)
        #self.ctl_pub = self.create_publisher(Float32MultiArray, 'control_terms', 10)

    def _telemetry_callback(self):
        # Publish servo telemetry
        t1 = self.q[0]
        t1d = self.q[2]
        t2 = self.q[1]
        t2d = self.q[3]

        current_cmd = self._odrv.axis0.motor.foc.Iq_setpoint
        current_act = self._odrv.axis0.motor.foc.Iq_measured

        tau_cmd = self._tau_des
        tau_act = current_act*TORQUE_CONSTANT

        if self._odrv.axis0.current_state == AxisState.CLOSED_LOOP_CONTROL:
            enabled = 1.0
        else:
            enabled = 0.0
    
        # Msg
        if self._logTime:
            outgoing = TelemetryFurata()
            outgoing.name = ["enabled", "motor_pos", "pend_pos", "motor_vel", "pend_vel", "I_cmd", "I_actual", "torque_actual", "command (rev/s)"]
            outgoing.data = [enabled, t1, t2, t1d, t2d, current_cmd, current_act, tau_act, tau_cmd]
            self.tel_pub.publish(outgoing)

        # # Check control terms
        # error = self.q - self._q_equ

        # gains = -self._lqr_K
        # if enabled:
        #     c0 = gains[0]*error[0]
        #     c1 = gains[1]*error[1]
        #     c2 = gains[2]*error[2]
        #     c3 = gains[3]*error[3]
        # else:
        #     c0 = 0.0
        #     c1 = 0.0
        #     c2 = 0.0
        #     c3 = 0.0

        # control_terms = Float32MultiArray()
        # control_terms.data = [c0, c1, c2, c3]
        # self.ctl_pub.publish(control_terms)

    """ CONTROL """

    def _update_state(self):
        # Simply update stored state

         # Time logging
        self._printer += 1
        now = perf_counter()
        if self._last_t is None:
            cur_dt = 0.001  # Default
        else:
            cur_dt = now - self._last_t

        if (self._printer % 1000 == 0) and (self._logTime):
            self.get_logger().info(f"cur_dt = {cur_dt}")
            self.get_logger().info(f"state = {self.q}")
        if (self._printer % 100 == 0) and (self._logTime):
            self.get_logger().info(f"Commanding torque = {-self._tau_des}")
        self._last_t = now
        self._cur_dt = cur_dt


        # Odrive
        t1 = -1*((self._odrv.axis0.pos_estimate)*REV_TO_RAD)  # Odrive space is NEGATIVE
        t1d_raw = -1*((self._odrv.axis0.vel_estimate)*REV_TO_RAD)
        t1d = self._ctrl_filt_t1d.update(t1d_raw, new_dt = cur_dt) #

        # Pendulum
        pend_state = self.mcu_reader.get_pend()
        t2 = pend_state[2] % (2*np.pi)
        t2d = pend_state[3]
        
        self.q = np.array([t1, t2, t1d, t2d])

    def control_callback(self):
        # Get state vars from member var
        self._update_state()  # Read MCU
        q = self.q
        t1 = self.q[0]
        t1d = self.q[2]
        t2 = self.q[1]
        t2d = self.q[3] # Parse from encoder

        """ State machine for LQR encoder """
        if self._control_state == OControlState.OFF:

            # Hit linear region, so re-engage
            if (t2 < (np.pi + SMALL_ANGLE)) and (t2 > (np.pi - SMALL_ANGLE)):
                request_state(self._odrv.axis0, AxisState.CLOSED_LOOP_CONTROL)  # Enable O-drive
                self._control_state = OControlState.ON
                self._odrv.axis0.pos_estimate = 0.0  # 0 the position on start
                self.get_logger().info("---Enabling motor")
            else:
                # Stay in IDLE
                pass 
            
            
        elif self._control_state == OControlState.ON:

            # Still in linear region, so run LQR controller
            if (t2 < (np.pi + SMALL_ANGLE)) and (t2 > (np.pi - SMALL_ANGLE)):
                # Generate command via controller
                tau = ((-self._lqr_K) @ (q - self._q_equ))
                self._tau_des = tau

                # Filter noise out of command signal
                # self._tau_des = self._ctrl_filt_tau.update(self._tau_des, new_dt = self._cur_dt) 

                # Safe clipping
                self._tau_des = np.clip(self._tau_des, -MAX_ALLOWABLE_VELOCITY, MAX_ALLOWABLE_VELOCITY)
                self._odrv.axis0.controller.input_vel = -self._tau_des  # Sign at motor level is opposite
            else:
                # Stay in IDLE
                self._odrv.axis0.controller.input_vel = 0.0
                self._tau_des = 0.0
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
    node = VelocityFurata(doLog = True, alpha = 0.0)
    try:
        rclpy.spin(node)  # Keep node alive and handle callbacks
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Odrive in IntegratedFurata node...")
    finally:
        node.shutdown_odrive() 
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
