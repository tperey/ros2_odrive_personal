import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float32MultiArray
from inputs import get_gamepad
from enum import Enum
import time
from time import perf_counter
import matplotlib.pyplot as plt

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

SCALE_TORQUE = 0.75 #1.0 #0.25 #0.22 #0.75 #0.4
MAX_ALLOWABLE_TORQUE = 10.0 # [N/m] for clipping

DEADBAND_POS_INTEGRAL = 0.02 # [rad]
DEADBAND_VEL_INTEGRAL = 0.3 # [rad/s]

# *** FRICTION STUFF ***
# Fitted parameters:
#   Inertia J: 0.0024 kg·m²
#   Viscous damping b: 0.0005 N·m·s/rad
#   Static friction tau_s PEAK: 0.1195 N·m
#   Static friction tau_s mean: 0.0667 N·m
#   Dynamic friction tau_d: 0.0504 N·m
VEL_KICKSTARTER = 0.3 # [rad/s]
TAU_STARTER = 0.05     # [N/m]
TORQUE_KICKSTART = 0.07 # [N/M]. Seems like 0.1 to move in negative direction
TORQUE_SUSTAINER = 0.0 #0.05

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

V_MAX = 10.0

""" FurataIntegrated: single node which (1) gets pendulum/encoder info
    AND (2) controls Odrive motor """
class FurataIntegrated(Node):
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
        self._odrv = get_Odrive_init(Vmax = V_MAX, Kv = 1.0, KvI = 3.0, ConType = ControlMode.TORQUE_CONTROL,
                                     shouldCalibrate = self._doCal, shouldErase = self._doErase, shouldReconfig = self._doRecon)
        self.get_logger().info("Odrive prepped :)")
        self._odrv.clear_errors()  # Clear errors on start
        self._odrv.axis0.pos_estimate = 0.0  # 0 the position on start
        self._odrv.axis0.controller.config.input_mode = InputMode.PASSTHROUGH #InputMode.VEL_RAMP
        self._odrv.axis0.controller.config.control_mode = ControlMode.TORQUE_CONTROL
        self._control_state = OControlState.OFF  # Start out off

        # MCU Reader
        self.mcu_reader = ReaderMCU(
            port='/dev/ttyACM3',   # change to your MCU port
            baud=115200,
            logTime = False
        )

        # Read initial pendulum pos to determine what balance point is
        pend_state = self.mcu_reader.get_pend()
        t2i = pend_state[2] % (2*np.pi)
        if (t2i < (np.pi + SMALL_ANGLE)) and (t2i > (np.pi - SMALL_ANGLE)):
            self._q_equ = np.array([0.0, t2i, 0.0, 0.0])
            self.get_logger().info(f"```Set initial state to {t2i}")
        else:
            self._q_equ = np.array([0.0, np.pi, 0.0, 0.0])
    
        # Controller
        #self._lqr_K = np.array([-0.31622777,  7.47867553, -0.40398849,  0.73223273])  # R = 10
        #self._lqr_K = np.array([0.31622777,  7.47867553, 0.40398849,  0.73223273])  # R = 10, manual sign correction
        #self._lqr_K = np.array([0.31622777,  7.47867553, 0.0,  0.73223273])  # R = 10, manual sign correction
        #self._lqr_K = np.array([-1.0, 21.93765365, -1.26986995,  2.20989628])  # R = 1
        #self._lqr_K = np.array([-0.07160001, 4.2962601, -0.11372813, 0.80764367])  # Disc, Q with pend punish
        #self._lqr_K = np.array([-0.07160001, 2*4.2962601, -0.11372813, 0.80764367])  # Disc, Q with pend punish
        #self._lqr_K = np.array([-0.21056844, 11.62858984, -0.33371892, 0.3*2.36114121]) # Disc, Q pend pun, dt = 0.001
        
        # BEST SO FAR(with scaling of 0.75)
        #self._lqr_K = np.array([0.21056844, 11.62858984, 0.1*0.33371892, 0.3*2.36114121]) # Disc, Q pend pun, dt = 0.001
        # BEST SO FAR
        # self._lqr_K = np.array([0.21056844, 11.62858984, 0.001*0.33371892, 0.003*2.36114121]) # Disc, Q pend pun, dt = 0.001
        #self._lqr_K = np.array([0.0, 11.62858984, 0.0, 0.0]) # Disc, Q pend pun, dt = 0.001


        # MANUAL K - requies scale torque of 1.0
        #self._lqr_K = np.array([0.0, 5.0, 0.2, 0.2]) # [Kp_motor, Kp_pend, Kd_motor, Kd_pend]
        #self._lqr_K = np.array([0.0, 3.5, 0.0, 0.3])
        #self._lqr_K = np.array([0.0, 4.5, 0.0, 0.0])
        #self._lqr_K = np.array([0.0, 5.0, 0.0, 0.0])

        # self._lqr_K = np.array([0.0, 10.0, 0.0, 0.02])
        # self._ki = 170.0

        self._lqr_K = np.array([0.0, 2.0, 0.0, 0.05])
        self._ki = 2.0

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

        # Error
        self._sum_e = 0.0
        self._integral_decay = 1.0 #0.995

        # Odrive filtering, and telemetry
        self._fric_flag = 0.0
        self._t1 = None
        self._filt_t1d = 0.0
        self._filtdt = 0.01
        self._vel_filter = SimpleSpeedFilter(self._filtdt, cutoff = 4.0)  # Cutoff in [Hz]
        #self.filt_timer = self.create_timer(self._filtdt, self._filter_callback)

        self.tel_pub = self.create_publisher(TelemetryFurata, 'telemetry_furata', 10)
        #self.ctl_pub = self.create_publisher(Float32MultiArray, 'control_terms', 10)

        # Logging
        self.q_log = []
        self.tau_log = []
        self.dt_log = []
        self.integrating = []
        self.sum_e_log = []
        self.motor_state_log = []

    def _filter_callback(self):
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
            outgoing.name = ["enabled", "motor_pos", "pend_pos", "motor_vel", "pend_vel", "I_cmd", "I_actual", "torq_cmd", "torq_actual", "fric_flag"]
            outgoing.data = [enabled, t1, t2, t1d, t2d, current_cmd, current_act, tau_cmd, tau_act, self._fric_flag]
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
    
    def _friction_compensation(self, tau_cmd = 0.0, vel = 0.0, vel_thresh = 0.0,
                                tau_thresh = np.inf, tau_kickstart = 0.0, tau_sustain = 0.0):

        tau_to_send = tau_cmd

        # Get telemetry
        direction_cmd = np.sign(tau_cmd)

        # Apply 
        if np.abs(tau_cmd) > np.abs(tau_thresh):  # Only apply if enough error to move (cmd above some thresh)
            if abs(vel) < vel_thresh:
                tau_to_send += direction_cmd*tau_kickstart
                self._fric_flag = 1.0
            else:
                tau_to_send += np.sign(vel)*tau_sustain
                self._fric_flag = 0.0
        else:
            self._fric_flag = 0.0

        #print(f"tau_to_send = {tau_to_send}")

        return tau_to_send

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
                tau = ((-self._lqr_K) @ (q - self._q_equ))
                tau = tau*SCALE_TORQUE # For some reason, WAYYYY too large
                self._tau_des = tau

                # if self._alpha > 0.0:
                #     self._tau_des = self._alpha*self._tau_des + (1.0 - self._alpha)*tau
                # else:
                #     self._tau_des = tau  # For telemetry

                # # Friction (stick-slip) comp
                # self._tau_des = self._friction_compensation(tau_cmd = self._tau_des,
                #                                             vel = t1d,
                #                                             vel_thresh = VEL_KICKSTARTER,
                #                                             tau_thresh = TAU_STARTER,
                #                                             tau_kickstart = TORQUE_KICKSTART,
                #                                             tau_sustain = TORQUE_SUSTAINER)
                # Filter noise out of command signal
                # self._tau_des = self._ctrl_filt_tau.update(self._tau_des, new_dt = self._cur_dt) 

                # Integral action
                self._sum_e += (t2 - self._q_equ[1])*(self._cur_dt)
                self._tau_des -= (self._ki)*(self._sum_e)

                # Safe clipping
                self._tau_des = np.clip(self._tau_des, -MAX_ALLOWABLE_TORQUE, MAX_ALLOWABLE_TORQUE)
                self._odrv.axis0.controller.input_torque = -self._tau_des  # Sign at motor level is opposite
            else:
                # Stay in IDLE
                self._odrv.axis0.controller.input_torque = 0.0
                self._tau_des = 0.0
                request_state(self._odrv.axis0, AxisState.IDLE)
                self._control_state = OControlState.OFF
                self.get_logger().info("~~~DISabling motor")

        else:
            self.get_logger().error("Bad control state")
        
        # Logging
        self.q_log.append((q - self._q_equ))
        self.tau_log.append(self._tau_des)
        self.dt_log.append(self._cur_dt)
        self.sum_e_log.append(-(self._ki)*(self._sum_e))
        self.motor_state_log.append(self._odrv.axis0.motor.foc.Iq_measured*TORQUE_CONSTANT)
        if ((np.abs(t2d - self._q_equ[3]) > DEADBAND_VEL_INTEGRAL)):
            self.integrating.append(1)
        else:
            self.integrating.append(0)
    
    def shutdown_odrive(self):
        # Stop motor safely
        self.get_logger().info("Stopping ODrive motors...")

        self._odrv.axis0.controller.input_torque = 0.0

        request_state(self._odrv.axis0, AxisState.IDLE)

        time.sleep(1.0)

        # --- PLOT LOGGED DATA ---
        if len(self.q_log) > 0:
            q_arr = np.array(self.q_log)  # shape: [timesteps, 4]
            tau_arr = np.array(self.tau_log)  # shape: [timesteps]
            sum_e_arr = np.array(self.sum_e_log)
            dt_arr = np.array(self.dt_log)
            motor_state_arr = np.array(self.motor_state_log)
            integrating_arr = np.array(self.integrating)

            max_idx = np.max(np.array([len(q_arr[:,1]), len(tau_arr), len(sum_e_arr), len(dt_arr)]))

            time_vec = np.arange(q_arr.shape[0]) * self._dt  # approximate time

            fig, axs = plt.subplots(4, 1, figsize=(15, 9), sharex=True)

            # Plot pendulum and motor positions
            #axs[0].plot(time_vec, q_arr[:,0], label='Motor pos t1 [rad]')
            axs[0].plot(time_vec[:max_idx], q_arr[:max_idx,1], label='Pend pos t2 [rad]')
            axs[0].set_ylabel('Position [rad]')
            axs[0].legend()
            axs[0].grid(True)

            # Plot velocities
            #axs[1].plot(time_vec, q_arr[:,2], label='Motor vel t1d [rad/s]')
            axs[1].plot(time_vec[:max_idx], q_arr[:max_idx,3], label='Pend vel t2d [rad/s]')
            axs[1].set_ylabel('Velocity [rad/s]')
            axs[1].legend()
            axs[1].grid(True)

            # Plot torque
            axs[2].plot(time_vec[:max_idx], tau_arr[:max_idx], label='Tau_des / command vel')
            axs[2].plot(time_vec[:max_idx], sum_e_arr[:max_idx], label='Ki term')
            axs[2].plot(time_vec[:len(motor_state_arr)], motor_state_arr[:len(motor_state_arr)], label = 'Actual Torque')
            axs[2].set_xlabel('Time [s]')
            axs[2].set_ylabel('Command')
            axs[2].legend()
            axs[2].grid(True)

            # Plot dt
            axs[3].plot(time_vec[:max_idx], dt_arr[:max_idx], label='dt [s]')
            axs[3].set_xlabel('Time [s]')
            axs[3].set_ylabel('dt [s]')
            axs[3].legend()
            axs[3].grid(True)

            plt.tight_layout()
            plt.show()

# ---- MAIN -----
def main():
    rclpy.init()
    node = FurataIntegrated(doLog = True, alpha = 0.0)
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
