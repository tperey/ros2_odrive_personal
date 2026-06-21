import threading
from enum import Enum
import time

import odrive
from odrive.enums import * # Get them all
from odrive.utils import request_state

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# Add common directory to path
import os
import sys
common_path = os.path.join(os.path.dirname(__file__), '..', 'common')
sys.path.insert(0, os.path.abspath(common_path))
from playing_with_odrive import get_Odrive_init

import numpy as np
import matplotlib.pyplot as plt
import pickle
from datetime import datetime
from pathlib import Path

from time import perf_counter

from scipy.signal import savgol_filter

# Constants
REV_TO_RAD = 2*np.pi  # Converts rev to radians

TORQUE_CONSTANT = 0.083 # [N-m/A]

TORQUE_AMPLITUDE = 0.13 # [N-m]
V_MAX = 40.0

class LinearSIDRun:
    def __init__(self, amp = TORQUE_AMPLITUDE, ramp_t = 5.0, fall_t = 2.0, cycles = 1):
        self.amp = amp
        self.ramp_t = ramp_t
        self.fall_t = fall_t
        self.cycles = cycles

# Input object list
sysid_inputs = [
    LinearSIDRun(TORQUE_AMPLITUDE, 2.0, 2.0, 100.0)
]

class SysIdState(Enum):
    # For preventing crazy motions when pendulum in weird states
    OFF = 0
    START = 1
    MAX = 2

class SIFurataLinear(Node):
    def __init__(self):

        # TODO: params for calibration
        super().__init__('linear_furata_SI')

        # Odrive
        print("!!!STARTING ODrive motors...")
        self._odrv = get_Odrive_init(Vmax = V_MAX, Kv = 0.333, ConType = ControlMode.TORQUE_CONTROL,
                                     shouldCalibrate = False, shouldErase = False, shouldReconfig = False)
        print("Odrive prepped :)")
        self._odrv.axis0.controller.config.control_mode = ControlMode.TORQUE_CONTROL
        self._odrv.axis0.controller.config.input_mode = InputMode.PASSTHROUGH
        self._odrv.axis0.controller.config.vel_limit = V_MAX
        self._odrv.axis0.config.motor.current_control_bandwidth = 2000
        self._odrv.clear_errors()  # Clear errors on start
        self._odrv.axis0.pos_estimate = 0.0  # 0 the position on start

        # Profile-specific
        self.input_list = sysid_inputs
        self.freq_counter = 0

        self.t0 = 0.0
        self.phase_start_time = 0.0
        self.cycle_counter = 0

        self.is_rising = True
        self.toggle_cycle = False
        self.is_motor_on = True

        self._dt = 0.005
        self.ctrl_timer = self.create_timer(self._dt, self.cyclic_control)

        # Pendulum
        self.encoder_sub = self.create_subscription(Float32MultiArray, 'encoder_furata', self.encoder_callback, 10)
        self.cur_pend_state = [0.0, 0.0, 0.0]

        # Logging
        self.tel_dict = {
            "time": [],
            "pos_actual": [],
            "vel_actual": [],
            "pend_pos": [],
            "pend_vel": [],
            "pend_acc": [],
            "tau_actual": [],
            "tau_cmd": [],
            "is_motor_on": [],
        }

        # TODO: avoid overwriting by_freq entries when front amplitudes repeat
        self.by_freq_dict = {sysidrun.amp: {"time": [], "pos_actual": [], "vel_actual": [], "pend_pos": [], "pend_vel": [], "pend_acc": [], "tau_actual": [], "tau_cmd": []} 
                 for sysidrun in self.input_list}
    
    def encoder_callback(self, msg):
        # accelKf - 1, 2, 3 are p, v, a
        self.cur_pend_state[0] = msg.data[1] # Prob don't need, but could have % (2*np.pi)
        self.cur_pend_state[1] = msg.data[2]
        self.cur_pend_state[2] = msg.data[3]

    def change_motor_state(self, state_to_request = AxisState.IDLE):
        # Put in closed loop controol
        if self._odrv.axis0.current_state != state_to_request:
            self._odrv.axis0.controller.input_torque = 0.0
            request_state(self._odrv.axis0, state_to_request)
    
    def cyclic_control(self):

        # Handle time
        if self.t0 == 0.0:
            self.t0 = perf_counter() # On first call, get initial times
            self.phase_start_time = perf_counter()  # For phase calculation

        cur_t = perf_counter() - self.t0 # Log
        phase_t = perf_counter() - self.phase_start_time # Cmd

        # Handle profile 
        cur_sysidrun = self.input_list[self.freq_counter]
        tau_cmd = 0.0

        if self.is_rising:
            # Set torque
            tau_cmd =(cur_sysidrun.amp/cur_sysidrun.ramp_t)*phase_t
            self._odrv.axis0.controller.input_torque = -tau_cmd
            
            # Check for fall
            if phase_t >= cur_sysidrun.ramp_t:
                self.is_rising = False
                self.phase_start_time = perf_counter()
                self.get_logger().info("Switching to falling")
                self.change_motor_state()  # Just cut power, and wait for fall time
                self.is_motor_on = False
        else:
            # Check for cycle end
            if (phase_t >= cur_sysidrun.fall_t) or (cur_sysidrun.fall_t == 0.0):
                self.is_rising = True
                self.toggle_cycle = True
                self.phase_start_time = perf_counter()
            
            tau_cmd = 0.0

        # Log update
        freq = cur_sysidrun.amp
        (self.tel_dict["time"]).append(cur_t)
        (self.tel_dict["pos_actual"]).append(-1*((self._odrv.axis0.pos_estimate)*REV_TO_RAD))
        (self.tel_dict["vel_actual"]).append(-1*((self._odrv.axis0.vel_estimate)*REV_TO_RAD))
        (self.tel_dict["pend_pos"]).append(self.cur_pend_state[0])
        (self.tel_dict["pend_vel"]).append(self.cur_pend_state[1])
        (self.tel_dict["pend_acc"]).append(self.cur_pend_state[2])
        (self.tel_dict["tau_actual"]).append(((self._odrv.axis0.motor.foc.Iq_measured)*TORQUE_CONSTANT))
        (self.tel_dict["tau_cmd"]).append(tau_cmd)
        (self.tel_dict["is_motor_on"]).append(self.is_motor_on)

        (self.by_freq_dict[freq]["time"]).append(cur_t)
        (self.by_freq_dict[freq]["pos_actual"]).append(-1 * (self._odrv.axis0.pos_estimate*REV_TO_RAD))
        (self.by_freq_dict[freq]["vel_actual"]).append(-1 * (self._odrv.axis0.vel_estimate*REV_TO_RAD))
        (self.by_freq_dict[freq]["pend_pos"]).append(self.cur_pend_state[0])
        (self.by_freq_dict[freq]["pend_vel"]).append(self.cur_pend_state[1])
        (self.by_freq_dict[freq]["pend_acc"]).append(self.cur_pend_state[2])
        (self.by_freq_dict[freq]["tau_actual"]).append(self._odrv.axis0.motor.foc.Iq_measured * TORQUE_CONSTANT)
        (self.by_freq_dict[freq]["tau_cmd"]).append(tau_cmd)

        # Handle cycle switching
        if self.toggle_cycle: # Cycle complete
            self.cycle_counter += 1
            self.toggle_cycle = False
            self.phase_start_time = perf_counter()
            self.change_motor_state(AxisState.CLOSED_LOOP_CONTROL)
            self.is_motor_on = True
            print(f"Updating cycle counter to {self.cycle_counter}")

        # Handle run switching
        if self.cycle_counter >= (cur_sysidrun.cycles):
            # Stop motor for switch
            self._odrv.axis0.controller.input_torque = 0.0
            time.sleep(2.5) # To let motor stop

            self.freq_counter = self.freq_counter + 1 # Update switcher
            
            if self.freq_counter >= (len(self.input_list)):
                raise RuntimeError("Done with freq_list")
            
            print(f"~~~SWITCHING to {self.input_list[self.freq_counter]}")

            # 0 everyting
            self._odrv.axis0.pos_estimate = 0.0
            self.phase_start_time = perf_counter()  # For phase calculation
            self.cycle_counter = 0

    def save_plot_log(self, save_dir='sys_id_data'):
        """
        Save telemetry data and generate plots
        
        Args:
            save_dir: Directory to save data and plots
        """
        # Convert to numpy arrays for plotting
        t = np.array(self.tel_dict['time'])
        pos = np.array(self.tel_dict['pos_actual'])
        vel = np.array(self.tel_dict['vel_actual'])
        p_pos = np.array(self.tel_dict['pend_pos'])
        p_vel = np.array(self.tel_dict['pend_vel'])
        p_acc = np.array(self.tel_dict["pend_acc"])
        tau_actual = np.array(self.tel_dict['tau_actual'])
        tau_cmd = np.array(self.tel_dict['tau_cmd'])

        # Create directory if it doesn't exist
        os.makedirs(save_dir, exist_ok=True)
        
        # Generate timestamp for filenames
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Save data as pickle
        # smooth_tau_actual = savgol_filter(tau_actual, window_length=51, polyorder=3)
        # self.tel_dict["smooth_tau_actual"] = smooth_tau_actual
        data_filename = os.path.join(save_dir, f'hr_linear_sys_id_data_{timestamp}.pkl')
        with open(data_filename, 'wb') as f:
            pickle.dump(self.tel_dict, f)
        print(f"Data saved to: {data_filename}")

        # for freq in self.by_freq_dict.keys():
        #     cur_smooth = savgol_filter(self.by_freq_dict[freq]['tau_actual'], window_length=51, polyorder=3)
        #     self.by_freq_dict[freq]['smooth_tau_actual'] = cur_smooth
        data_filename = os.path.join(save_dir, f'hr_linear_sys_id_BYFREQ_{timestamp}.pkl')
        with open(data_filename, 'wb') as f:
            pickle.dump(self.by_freq_dict, f)
        print(f"By Freq Data saved to: {data_filename}")
        
        # # Save data as numpy (optional, easier to load later)
        # npz_filename = os.path.join(save_dir, f'sys_id_data_{timestamp}.npz')
        # np.savez(npz_filename, **{key: np.array(val) for key, val in self.tel_dict.items()})
        # print(f"Data saved to: {npz_filename}")
        
        # Create figure with 3 subplots
        fig, axes = plt.subplots(3, 1, figsize=(12, 6), sharex = True)
        
        # Plot 1: Position
        axes[0].plot(t, pos, 'b-', linewidth=1.5, label='Position')
        axes[0].set_ylabel('Position (rad)', fontsize=12)
        axes[0].set_xlabel('Time (s)', fontsize=12)
        axes[0].grid(True, alpha=0.3)
        axes[0].legend(loc='upper right')
        axes[0].set_title('System Identification Results', fontsize=14, fontweight='bold')
        
        # Plot 2: Velocity
        axes[1].plot(t, vel, 'g-', linewidth=1.5, label='Velocity')
        axes[1].set_ylabel('Velocity (rad/s)', fontsize=12)
        axes[1].set_xlabel('Time (s)', fontsize=12)
        axes[1].grid(True, alpha=0.3)
        axes[1].legend(loc='upper right')
        
        # Plot 3: Torque (commanded and actual)
        axes[2].plot(t, tau_cmd, 'k-', linewidth=3.0, label='Commanded', alpha=0.7)
        axes[2].plot(t, tau_actual, 'r--', linewidth=1.0, label='Actual')
        #axes[2].plot(t, smooth_tau_actual, 'b-', linewidth=2.0, label='Filtered Actual')
        axes[2].set_ylabel('Torque (N·m)', fontsize=12)
        axes[2].set_xlabel('Time (s)', fontsize=12)
        axes[2].grid(True, alpha=0.3)
        axes[2].legend(loc='upper right')
        
        # Save figure
        plot_filename = os.path.join(save_dir, f'hr_linearsi_motor_plot_{timestamp}.png')
        plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
        print(f"Plot saved to: {plot_filename}")
        
        # Show plot
        plt.show()

        # ANOTHER PLOT FOR PHASE
        plt.figure()
        fig, axes = plt.subplots(1, 1, figsize=(12, 6), sharex = True)

        # Normalized, all, for phase comparison
        normal_tau_cmd = tau_cmd/np.max(tau_cmd)
        #ns_tau_act = smooth_tau_actual/np.max(smooth_tau_actual)
        normal_pos = pos/np.max(pos)
        normal_vel = vel/np.max(vel)

        axes.plot(t, normal_pos, 'b-', linewidth=1.5, label='Position')
        axes.plot(t, normal_vel, 'g-', linewidth=1.5, label='Velocity')
        #axes.plot(t, ns_tau_act, 'r--', linewidth=1.5, label='Smooth Real Torque')
        axes.plot(t, normal_tau_cmd, 'k-', linewidth=1.5, label='Commanded Torque')
        axes.grid(True, alpha=0.3)
        axes.legend(loc='upper right')
        
        plt.tight_layout()
        
        # Save figure
        plot_filename = os.path.join(save_dir, f'hr_linearsi_motorPhase_plot_{timestamp}.png')
        plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
        print(f"Plot saved to: {plot_filename}")
        
        # Show plot
        plt.show()

        # ANOTHER PLOT FOR PENDULUM
        plt.figure()
        fig, axes = plt.subplots(3,1, figsize=(9,3), sharex = True)

        # Plot 1: Position
        axes[0].plot(t, p_pos, 'b-', linewidth=1.5, label='Position')
        axes[0].set_ylabel('Position (rad)', fontsize=12)
        axes[0].set_xlabel('Time (s)', fontsize=12)
        axes[0].grid(True, alpha=0.3)
        axes[0].legend(loc='upper right')
        axes[0].set_title('System Identification Results', fontsize=14, fontweight='bold')
        
        # Plot 2: Velocity
        axes[1].plot(t, p_vel, 'g-', linewidth=1.5, label='Velocity')
        axes[1].set_ylabel('Velocity (rad/s)', fontsize=12)
        axes[1].set_xlabel('Time (s)', fontsize=12)
        axes[1].grid(True, alpha=0.3)
        axes[1].legend(loc='upper right')

        # Plot 3: Acceleration
        axes[2].plot(t, p_acc, 'g-', linewidth=1.5, label='Acceleration')
        axes[2].set_ylabel('Acceleration (rad/s^2)', fontsize=12)
        axes[2].set_xlabel('Time (s)', fontsize=12)
        axes[2].grid(True, alpha=0.3)
        axes[2].legend(loc='upper right')

        # Save figure
        plot_filename = os.path.join(save_dir, f'hr_linearsi_Pend_plot_{timestamp}.png')
        plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
        print(f"Plot saved to: {plot_filename}")
        
        # Show plot
        plt.show()
        
    
    def shutdown_odrive(self):
        # Stop motor safely
        self.get_logger().info("Stopping ODrive motors...")

        self._odrv.axis0.controller.input_torque = 0.0

        request_state(self._odrv.axis0, AxisState.IDLE)

        time.sleep(1.0)

# ---- MAIN -----
def main():
    rclpy.init()
    node = SIFurataLinear()
    node.change_motor_state(AxisState.CLOSED_LOOP_CONTROL)
    try:
        rclpy.spin(node)  # Keep node alive and handle callbacks
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Odrive in FurataController node...")
    finally:
        print("Stopping motor...")
        node.shutdown_odrive() 

        print("Saving and plotting log...")
        DATA_DIR = Path.home() / "ws_ros2_odrive" / "src" / "ps5_odrive_control" / "ps5_odrive_control" / "sys_id" / "linear_si_data"
        DATA_DIR.mkdir(parents=True, exist_ok=True)
        node.save_plot_log(save_dir=DATA_DIR)
        

        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
