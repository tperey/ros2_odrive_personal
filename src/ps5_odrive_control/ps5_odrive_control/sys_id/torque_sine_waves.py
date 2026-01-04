import threading
from enum import Enum
import time

import odrive
from odrive.enums import * # Get them all
from odrive.utils import request_state

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


from time import perf_counter

from scipy.signal import savgol_filter


# Constants
REV_TO_RAD = 2*np.pi  # Converts rev to radians

TORQUE_CONSTANT = 0.083 # [N-m/A]

TORQUE_AMPLITUDE = 0.11 # [N-m]
V_MAX = 10.0

freq_list = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 2.5, 5.0, 7.5, 10.0, 25.0, 50.0, 75.0, 100.0] # [Hz]
count_list = [2.0, 2.0, 2.0, 2.0, 5.0, 5.0, 5.0, 5.0, 5.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 20.0, 20.0, 20.0, 20.0]
count_list = 2*np.array(count_list)

# freq_list = [0.1, 0.2]
# count_list = [1.0, 1.0]

TIME_PER_FREQ = 10.0 # [s]

# CYCLES_PER_FREQ = 10 # [n]

class SysIdState(Enum):
    # For preventing crazy motions when pendulum in weird states
    OFF = 0
    START = 1
    MAX = 2

class SysIdOdrive():
    def __init__(self):

        # TODO: params for calibration

        # Odrive
        print("!!!STARTING ODrive motors...")
        self._odrv = get_Odrive_init(Vmax = V_MAX, Kv = 0.333, ConType = ControlMode.TORQUE_CONTROL,
                                     shouldCalibrate = False, shouldErase = False, shouldReconfig = False)
        print("Odrive prepped :)")
        self._odrv.axis0.controller.config.control_mode = ControlMode.TORQUE_CONTROL
        self._odrv.axis0.controller.config.input_mode = InputMode.PASSTHROUGH
        self._odrv.axis0.config.motor.current_control_bandwidth = 2000
        self._odrv.clear_errors()  # Clear errors on start
        self._odrv.axis0.pos_estimate = 0.0  # 0 the position on start

        # Profile-specific
        self.freq_list = freq_list
        self.count_list = count_list
        self.freq_counter = 0
        

        self.t0 = 0.0
        self.phase_start_time = 0.0
        self.cycle_counter = 0

        # Logging
        self.tel_dict = {
            "time": [],
            "pos_actual": [],
            "vel_actual": [],
            "tau_actual": [],
            "tau_cmd": [],
        }

        self.by_freq_dict = {f: {"time": [], "pos_actual": [], "vel_actual": [], "tau_actual": [], "tau_cmd": []} 
                 for f in self.freq_list}

    def change_motor_state(self, state_to_request = AxisState.IDLE):
        # Put in closed loop controol
        if self._odrv.axis0.current_state != state_to_request:
            self._odrv.axis0.controller.input_torque = 0.0
            request_state(self._odrv.axis0, state_to_request)
    
    def cyclic_control(self):

        # Handle time
        if self.t0 == 0.0:
            self.t0 = perf_counter() # On first call, get initial times
            self.duration_f = perf_counter()
            self.phase_start_time = perf_counter()  # For phase calculation

        cur_t = perf_counter() - self.t0 # Log
        phase_t = perf_counter() - self.phase_start_time # Cmd

        # Handle profile 
        freq = self.freq_list[self.freq_counter] # Just try one to start
        tau_cmd = TORQUE_AMPLITUDE*np.sin( (2*np.pi*freq)*phase_t)

        # Update odrive
        #print(f"tau_cmd = {tau_cmd}")
        self._odrv.axis0.controller.input_torque = -tau_cmd

        # Log update
        (self.tel_dict["time"]).append(cur_t)
        (self.tel_dict["pos_actual"]).append(-1*((self._odrv.axis0.pos_estimate)*REV_TO_RAD))
        (self.tel_dict["vel_actual"]).append(-1*((self._odrv.axis0.vel_estimate)*REV_TO_RAD))
        (self.tel_dict["tau_actual"]).append(((self._odrv.axis0.motor.foc.Iq_measured)*TORQUE_CONSTANT))
        (self.tel_dict["tau_cmd"]).append(tau_cmd)

        (self.by_freq_dict[freq]["time"]).append(cur_t)
        (self.by_freq_dict[freq]["pos_actual"]).append(-1 * (self._odrv.axis0.pos_estimate*REV_TO_RAD))
        (self.by_freq_dict[freq]["vel_actual"]).append(-1 * (self._odrv.axis0.vel_estimate*REV_TO_RAD))
        (self.by_freq_dict[freq]["tau_actual"]).append(self._odrv.axis0.motor.foc.Iq_measured * TORQUE_CONSTANT)
        (self.by_freq_dict[freq]["tau_cmd"]).append(tau_cmd)

        # Handle freq switching
        cycle_state = phase_t * freq
        if (cycle_state - self.cycle_counter) > 1.0: # Cycle complete
            self.cycle_counter += 1
            print(f"Updating cycle counter to {self.cycle_counter}")

        freq_switch_check = perf_counter() - self.duration_f
        #print(f"freq_switch_check = {freq_switch_check}")

        #if freq_switch_check >= TIME_PER_FREQ:
        if self.cycle_counter >= self.count_list[self.freq_counter]:
            # Stop motor for switch
            self._odrv.axis0.controller.input_torque = 0.0
            time.sleep(1.5) # To let motor stop

            self.freq_counter = self.freq_counter + 1 # Update switcher
            self.duration_f = perf_counter()
            
            if self.freq_counter >= (len(self.freq_list)):
                raise RuntimeError("Done with freq_list")

            print(f"~~~SWITCHING_FREQ to {self.freq_list[self.freq_counter]}")

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
        tau_actual = np.array(self.tel_dict['tau_actual'])
        tau_cmd = np.array(self.tel_dict['tau_cmd'])

        # Create directory if it doesn't exist
        os.makedirs(save_dir, exist_ok=True)
        
        # Generate timestamp for filenames
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Save data as pickle
        smooth_tau_actual = savgol_filter(tau_actual, window_length=51, polyorder=3)
        self.tel_dict["smooth_tau_actual"] = smooth_tau_actual
        data_filename = os.path.join(save_dir, f'sys_id_data_{timestamp}.pkl')
        with open(data_filename, 'wb') as f:
            pickle.dump(self.tel_dict, f)
        print(f"Data saved to: {data_filename}")

        for freq in self.by_freq_dict.keys():
            cur_smooth = savgol_filter(self.by_freq_dict[freq]['tau_actual'], window_length=51, polyorder=3)
            self.by_freq_dict[freq]['smooth_tau_actual'] = cur_smooth
        data_filename = os.path.join(save_dir, f'sys_id_BYFREQ_{timestamp}.pkl')
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
        axes[2].plot(t, smooth_tau_actual, 'b-', linewidth=2.0, label='Filtered Actual')
        axes[2].set_ylabel('Torque (N·m)', fontsize=12)
        axes[2].set_xlabel('Time (s)', fontsize=12)
        axes[2].grid(True, alpha=0.3)
        axes[2].legend(loc='upper right')
        
        # Save figure
        plot_filename = os.path.join(save_dir, f'sys_id_plot_{timestamp}.png')
        plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
        print(f"Plot saved to: {plot_filename}")
        
        # Show plot
        plt.show()

        # ANOTHER PLOT FOR PHASE
        plt.figure()
        fig, axes = plt.subplots(1, 1, figsize=(12, 6), sharex = True)

        # Normalized, all, for phase comparison
        normal_tau_cmd = tau_cmd/np.max(tau_cmd)
        ns_tau_act = smooth_tau_actual/np.max(smooth_tau_actual)
        normal_pos = pos/np.max(pos)
        normal_vel = vel/np.max(vel)

        axes.plot(t, normal_pos, 'b-', linewidth=1.5, label='Position')
        axes.plot(t, normal_vel, 'g-', linewidth=1.5, label='Velocity')
        axes.plot(t, ns_tau_act, 'r--', linewidth=1.5, label='Smooth Real Torque')
        axes.plot(t, normal_tau_cmd, 'k-', linewidth=1.5, label='Commanded Torque')
        axes.grid(True, alpha=0.3)
        axes.legend(loc='upper right')
        
        plt.tight_layout()
        
        # Save figure
        plot_filename = os.path.join(save_dir, f'sys_PHASE_plot_{timestamp}.png')
        plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
        print(f"Plot saved to: {plot_filename}")
        
        # Show plot
        plt.show()

# ---- MAIN -----
def main():
    odrive_sys_id = SysIdOdrive()
    odrive_sys_id.change_motor_state(AxisState.CLOSED_LOOP_CONTROL)
    try:
        while True:
            odrive_sys_id.cyclic_control()
    except KeyboardInterrupt:
        pass
    finally:
        print("Stopping motor...")
        odrive_sys_id.change_motor_state(AxisState.IDLE)

        print("Saving and plotting log...")
        odrive_sys_id.save_plot_log()

if __name__ == "__main__":
    main()
