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

# Frictions

class FrictionMode(Enum):
    TORQUE = 0
    VELOCITY = 1

class FrictionOdrive():
    def __init__(self):

        # TODO: params for calibration

        # Odrive
        print("!!!STARTING ODrive motors...")
        self._odrv = get_Odrive_init(Vmax = V_MAX, Kv = 0.333, ConType = ControlMode.TORQUE_CONTROL,
                                     shouldCalibrate = False, shouldErase = False, shouldReconfig = False)
        print("Odrive prepped :)")
        
        self._odrv.axis0.config.motor.current_control_bandwidth = 2000
        self._odrv.clear_errors()  # Clear errors on start
        self._odrv.axis0.pos_estimate = 0.0  # 0 the position on start

        # STUFF
        self.torque_to_use = 0
        self.velocity_to_use = 0
        self.mode = FrictionMode.TORQUE 
        self._odrv.axis0.controller.config.control_mode = ControlMode.TORQUE_CONTROL
        self._odrv.axis0.controller.config.input_mode = InputMode.PASSTHROUGH

        # Logging
        self.t0 = perf_counter()
        self.tel_dict = {
            "time": [],
            "pos_actual": [],
            "vel_actual": [],
            "tau_actual": [],
        }

    def change_motor_state(self, state_to_request = AxisState.IDLE):
        # Put in closed loop controol
        if self._odrv.axis0.current_state != state_to_request:
            self._odrv.axis0.controller.input_torque = 0.0
            request_state(self._odrv.axis0, state_to_request)
    
    def assign_torque(self):
        self.mode = FrictionMode.TORQUE
        self._odrv.axis0.controller.config.control_mode = ControlMode.TORQUE_CONTROL
        self.torque_to_use = float(input("Type torque [N/m] to run as a float, and hit enter: "))

        if abs(self.torque_to_use) >= 0.1:
            raise ValueError("Torque way too high!")
    
    def assign_velocity(self):
        self.mode = FrictionMode.VELOCITY
        self._odrv.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
        self.velocity_to_use = float(input("Type VELOCITY [rad/s] to run as a float, and hit enter: "))/REV_TO_RAD

        if abs(self.velocity_to_use) >= 1.0:
            raise ValueError("Velocity way too high!")

    
    def cyclic_control(self):
        #print("Commanding")
        if self.mode == FrictionMode.TORQUE:
            self._odrv.axis0.controller.input_torque = self.torque_to_use
        elif self.mode == FrictionMode.VELOCITY:
            self._odrv.axis0.controller.input_vel = self.velocity_to_use

        # Logging
        if self.t0 == 0.0:
            self.t0 = perf_counter() # On first call, get initial times
        
        #print("Logging, supposedly")
        cur_t = perf_counter() - self.t0 # Log
        (self.tel_dict["time"]).append(cur_t)
        (self.tel_dict["pos_actual"]).append(-1*((self._odrv.axis0.pos_estimate)*REV_TO_RAD))
        (self.tel_dict["vel_actual"]).append(-1*((self._odrv.axis0.vel_estimate)*REV_TO_RAD))
        (self.tel_dict["tau_actual"]).append(((self._odrv.axis0.motor.foc.Iq_measured)*TORQUE_CONSTANT))
    
    def save_plot_log(self, save_dir='sys_id_data'):
        """
        Save telemetry data and generate plots
        
        Args:
            save_dir: Directory to save data and plots
        """
        # Align to min
        for key in self.tel_dict:
            print(f"len({key}) = {len(self.tel_dict[key])}")
    
        min_len = min(len(self.tel_dict[key]) for key in self.tel_dict)
        for key in self.tel_dict:
            self.tel_dict[key] = self.tel_dict[key][:min_len]

        # Convert to numpy arrays for plotting
        t = np.array(self.tel_dict['time'])
        pos = np.array(self.tel_dict['pos_actual'])
        vel = np.array(self.tel_dict['vel_actual'])
        tau_actual = np.array(self.tel_dict['tau_actual'])

        print(f"lenght of tau_actual = {len(tau_actual)}")

        # Create directory if it doesn't exist
        os.makedirs(save_dir, exist_ok=True)
        
        # Generate timestamp for filenames
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Save data as pickle
        smooth_tau_actual = savgol_filter(tau_actual, window_length=11, polyorder=3)
        self.tel_dict["smooth_tau_actual"] = smooth_tau_actual
        data_filename = os.path.join(save_dir, f'friction_data{timestamp}.pkl')
        with open(data_filename, 'wb') as f:
            pickle.dump(self.tel_dict, f)
        print(f"Data saved to: {data_filename}")

        # Create figure with 3 subplots
        fig, axes = plt.subplots(3, 1, figsize=(12, 6), sharex = True)
        
        # Plot 1: Position
        axes[0].plot(t, pos, 'b-', linewidth=1.5, label='Position')
        axes[0].set_ylabel('Position (rad)', fontsize=12)
        axes[0].set_xlabel('Time (s)', fontsize=12)
        axes[0].grid(True, alpha=0.3)
        axes[0].legend(loc='upper right')
        axes[0].set_title('Friction Results', fontsize=14, fontweight='bold')
        
        # Plot 2: Velocity
        axes[1].plot(t, vel, 'g-', linewidth=1.5, label='Velocity')
        axes[1].set_ylabel('Velocity (rad/s)', fontsize=12)
        axes[1].set_xlabel('Time (s)', fontsize=12)
        axes[1].grid(True, alpha=0.3)
        axes[1].legend(loc='upper right')
        
        # Plot 3: Torque (commanded and actual)
        axes[2].plot(t, tau_actual, 'r--', linewidth=1.0, label='Actual')
        axes[2].plot(t, smooth_tau_actual, 'b-', linewidth=2.0, label='Filtered Actual')
        axes[2].set_ylabel('Torque (N·m)', fontsize=12)
        axes[2].set_xlabel('Time (s)', fontsize=12)
        axes[2].grid(True, alpha=0.3)
        axes[2].legend(loc='upper right')
        
        # Save figure
        plot_filename = os.path.join(save_dir, f'friction_plot_{timestamp}.png')
        plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
        print(f"Plot saved to: {plot_filename}")
        
        # Show plot
        plt.show()
            

# ---- MAIN -----
def main():
    odrive_sys_id = FrictionOdrive()
    odrive_sys_id.change_motor_state(AxisState.CLOSED_LOOP_CONTROL)
    #odrive_sys_id.assign_torque()
    odrive_sys_id.assign_velocity()
    try:
        while True:
            odrive_sys_id.cyclic_control()
    except KeyboardInterrupt:
        pass
    finally:
        print("Stopping motor...")
        odrive_sys_id.change_motor_state(AxisState.IDLE)

        print("Logging...")
        odrive_sys_id.save_plot_log()


if __name__ == "__main__":
    main()
