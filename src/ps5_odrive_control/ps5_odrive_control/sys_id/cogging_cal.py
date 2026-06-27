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

VEL_AMPLITUDE = 0.5 # [rev/s]
V_MAX = 40.0
MAP_LENGTH = 1024

class SysIdState(Enum):
    # For preventing crazy motions when pendulum in weird states
    OFF = 0
    START = 1
    MAX = 2

class SICoggingComp(Node):
    def __init__(self):

        # TODO: params for calibration
        super().__init__('cogging_cal_odrive')

        # Odrive
        print("!!!STARTING ODrive motors...")
        self._odrv = get_Odrive_init(Vmax = V_MAX, Kv = 0.333, ConType = ControlMode.TORQUE_CONTROL,
                                     shouldCalibrate = False, shouldErase = False, shouldReconfig = True)
        print("Odrive prepped :)")
        self._odrv.axis0.controller.config.input_mode = InputMode.PASSTHROUGH
        self._odrv.axis0.controller.config.vel_limit = V_MAX
        self._odrv.axis0.config.motor.current_control_bandwidth = 2000
        self._odrv.clear_errors()  # Clear errors on start

        # Anticogging config
        self._odrv.axis0.controller.config.pos_gain = 200.0
        self._odrv.axis0.controller.config.vel_gain = 0.5
        self._odrv.axis0.config.anticogging.max_torque = 0.16
        self._odrv.axis0.config.anticogging.calib_start_vel = 0.5
        self._odrv.axis0.config.anticogging.calib_end_vel = 0.15
        self._odrv.axis0.config.anticogging.calib_coarse_integrator_gain = 10
        self._odrv.axis0.pos_estimate = self._odrv.onboard_encoder0.raw  # Use absolute reference
        
        # Logging
        self._dt = 0.002
        self.t0 = 0.0
        self.ctrl_timer = self.create_timer(self._dt, self.cyclic_log)

        self.tel_dict = {
            "time": [],
            "pos_actual": [],
            "vel_actual": [],
            "tau_actual": [],
        }

    def change_motor_state(self, state_to_request = AxisState.IDLE):
        if self._odrv.axis0.current_state != state_to_request:
            request_state(self._odrv.axis0, state_to_request)
    
    def cyclic_log(self):

        # Handle time
        if self.t0 == 0.0:
            self.t0 = perf_counter() # On first call, get initial times

        cur_t = perf_counter() - self.t0 # Log

        # Log update
        (self.tel_dict["time"]).append(cur_t)
        (self.tel_dict["pos_actual"]).append(-1*((self._odrv.axis0.pos_estimate)*REV_TO_RAD))
        (self.tel_dict["vel_actual"]).append(-1*((self._odrv.axis0.vel_estimate)*REV_TO_RAD))
        (self.tel_dict["tau_actual"]).append(((self._odrv.axis0.motor.foc.Iq_measured)*TORQUE_CONSTANT))

        # Detect completion
        if self._odrv.axis0.current_state == AxisState.IDLE:
            self.ctrl_timer.cancel()
            self.get_logger().info("Cal complete!")
            raise KeyboardInterrupt

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

        # Create directory if it doesn't exist
        os.makedirs(save_dir, exist_ok=True)
        
        # Generate timestamp for filenames
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Save data as pickle
        data_filename = os.path.join(save_dir, f'cogging_comp_data_{timestamp}.pkl')
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
        axes[0].set_title('System Identification Results', fontsize=14, fontweight='bold')
        
        # Plot 2: Velocity
        axes[1].plot(t, vel, 'g-', linewidth=1.5, label='Velocity')
        axes[1].set_ylabel('Velocity (rad/s)', fontsize=12)
        axes[1].set_xlabel('Time (s)', fontsize=12)
        axes[1].grid(True, alpha=0.3)
        axes[1].legend(loc='upper right')
        
        # Plot 3: Torque (commanded and actual)
        axes[2].plot(t, tau_actual, 'r--', linewidth=1.0, label='Actual')
        axes[2].set_ylabel('Torque (N·m)', fontsize=12)
        axes[2].set_xlabel('Time (s)', fontsize=12)
        axes[2].grid(True, alpha=0.3)
        axes[2].legend(loc='upper right')
        
        # Save figure
        plot_filename = os.path.join(save_dir, f'cogging_comp_telplot_{timestamp}.png')
        plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
        print(f"Plot saved to: {plot_filename}")
        
        # Show plot
        plt.show()

    def save_cogging_map(self, save_dir='sys_id_data'):
        # Get the cogging map
        cogging_map = []
        for i in range(1024):
            cogging_map.append(self._odrv.axis0.config.anticogging.get_map(i))
        self.get_logger().info(f"Cogging map polled")
        cogging_map = np.array(cogging_map)

        # Save data as pickle
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        data_filename = os.path.join(save_dir, f'cogging_map_{timestamp}.pkl')
        with open(data_filename, 'wb') as f:
            pickle.dump(cogging_map, f)
        print(f"Data saved to: {data_filename}")

        # ANOTHER PLOT FOR COGGING MAP
        plt.figure()
        fig, axes = plt.subplots(1, 1, figsize=(12, 6), sharex = True)

        axes.plot(cogging_map, 'b-', linewidth=1.5, label='Cogging Map')
        axes.set_xlabel('Index', fontsize = 12)
        axes.set_ylabel('Torque (N-m)', fontsize = 12)
        axes.grid(True, alpha=0.3)
        axes.legend(loc='upper right')
        
        plt.tight_layout()
        
        # Save figure
        plot_filename = os.path.join(save_dir, f'cogging_map_plot_{timestamp}.png')
        plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
        print(f"Plot saved to: {plot_filename}")
        
        # Show plot
        plt.show()

        # Show the FFT
        plt.figure()

        # FFT
        N = len(cogging_map)
        dx = 1/N
        fft_vals = np.fft.rfft(cogging_map)
        freqs = np.fft.rfftfreq(len(cogging_map), d=dx)
        magnitude = np.abs(fft_vals)

        # Plot
        plt.plot(freqs, magnitude)
        plt.xlabel('Frequency (Hz, i.e. cycles/rev)')
        plt.ylabel('Magnitude')
        plt.grid(True)

        plt.show()
        plot_filename = os.path.join(save_dir, f'cogging_map_FFT_{timestamp}.png')
        plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
        print(f"Plot saved to: {plot_filename}")
        
    
    def shutdown_odrive(self):
        # Stop motor safely
        self.get_logger().info("Stopping ODrive motors...")

        request_state(self._odrv.axis0, AxisState.IDLE)

        time.sleep(1.0)

        # Save the cogging map
        self._odrv.axis0.controller.config.pos_gain = 20.0  # Restore gains
        self._odrv.axis0.controller.config.vel_gain = 0.333
        self.get_logger().info("Reset gains, saving config")
        self._odrv.save_configuration()

# ---- MAIN -----
def main():
    rclpy.init()
    node = SICoggingComp()
    node.change_motor_state(AxisState.ANTICOGGING_CALIBRATION)  # Start the cal
    try:
        rclpy.spin(node)  # Keep node alive and handle callbacks
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node...")
    finally:
        rclpy.shutdown()  # Interferes with accessing Odrive
    
    # Save
    print("Saving and plotting log...")
    DATA_DIR = Path.home() / "ws_ros2_odrive" / "src" / "ps5_odrive_control" / "ps5_odrive_control" / "sys_id" / "cogging_comp_onboard"
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    node.save_plot_log(save_dir=DATA_DIR)
    node.save_cogging_map(save_dir=DATA_DIR)

    print("Stopping motor...")
    node.shutdown_odrive() 
        
    node.destroy_node()

if __name__ == "__main__":
    main()
