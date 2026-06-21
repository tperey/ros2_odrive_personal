#!/usr/bin/env python3
import rclpy
import struct
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
from time import perf_counter
import time
import os
from datetime import datetime
import pickle
import matplotlib.pyplot as plt
from pathlib import Path

RAD_PER_PULSE = (2.0 * 3.1415926535)/2400
PACKET_SIZE = 19

class SimpleLowFilter():
    def __init__(self, dt, cutoff):
        self._dt = dt
        self._alpha = np.exp(-2*np.pi*dt*cutoff)
        self.tau = 1.0 / (2.0*np.pi*cutoff)

        self._prev_pos = 0.0
        self._prev_velo = 0.0

        self._cutoff = cutoff
    
    def update(self, pos, new_dt = 0.0):

        # Allow dynamic dt
        if new_dt > 0.0:
            self._dt = new_dt
            self._alpha = np.exp(-2*np.pi*self._dt*self._cutoff)

        # Passed in a pos, compute velo
        cur_velo = (pos - self._prev_pos)/self._dt
        new_velo = self._alpha*self._prev_velo + (1.0 - self._alpha)*cur_velo
        self._prev_pos = pos  # Update previous
        self._prev_velo = new_velo
        return new_velo
    
    def var_update(self, pos, dt):
        # If dt varies

        dt = max(dt, 1e-6)  # avoid division by zero

        # Raw velocity
        cur_velo = (pos - self._prev_pos) / dt

        # Compute alpha dynamically
        self._alpha = dt / (self.tau + dt)

        # Filter velocity
        new_velo = self._prev_velo + self._alpha * (cur_velo - self._prev_velo)

        # Save previous values
        self._prev_pos = pos
        self._prev_velo = new_velo

        return new_velo

class AcckfNode(Node):
    def __init__(self,
                 port='/dev/ttyACM0',
                 baud=115200,
                 topic='encoder_furata',
                 logTime = False):
        """
        ROS2 node to read ASCII encoder values (degrees) from MCU
        and publish as Float32.
        """
        super().__init__('acckf_node')
        self._logTime = logTime

        # Logging
        self.declare_parameter('doLog', False)
        self.doLog = self.get_parameter('doLog').value
        if self.doLog:
            self.t = []
            self.pos = []
            self.vel = []
            self.acc = []
            self._first_t = perf_counter()

        # Publisher
        self.pub = self.create_publisher(Float32MultiArray, topic, 10)

        # Open serial port encoder_degrees
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f"Connected to MCU on {port} at {baud} baud")

            # Signal start
            time.sleep(0.2)  # Optional: wait for Teensy reset on serial open
            self.ser.write(b'\x01') # Ready byte
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to MCU to MCU: {e}")
            raise e

        self._buf = bytearray()  # Serial buffer

        # Create a fast timer to poll serial port
        self._dt = 0.001
        self.timer = self.create_timer(self._dt, self.timer_callback)  # 1 KHz polling, same as MCU

        # # Velocity estimations
        # self._vel_filter = SimpleLowFilter(self._dt, cutoff = 4.0)  # Cutoff in [Hz]
        # self._assumed_filter = SimpleLowFilter(self._dt, cutoff = 4.0)  # Cutoff in [Hz]
        self._last_t = None  # Fallback to initial dt
        self._printer = 0

    def timer_callback(self):
        try:
            # Read bytes
            if self.ser.in_waiting:  # Bring in available byte
                self._buf += self.ser.read(self.ser.in_waiting)
            
            # Process new msg
            while len(self._buf) >= PACKET_SIZE:  # If at least PACKET_SIZE bytes, then new msg has arrived

                # Find start
                start_idx = self._buf.find(b'\xAA\x55')

                if start_idx == -1:
                    # No starts found, so clear
                    self._buf.clear()
                    self.get_logger().warn("---Missing start byte")

                    break
                elif start_idx > 0:
                    # Discard before start
                    self._buf = self._buf[start_idx:]
                    self.get_logger().warn("```Clearing before start byte")
                
                # Extract
                payload = self._buf[2:6]
                speedload = self._buf[6:10]
                kfpayload = self._buf[10:14]
                kfspeedload = self._buf[14:18]
                checksum = self._buf[18]

                # Compute checksum
                cs_calc = (0xAA + 0x55 + sum(payload) + sum(speedload) + sum(kfpayload) + sum(kfspeedload)) & 0xFF
                if cs_calc != checksum:
                    self.get_logger().warn("+++Checksum mismatch, discarding packet")
                    self._buf = self._buf[1:]  # discard first byte and retry
                    continue

                # Convert payload to signed 32-bit integer (little-endian)
                val = int.from_bytes(payload, byteorder='little', signed=True)
                val_rad = float(val) * RAD_PER_PULSE  # [rad] conversion
                # self.get_logger().info(f"Raw bytes: {list(self._buf[0:6])}")
                # self.get_logger().info(f"val_rad = {val_rad}")
                spd = int.from_bytes(speedload, byteorder='little', signed=True)
                spd_rad = float(spd) * RAD_PER_PULSE  # [rad] conversion

                spd_rad = struct.unpack('<f', speedload)[0]
                kfval_rad = struct.unpack('<f', kfpayload)[0]
                kfspeed_rad = struct.unpack('<f', kfspeedload)[0]

                # Process ALL position, but only give one velo estimate per loop
                self._buf = self._buf[PACKET_SIZE:]  # Remove packet we just processed
                val_rad_for_velo = val_rad

                # Msg
                msg = Float32MultiArray()
                msg.data = [val_rad, spd_rad, kfval_rad, kfspeed_rad] # velo, assum_vel]  # Zero velocity for now
                self.pub.publish(msg)
                #self.get_logger().info(f"_ deg: {val}")
            
                # Time logging
                self._printer += 1
                now = perf_counter()
                if self._last_t is None:
                    cur_dt = 0.001  # Default
                    self._first_t = now
                else:
                    cur_dt = now - self._last_t

                if (self._printer % 1000 == 0) and (self._logTime):
                    self.get_logger().info(f"cur_dt = {cur_dt}")
                self._last_t = now

                # Data logging
                now = perf_counter()
                if self.doLog:
                    self.t.append(now - self._first_t)
                    self.pos.append(spd_rad)
                    self.vel.append(kfval_rad)
                    self.acc.append(kfspeed_rad)

        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}")
    
    def save_plot_log(self, save_dir='sys_id_data'):
        """
        Save telemetry data and generate plots
        
        Args:
            save_dir: Directory to save data and plots
        """
        if self.doLog:
            # Convert to numpy arrays for plotting
            t = np.array(self.t)
            p_pos = np.array(self.pos)
            p_vel = np.array(self.vel)
            p_acc = np.array(self.acc)

            tel_dict = {
                "time": t,
                "pend_pos": p_pos,
                "pend_vel": p_vel,
                "pend_acc": p_acc,
            }

            # Create directory if it doesn't exist
            os.makedirs(save_dir, exist_ok=True)
            
            # Generate timestamp for filenames
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            
            # Save data as pickle
            data_filename = os.path.join(save_dir, f'pend_sys_id_data_{timestamp}.pkl')
            with open(data_filename, 'wb') as f:
                pickle.dump(tel_dict, f)
            print(f"Data saved to: {data_filename}")
            
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
            plot_filename = os.path.join(save_dir, f'pend_sys_id_plot_{timestamp}.png')
            plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
            print(f"Plot saved to: {plot_filename}")

def main(args=None):
    rclpy.init(args=args)
    node = AcckfNode(
        port='/dev/ttyACM1',   # change to your MCU port
        baud=115200,
        topic='encoder_furata',
        logTime = False
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print("Saving and plotting log...")
        DATA_DIR = Path.home() / "ws_ros2_odrive" / "src" / "ps5_odrive_control" / "ps5_odrive_control" / "sys_id" / "pend_si_data"
        DATA_DIR.mkdir(parents=True, exist_ok=True)
        node.save_plot_log(save_dir=DATA_DIR)

        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
