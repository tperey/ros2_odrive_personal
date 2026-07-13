import threading
import serial
import serial.tools.list_ports

import rclpy
from rclpy.node import Node
from msgs_furata.msg import MotorSingleTelemetry

import time
from time import perf_counter
import os
from datetime import datetime
import struct
import numpy as np
import pickle
import matplotlib.pyplot as plt
from pathlib import Path

# Protocol info
TEENSY_VID = 0x16C0
TEENSY_PID = 0x0483

# Expect [0xAA] [0x55] [t] [pos] [vel] [iq_set] [iq_measured] [tau_target] [tau_actual] [cmd] [checksum]
PACKET_SIZE = 35  # SINGLE_TEL_PCKT_SIZE

BUF_SIZE_WARNING = 525  # Behind by 3x cycles of 5 packets (of 35 bits)


class LoggerNode(Node):
    def __init__(self,
                 baud=115200,
                 topic='telemetry_teensy',
                 log_path='logs',
                 logTime = False):
        super().__init__("logger")

        # Ros
        self.publisher = self.create_publisher(MotorSingleTelemetry, topic, 10)
        self.create_timer(0.01, self.publish_latest)   # 100 Hz

        # Open serial port to Teensy
        self._read_buf = bytearray()  # Separate array, for faster reads
        self._started = False
        try:
            port = self.find_teensy()
            self.ser = serial.Serial(port, baud, timeout=0)
            self.get_logger().info(f"Connected to MCU on {port} at {baud} baud")

            # Signal start
            time.sleep(0.2)  # Optional: wait for Teensy reset on serial open
            while not self._started:
                if self.ser.in_waiting:
                    cur_byte = self.ser.read(1)  # Just look at a byte

                    if cur_byte == b'\x10':
                        self._started = True
                        self.ser.reset_input_buffer()
                else:
                    self.ser.write(b'\x01') # Keep sending ready byte until get ack
                    time.sleep(0.2)
                    self.get_logger().info("Waiting for TEENSY ack...")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to MCU: {e}")
            raise e
        
        # Serial thread
        self.latest_telemetry = None
        self.lock = threading.Lock()
        self.reader_thread = threading.Thread(
            target=self.serial_reader,
            daemon=True)
        self.reader_thread.start()

        # Logging
        self._buf = bytearray()
        self.create_timer(0.005, self._process_serial_buffer)  # Timer for parsing read serial data
        self.log_dict = {
            'time': [],
            'pos': [],
            'vel': [],
            'iq_set': [],
            'iq_act': [],
            'tau_set': [],
            'tau_act': [],
            'cmd': [],
        }

        self._logTime = logTime
        if self._logTime:
            self.N = 0
            self.print_N = 1000 # [Roughly every 1 sec]
            self.sum_dt = 0
            self.prev_t = perf_counter()
            self.isFirst = True

    def find_teensy(self):
        ports = serial.tools.list_ports.comports()

        for port in ports:
            if port.vid == TEENSY_VID and port.pid == TEENSY_PID:
                print(f"Found Teensy: {port.device}")
                return port.device

        raise RuntimeError("Teensy not found")

    def serial_reader(self):
        while rclpy.ok():
            try:
                # Constantly grab bytes as soon as they are available.
                # Store them for separate parsing
                n = self.ser.in_waiting
                if n:
                    data = self.ser.read(n)
                    with self.lock:
                        self._read_buf.extend(data)

                        if len(self._read_buf) > 500:
                            self.get_logger().warn(f"WARNING: read_buf has {len(self._read_buf)} bytes")
            except serial.SerialException as e:
                self.get_logger().error(f"Serial error: {e}")
    
    def _process_serial_buffer(self):
        try:
            # Transfer and clear read buffer
            with self.lock:
                self._buf.extend(self._read_buf)
                self._read_buf.clear()

            # Process new msgs
            while len(self._buf) >= PACKET_SIZE:  # If at least PACKET_SIZE bytes, then there are still msgs to process

                # Find start
                start_idx = self._buf.find(b'\xAA\x55')

                if start_idx == -1:
                    # No starts found, so clear
                    self._buf.clear()
                    self.get_logger().warn("---Missing start byte. Likely dropped a packet.")

                    break
                elif start_idx > 0:
                    # Discard before start
                    self._buf = self._buf[start_idx:]
                    self.get_logger().warn("```Clearing before start byte. Likely dropped a packet.")
                
                if len(self._buf) < PACKET_SIZE:
                    self.get_logger().warn("          Breaking after broken packet")
                    break  # wait for more bytes to arrive before parsing further

                # Extract (protocol dependent)
                t_ms_bits = self._buf[2:6] 
                pos_bits = self._buf[6:10]
                vel_bits = self._buf[10:14]
                iq_set_bits = self._buf[14:18]
                iq_act_bits = self._buf[18:22]
                tau_set_bits = self._buf[22:26]
                tau_act_bits = self._buf[26:30]
                cmd_bits = self._buf[30:34]

                # Convert payload from bits
                t_ms = int.from_bytes(t_ms_bits, byteorder='little', signed=False)
                pos = struct.unpack('<f', pos_bits)[0]
                vel = struct.unpack('<f', vel_bits)[0]
                iq_set = struct.unpack('<f', iq_set_bits)[0]
                iq_act = struct.unpack('<f', iq_act_bits)[0]
                tau_set = struct.unpack('<f', tau_set_bits)[0]
                tau_act = struct.unpack('<f', tau_act_bits)[0]
                cmd = struct.unpack('<f', cmd_bits)[0]

                # Compute checksum
                checksum = self._buf[PACKET_SIZE-1]
                cs_calc = (0xAA + 0x55 + sum(t_ms_bits) + sum(pos_bits) + sum(vel_bits) + 
                            sum(iq_set_bits) + sum(iq_act_bits) + sum(tau_set_bits) + 
                            sum(tau_act_bits) + sum(cmd_bits)) & 0xFF
                if cs_calc != checksum:
                    self.get_logger().warn("+++Checksum mismatch, discarding packet")
                    self._buf = self._buf[1:]  # discard first byte and retry
                    continue

                self._buf = self._buf[PACKET_SIZE:]  # Remove packet we just processed

                # Update latest
                cur_telemetry = [t_ms, pos, vel, iq_set, iq_act, tau_set, tau_act, cmd]
                with self.lock:
                    self.latest_telemetry = cur_telemetry

                # Logging
                for (dict_key, telemetry_entry) in zip(self.log_dict.keys(), cur_telemetry):
                    (self.log_dict[dict_key]).append(telemetry_entry)

            if self._logTime:
                if self.isFirst:
                    self.prev_t = perf_counter() # No time on first. Just initialize prev_t
                    self.isFirst = False
                else:
                    self.N += 1
                    cur_t = perf_counter()
                    self.sum_dt += cur_t - self.prev_t
                    if self.N % self.print_N == 0:
                        self.get_logger().info(f"Mean dt btw serial processing, so far = {self.sum_dt/self.N}")
                    self.prev_t = cur_t

        except serial.SerialException as e:
            self.get_logger().error(f"Buffer parsing error: {e}")
        
    def publish_latest(self):
        with self.lock:
            if self.latest_telemetry is None:
                return
            value = self.latest_telemetry.copy()

        # Publish for visualization every 10 ms
        msg = MotorSingleTelemetry()
        msg.time = float(value[0])
        msg.pos = value[1]
        msg.vel = value[2]
        msg.iq_set = value[3]
        msg.iq_act = value[4]
        msg.tau_set = value[5]
        msg.tau_act = value[6]
        msg.cmd = value[7]

        self.publisher.publish(msg)

    # TODO: implement log saving
    def save_plot_log(self, save_dir='logs'):
        """
        Save telemetry data and generate plots
        
        Args:
            save_dir: Directory to save data and plots
        """
        # Convert to numpy arrays for plotting
        t = np.array(self.log_dict['time'])
        t = (t - t[0])/1000  # Convert from ms to s, and subtract start
        pos = np.array(self.log_dict['pos'])
        vel = np.array(self.log_dict['vel'])
        iq_set = np.array(self.log_dict['iq_set'])
        iq_act = np.array(self.log_dict['iq_act'])
        tau_set = np.array(self.log_dict['tau_set'])
        tau_act = np.array(self.log_dict['tau_act'])
        cmd = np.array(self.log_dict['cmd'])

        # Create directory if it doesn't exist
        os.makedirs(save_dir, exist_ok=True)
        
        # Generate timestamp for filenames
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Save data as pickle
        data_filename = os.path.join(save_dir, f'logs_{timestamp}.pkl')
        with open(data_filename, 'wb') as f:
            pickle.dump(self.log_dict, f)
        print(f"Data saved to: {data_filename}")


        """ POSITION, VELOCITY, AND COMMAND """
        # Create figure with subplots
        fig, axes = plt.subplots(2, 1, figsize=(12, 6), sharex = True)
        
        # Plot 1: Position
        axes[0].plot(t, pos, 'b-', linewidth=1.5, label='Position')
        axes[0].plot(t, cmd, 'k--', linewidth=1.5, label='Position')
        axes[0].set_ylabel('Position (rad)', fontsize=12)
        axes[0].set_xlabel('Time (s)', fontsize=12)
        axes[0].grid(True, alpha=0.3)
        axes[0].legend(loc='upper right')
        axes[0].set_title('Logs', fontsize=14, fontweight='bold')
        
        # Plot 2: Velocity
        axes[1].plot(t, vel, 'g-', linewidth=1.5, label='Velocity')
        axes[1].set_ylabel('Velocity (rad/s)', fontsize=12)
        axes[1].set_xlabel('Time (s)', fontsize=12)
        axes[1].grid(True, alpha=0.3)
        axes[1].legend(loc='upper right')
        
        # Save figure
        plot_filename = os.path.join(save_dir, f'log_pvcplot_{timestamp}.png')
        plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
        print(f"Plot saved to: {plot_filename}")
        
        # Show plot
        plt.show()

        """ TORQUE AND CURRENT """
        # Create figure with subplots
        fig, axes = plt.subplots(2, 1, figsize=(12, 6), sharex = True)
        
        # Plot 1: Currents
        axes[0].plot(t, iq_set, 'p-', linewidth=1.5, label='iq_set')
        axes[0].plot(t, iq_act, 'r--', linewidth=1.5, label='iq_act')
        axes[0].set_ylabel('Position (rad)', fontsize=12)
        axes[0].set_xlabel('Time (s)', fontsize=12)
        axes[0].grid(True, alpha=0.3)
        axes[0].legend(loc='upper right')
        axes[0].set_title('Logs', fontsize=14, fontweight='bold')
        
        # Plot 2: Tau
        axes[1].plot(t, tau_set, 'g-', linewidth=1.5, label='tau_set')
        axes[1].plot(t, tau_act, 'b--', linewidth=1.5, label='tau_act')
        axes[1].set_ylabel('Velocity (rad/s)', fontsize=12)
        axes[1].set_xlabel('Time (s)', fontsize=12)
        axes[1].grid(True, alpha=0.3)
        axes[1].legend(loc='upper right')
        
        # Save figure
        plot_filename = os.path.join(save_dir, f'log_ctplot_{timestamp}.png')
        plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
        print(f"Plot saved to: {plot_filename}")
        
        # Show plot
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = LoggerNode(
        baud=115200,
        topic='telemetry_teensy',
        logTime = True
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Check on timing stats
        times = node.log_dict["time"]
        dts = np.diff(np.array(times))
        print(f"FINAL MEAN LOG DT (ms) = {np.mean(dts)}")
    finally:
        print("Saving and plotting log...")
        DATA_DIR = Path.home() / "ws_ros2_odrive" / "src" / "ps5_odrive_control" / "ps5_odrive_control" / "logs"
        DATA_DIR.mkdir(parents=True, exist_ok=True)
        node.save_plot_log(save_dir=DATA_DIR)

        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
