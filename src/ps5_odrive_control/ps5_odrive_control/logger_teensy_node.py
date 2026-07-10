import threading
import serial

import rclpy
from rclpy.node import Node
from msgs_furata.msg import MotorSingleTelemetry

import time
from time import perf_counter
import os
from datetime import datetime
import struct

# Protocol info
# Expect [0xAA] [0x55] [t] [pos] [vel] [iq_set] [iq_measured] [tau_target] [tau_actual] [cmd] [checksum]
PACKET_SIZE = 35  # SINGLE_TEL_PCKT_SIZE

class LoggerNode(Node):
    def __init__(self,
                 port='/dev/ttyACM0',
                 baud=115200,
                 topic='telemetry_teensy',
                 log_path='logs',
                 logTime = False):
        super().__init__("logger")

        # Ros
        self.publisher = self.create_publisher(MotorSingleTelemetry, topic, 10)
        self.create_timer(0.01, self.publish_latest)   # 100 Hz

        # Open serial port to Teensy
        self._buf = bytearray()
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f"Connected to MCU on {port} at {baud} baud")

            # Signal start
            time.sleep(0.2)  # Optional: wait for Teensy reset on serial open
            self.ser.write(b'\x01') # Ready byte
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to MCU to MCU: {e}")
            raise e
        
        # Serial thread
        self.latest_telemetry = None
        self.lock = threading.Lock()
        self.reader_thread = threading.Thread(
            target=self.serial_reader,
            daemon=True)
        self.reader_thread.start()

        # Logging
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
            self.print_N = 60000 # [Roughly every 1 min]
            self.sum_dt = 0
            self.prev_t = perf_counter()
            self.isFirst = True


    def serial_reader(self):
        while rclpy.ok():
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
                        self.get_logger().warn("---Missing start byte. Likely dropped a packet.")

                        break
                    elif start_idx > 0:
                        # Discard before start
                        self._buf = self._buf[start_idx:]
                        self.get_logger().warn("```Clearing before start byte. Likely dropped a packet.")
                    
                    # Extract (protocol dependent)
                    t_ms_bits = self._buf[2:6] 
                    pos_bits = self._buf[6:10]
                    vel_bits = self._buf[10:14]
                    iq_set_bits = self._buf[14:18]
                    iq_act_bits = self._buf[18:22]
                    tau_set_bits = self._buf[22:26]
                    tau_act_bits = self._buf[26:30]
                    cmd_bits = self._buf[30:34]

                    # Compute checksum
                    checksum = self._buf[PACKET_SIZE-1]
                    cs_calc = (0xAA + 0x55 + sum(t_ms_bits) + sum(pos_bits) + sum(vel_bits) + 
                               sum(iq_set_bits) + sum(iq_act_bits) + sum(tau_set_bits) + 
                               sum(tau_act_bits) + sum(cmd_bits)) & 0xFF
                    if cs_calc != checksum:
                        self.get_logger().warn("+++Checksum mismatch, discarding packet")
                        self._buf = self._buf[1:]  # discard first byte and retry
                        continue

                    # Convert payload from bits
                    t_ms = int.from_bytes(t_ms_bits, byteorder='little', signed=False)
                    pos = struct.unpack('<f', pos_bits)[0]
                    vel = struct.unpack('<f', vel_bits)[0]
                    iq_set = struct.unpack('<f', iq_set_bits)[0]
                    iq_act = struct.unpack('<f', iq_act_bits)[0]
                    tau_set = struct.unpack('<f', tau_set_bits)[0]
                    tau_act = struct.unpack('<f', tau_act_bits)[0]
                    cmd = struct.unpack('<f', cmd_bits)[0]

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
                                self.get_logger().info(f"Mean dt btw serial reads, so far = {self.sum_dt/self.N}")
                            self.prev_t = cur_t

            except serial.SerialException as e:
                self.get_logger().error(f"Serial error: {e}")
        
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
    def save_log(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = LoggerNode(
        port='/dev/ttyACM1',   # change to your MCU port
        baud=115200,
        topic='telemetry_teensy',
        logTime = False
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        pass

        # TODO: Implement log saving
        # print("Saving and plotting log...")
        # DATA_DIR = Path.home() / "ws_ros2_odrive" / "src" / "ps5_odrive_control" / "ps5_odrive_control" / "sys_id" / "pend_si_data"
        # DATA_DIR.mkdir(parents=True, exist_ok=True)
        # node.save_plot_log(save_dir=DATA_DIR)

        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
