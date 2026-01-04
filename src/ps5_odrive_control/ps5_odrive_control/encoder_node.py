#!/usr/bin/env python3
import rclpy
import struct
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
from time import perf_counter

RAD_PER_PULSE = (2.0 * 3.1415926535)/2400
PACKET_SIZE = 19

class SimpleLowFilter():
    def __init__(self, dt, cutoff):
        self._dt = dt
        self._alpha = np.exp(-2*np.pi*dt*cutoff)

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

class EncoderNode(Node):
    def __init__(self,
                 port='/dev/ttyACM0',
                 baud=115200,
                 topic='encoder_furata',
                 logTime = False):
        """
        ROS2 node to read ASCII encoder values (degrees) from MCU
        and publish as Float32.
        """
        super().__init__('encoder_node')
        self._logTime = logTime

        # Publisher
        self.pub = self.create_publisher(Float32MultiArray, topic, 10)

        # Open serial port encoder_degrees
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f"Connected to MCU on {port} at {baud} baud")
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
            else:
                cur_dt = now - self._last_t

            if (self._printer % 1000 == 0) and (self._logTime):
                self.get_logger().info(f"cur_dt = {cur_dt}")
            self._last_t = now

        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = EncoderNode(
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
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
