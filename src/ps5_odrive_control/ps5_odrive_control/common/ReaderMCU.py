#!/usr/bin/env python3
import struct
import numpy as np
import serial
from time import perf_counter
import time

RAD_PER_PULSE = (2.0 * 3.1415926535)/2400
PACKET_SIZE = 19

# Class for reading MCU serial input.
# Outside of ROS, so can be integrated into a single node
class ReaderMCU():
    def __init__(self,
                 port='/dev/ttyACM0',
                 baud=115200,
                 pack_sz = PACKET_SIZE,
                 logTime = False):
        """
        Class to read pendulum pos and SPEED from MCU
        """
        # Time logging
        self._logTime = logTime
        self._last_t = None  # Fallback to initial dt
        self._printer = 0

        # Open serial port
        self._pack_sz = pack_sz
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            print(f"-----Connected to MCU on {port} at {baud} baud-----")
        except serial.SerialException as e:
            print(f"~~~~~Failed to MCU to MCU: {e}~~~~~")
            raise e

        self._buf = bytearray()  # Serial buffer

        # Pos, Speed (LPF), Pos (KF), Speed (KF)
        self._pend_state = np.array([0.0, 0.0, 0.0, 0.0])

    def _read_pendulum(self):
        # Read serial. Store in member var for getting on request
        try:
            # Read bytes
            if self.ser.in_waiting:  # Bring in available byte
                self._buf += self.ser.read(self.ser.in_waiting)
            
            # Process new msg
            while len(self._buf) >= self._pack_sz:  # If at least PACKET_SIZE bytes, then new msg has arrived

                # Find start
                start_idx = self._buf.find(b'\xAA\x55')

                if start_idx == -1:
                    # No starts found, so clear
                    self._buf.clear()
                    print("---Missing start byte")

                    break
                elif start_idx > 0:
                    # Discard before start
                    self._buf = self._buf[start_idx:]
                    print("```Clearing before start byte")
                
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

                # Convert payload (little-endian)
                val = int.from_bytes(payload, byteorder='little', signed=True)
                val_rad = float(val) * RAD_PER_PULSE  # [rad] conversion
                spd = int.from_bytes(speedload, byteorder='little', signed=True)
                spd_rad = float(spd) * RAD_PER_PULSE  # [rad] conversion

                kfval_rad = struct.unpack('<f', kfpayload)[0]
                kfspeed_rad = struct.unpack('<f', kfspeedload)[0]

                self._buf = self._buf[self._pack_sz:]  # Remove packet we just processed

                # Build state
                self._pend_state = np.array([val_rad, spd_rad, kfval_rad, kfspeed_rad])
            
            # Time logging
            self._printer += 1
            now = perf_counter()
            if self._last_t is None:
                cur_dt = 0.001  # Default
            else:
                cur_dt = now - self._last_t

            if (self._printer % 1000 == 0) and (self._logTime):
                print(f"cur_dt = {cur_dt}")
            self._last_t = now

        except serial.SerialException as e:
            print(f"Serial error: {e}")
    
    def get_pend(self):
        self._read_pendulum() # Update
        return self._pend_state # Return most recent value


""" TEST """
def main(args=None):
    reader = ReaderMCU(
        port='/dev/ttyACM1',   # change to your MCU port
        baud=115200,
        logTime = True
    )
    try:
        print_count = 0
        while True:
            pend_state = reader.get_pend()

            print_count += 1
            if (print_count % 1000 == 0):
                print(f"Pend state = {pend_state}")

            time.sleep(0.001)
    except KeyboardInterrupt as e:
        print(f"Quitting...")

if __name__ == '__main__':
    main()
