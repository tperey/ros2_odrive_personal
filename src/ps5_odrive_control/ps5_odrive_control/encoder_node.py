#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial

RAD_PER_PULSE = (2.0 * 3.1415926535)/2400

class SimpleLowFilter():
    def __init__(self, dt, cutoff):
        self._dt = dt
        self._alpha = np.exp(-2*np.pi*dt*cutoff)

        self._prev_pos = 0.0
        self._prev_velo = 0.0
    
    def update(self, pos):
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
                 topic='encoder_furata'):
        """
        ROS2 node to read ASCII encoder values (degrees) from Arduino
        and publish as Float32.
        """
        super().__init__('encoder_node')

        # Publisher
        self.pub = self.create_publisher(Float32MultiArray, topic, 10)

        # Open serial port encoder_degrees
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f"Connected to Arduino on {port} at {baud} baud")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            raise e

        self._buf = bytearray()  # Serial buffer

        # Create a fast timer to poll serial port
        self._dt = 0.001
        self.timer = self.create_timer(self._dt, self.timer_callback)  # 100 Hz polling, much slower than Arduino, just takes newest message

        # Velocity estimations
        self._vel_filter = SimpleLowFilter(self._dt, cutoff = 4.0)  # Cutoff in [Hz]

    def timer_callback(self):
        try:

            # Read bytes
            if self.ser.in_waiting:  # Bring in available byte
                self._buf += self.ser.read(self.ser.in_waiting)
            
            # Process new msg
            while len(self._buf) >= 7:  # If at least 7 bytes, then new msg has arrived

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
                checksum = self._buf[6]

                # Compute checksum
                cs_calc = (0xAA + 0x55 + sum(payload)) & 0xFF
                if cs_calc != checksum:
                    self.get_logger().warn("+++Checksum mismatch, discarding packet")
                    self._buf = self._buf[1:]  # discard first byte and retry
                    continue

                # Convert payload to signed 32-bit integer (little-endian)
                val = int.from_bytes(payload, byteorder='little', signed=True)
                val_rad = float(val) * RAD_PER_PULSE  # [rad] conversion
                # self.get_logger().info(f"Raw bytes: {list(self._buf[0:6])}")
                # self.get_logger().info(f"val_rad = {val_rad}")

                # Update velocity
                velo = self._vel_filter.update(val_rad)

                # Msg
                msg = Float32MultiArray()
                msg.data = [val_rad, velo]  # Zero velocity for now
                self.pub.publish(msg)
                #self.get_logger().info(f"_ deg: {val}")

                # Remove processed packet
                self._buf = self._buf[7:]

        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = EncoderNode(
        port='/dev/ttyACM0',   # change to your Arduino port
        baud=115200,
        topic='encoder_furata'
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
