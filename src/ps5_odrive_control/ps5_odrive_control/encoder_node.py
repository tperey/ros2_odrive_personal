#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial

class SimpleLowFilter():
    def __init__(self, dt, cutoff):
        self._alpha = np.exp(-2*np.pi*dt*cutoff)
        self._prev_val = 0.0
    
    def update(self, cur_val):
        new_val = self._alpha*self._prev_val + (1.0 - self._alpha)*cur_val
        self._prev_val = new_val
        return new_val

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

        # Create a fast timer to poll serial port
        self._dt = 0.001
        self.timer = self.create_timer(self._dt, self.timer_callback)  # 100 Hz polling, much slower than Arduino, just takes newest message

        # Velocity estimations
        self._vel_filter = SimpleLowFilter(self._dt, cutoff = 10.0)  # Cutoff in [Hz]

    def timer_callback(self):
        try:
            # Read all available lines
            while self.ser.in_waiting:(val - self._deg_prev)/self._dt
                try:
                    line = self.ser.readline().decode('utf-8').strip()
                    if line:
                        try:
                            val = float(line)

                            # Update velo
                            velo = self._vel_filter.update(val)

                            # Msg
                            msg = Float32MultiArray()
                            msg.data = [val, velo]  # Zero velocity for now
                            self.pub.publish(msg)
                            #self.get_logger().info(f"Encoder deg: {val}")

                            self._deg_prev = val  # Reset prev only if successful
                        except ValueError:
                            # Ignore malformed lines
                            self.get_logger().info("~~~Mishapen msg")
                            continue
                except UnicodeDecodeError:
                    self.get_logger().info("!!!Bad decode")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = EncoderNode(
        port='/dev/ttyACM0',   # change to your Arduino port
        baud=115200,
        topic='encoder_degrees'
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
