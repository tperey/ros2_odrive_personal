import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from inputs import get_gamepad

class PS5ControllerNode(Node):
    def __init__(self):

        # General init
        super().__init__('ps5_controller_node')
        self._joystick_y = 122.0 # Init to middle
        self._prev_js = 122.0 
        self._button_east = False # Init to false (not pressed)

        # Publishers
        self.button_pub = self.create_publisher(Bool, 'button_o', 10)
        self.stick_pub = self.create_publisher(Float32, 'left_stick_y', 10)

        # Timer for publishing
        pub_period = 0.010 # [s]. 0.010 is 100 Hz
        self.pub_timer = self.create_timer(pub_period, self.pub_timer_callback)

        # Start background thread for checking events
        self._lock = threading.Lock() # Protect shared vars
        thread = threading.Thread(target=self.poll_controller)
        thread.daemon = True
        thread.start()
    
    def pub_timer_callback(self):

        # Get last read values but make sure its safe
        with self._lock:
            cur_js = float(self._joystick_y)
            cur_btn = self._button_east

        # Publish joystick
        js_msg = Float32()
        js_msg.data = cur_js
        self.stick_pub.publish(js_msg)

        # Publish o button
        btn_msg = Bool()
        btn_msg.data = cur_btn
        self.button_pub.publish(btn_msg)

        if cur_js != self._prev_js:  # Clip to publish on change
            self.get_logger().info(f"Published joystick: {js_msg.data}, button {btn_msg.data}")
            self._prev_js = cur_js

    # SEPARATE THREAD TRACKING CONTROLLER
    def poll_controller(self):
        while rclpy.ok():
            events = get_gamepad()  # blocks until new ps5 event
            for e in events:
                with self._lock: # Make sure safe to access vars
                    if e.code == "ABS_Y":
                        self._joystick_y = e.state # Store joystick
                    elif e.code == "BTN_EAST":
                        self._button_east = bool(e.state) # Store O button

def main():
    rclpy.init()
    node = PS5ControllerNode()
    try:
        rclpy.spin(node)  # Keep node alive and handle callbacks
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down PS5 Controller node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
