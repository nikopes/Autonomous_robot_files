#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from evdev import InputDevice, ecodes, list_devices

class DirectTeleop(Node):
    def __init__(self):
        super().__init__('direct_teleop')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.publish_twist)  # Faster update rate (20Hz)
        
        # --- CONFIGURATION ---
        self.max_linear_speed = 0.5  # Starting speed (m/s)
        self.max_angular_speed = 1.0 # Starting turn speed (rad/s)
        self.step_size = 0.05        # How much to change speed per click
        
        # Internal State
        self.current_speed = 0.0
        self.current_turn = 0.0
        self.enabled = False
        self.last_hat_x = 0
        self.last_hat_y = 0

        # Connect to Controller
        devices = [InputDevice(path) for path in list_devices()]
        self.gamepad = None
        for dev in devices:
            if any(name in dev.name for name in ["Logitech", "Xbox", "Microsoft", "X-Box"]):
                self.gamepad = dev
                print(f"✅ Connected to: {dev.name}")
                break
        
        if self.gamepad is None:
            print("❌ ERROR: No Controller Found! (Did you run xboxdrv?)")
            exit(1)
            
        print(f"🚀 Ready! Hold 'A' or 'X' to move.")
        print(f"🎮 D-PAD UP/DOWN: Adjust Speed ({self.max_linear_speed:.2f} m/s)")
        print(f"🎮 D-PAD LEFT/RIGHT: Adjust Turn ({self.max_angular_speed:.2f} rad/s)")

    def publish_twist(self):
        try:
            for event in self.gamepad.read():
                # --- 1. ENABLE BUTTON (Hold A or X) ---
                if event.type == ecodes.EV_KEY:
                    if event.code in [304, 307]:  # 304=A, 307=X
                        self.enabled = (event.value == 1)

                # --- 2. ANALOG STICKS (Driving) ---
                if event.type == ecodes.EV_ABS:
                    # Left Stick Up/Down (Linear)
                    if event.code == ecodes.ABS_Y:
                        val = event.value
                        # Normalize: Xbox is -32768 (Up) to 32767 (Down)
                        if abs(val) < 4000: val = 0 # Deadzone
                        # Negative because Up is negative on joystick, but positive for robot
                        self.current_speed = -1.0 * (val / 32768.0) * self.max_linear_speed

                    # Left Stick Left/Right (Angular)
                    if event.code == ecodes.ABS_X:
                        val = event.value
                        if abs(val) < 4000: val = 0
                        self.current_turn = -1.0 * (val / 32768.0) * self.max_angular_speed

                    # --- 3. D-PAD (Speed Adjustment) ---
                    # D-Pad usually reports as ABS_HAT0X (Left/Right) and ABS_HAT0Y (Up/Down)
                    if event.code == ecodes.ABS_HAT0Y: # Up/Down
                        if event.value == -1 and self.last_hat_y != -1: # Up Pressed
                            self.max_linear_speed += self.step_size
                            print(f"🔼 Speed INCREASED: {self.max_linear_speed:.2f} m/s")
                        elif event.value == 1 and self.last_hat_y != 1: # Down Pressed
                            self.max_linear_speed = max(0.1, self.max_linear_speed - self.step_size)
                            print(f"🔽 Speed DECREASED: {self.max_linear_speed:.2f} m/s")
                        self.last_hat_y = event.value

                    if event.code == ecodes.ABS_HAT0X: # Left/Right
                        if event.value == -1 and self.last_hat_x != -1: # Left Pressed
                            self.max_angular_speed += self.step_size
                            print(f"◀️ Turn Speed INCREASED: {self.max_angular_speed:.2f} rad/s")
                        elif event.value == 1 and self.last_hat_x != 1: # Right Pressed
                            self.max_angular_speed = max(0.1, self.max_angular_speed - self.step_size)
                            print(f"▶️ Turn Speed DECREASED: {self.max_angular_speed:.2f} rad/s")
                        self.last_hat_x = event.value

        except BlockingIOError:
            pass
        except OSError:
            print("❌ Controller disconnected!")
            exit(1)

        # Publish Twist Message
        msg = Twist()
        if self.enabled:
            msg.linear.x = float(self.current_speed)
            msg.angular.z = float(self.current_turn)
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DirectTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
