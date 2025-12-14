#!/usr/bin/env python3
"""
Simple WASD Teleop for CREATE3
Control your robot with keyboard keys
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import select

class WASDTeleop(Node):
    def __init__(self):
        super().__init__('wasd_teleop')
        
        # ==========================================
        # TUNABLE PARAMETERS
        # ==========================================
        self.LINEAR_SPEED = 0.3      # m/s - Forward/backward speed
        self.ANGULAR_SPEED = 1.0     # rad/s - Turning speed
        self.LINEAR_INCREMENT = 0.05 # Speed change per key press
        self.ANGULAR_INCREMENT = 0.1 # Turn speed change per key press
        
        # Print tuning info
        print("\n" + "="*60)
        print("WASD TELEOP - TUNABLE PARAMETERS")
        print("="*60)
        print(f"Linear Speed:  {self.LINEAR_SPEED} m/s")
        print(f"Angular Speed: {self.ANGULAR_SPEED} rad/s")
        print(f"Linear Step:   {self.LINEAR_INCREMENT} m/s")
        print(f"Angular Step:  {self.ANGULAR_INCREMENT} rad/s")
        print("="*60 + "\n")
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Current velocities
        self.linear = 0.0
        self.angular = 0.0
        
        # Key mappings
        self.key_bindings = {
            'w': (1, 0),   # Forward
            's': (-1, 0),  # Backward
            'a': (0, 1),   # Turn left
            'd': (0, -1),  # Turn right
            'q': (1, 1),   # Forward + Left
            'e': (1, -1),  # Forward + Right
            'z': (-1, 1),  # Backward + Left
            'c': (-1, -1), # Backward + Right
            ' ': (0, 0),   # Stop (spacebar)
        }
        
        self.speed_bindings = {
            'i': (1, 0),   # Increase linear speed
            'k': (-1, 0),  # Decrease linear speed
            'j': (0, 1),   # Increase angular speed
            'l': (0, -1),  # Decrease angular speed
        }
        
        # Terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.print_instructions()
        
    def print_instructions(self):
        print("""
╔══════════════════════════════════════════════════════════════╗
║                    WASD TELEOP CONTROLS                      ║
╠══════════════════════════════════════════════════════════════╣
║                                                              ║
║  Movement:                    Speed Adjustment:              ║
║                                                              ║
║      Q   W   E                    I - Faster forward        ║
║       \  |  /                     K - Slower forward        ║
║        \ | /                      J - Faster turning        ║
║    A --- + --- D                  L - Slower turning        ║
║        / | \                                                ║
║       /  |  \                 R - Reset to defaults         ║
║      Z   S   C                                              ║
║                               SPACE - Emergency Stop        ║
║  W/S: Forward/Backward        X - Quit                      ║
║  A/D: Turn Left/Right                                       ║
║  Q/E/Z/C: Diagonal moves                                    ║
║                                                              ║
╚══════════════════════════════════════════════════════════════╝
        """)
        
    def get_key(self):
        """Get a single keypress from terminal"""
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def publish_velocity(self):
        """Publish current velocity to robot"""
        twist = Twist()
        twist.linear.x = self.linear
        twist.angular.z = self.angular
        self.cmd_vel_pub.publish(twist)
        
    def update_status(self):
        """Print current status"""
        print(f"\rLinear: {self.linear:+.2f} m/s | Angular: {self.angular:+.2f} rad/s | "
              f"Speed Settings: L={self.LINEAR_SPEED:.2f} A={self.ANGULAR_SPEED:.2f}   ", 
              end='', flush=True)
    
    def run(self):
        """Main control loop"""
        try:
            while True:
                key = self.get_key()
                
                # Movement keys
                if key in self.key_bindings:
                    linear_dir, angular_dir = self.key_bindings[key]
                    self.linear = linear_dir * self.LINEAR_SPEED
                    self.angular = angular_dir * self.ANGULAR_SPEED
                    self.publish_velocity()
                    self.update_status()
                
                # Speed adjustment keys
                elif key in self.speed_bindings:
                    linear_change, angular_change = self.speed_bindings[key]
                    self.LINEAR_SPEED += linear_change * self.LINEAR_INCREMENT
                    self.ANGULAR_SPEED += angular_change * self.ANGULAR_INCREMENT
                    
                    # Clamp speeds to reasonable ranges
                    self.LINEAR_SPEED = max(0.0, min(1.0, self.LINEAR_SPEED))
                    self.ANGULAR_SPEED = max(0.0, min(3.0, self.ANGULAR_SPEED))
                    
                    print(f"\n[SPEED ADJUSTED] Linear: {self.LINEAR_SPEED:.2f} m/s | "
                          f"Angular: {self.ANGULAR_SPEED:.2f} rad/s")
                    self.update_status()
                
                # Reset speeds to defaults
                elif key == 'r':
                    self.LINEAR_SPEED = 0.3
                    self.ANGULAR_SPEED = 1.0
                    self.linear = 0.0
                    self.angular = 0.0
                    self.publish_velocity()
                    print("\n[RESET] Speeds reset to defaults")
                    self.update_status()
                
                # Quit
                elif key == 'x' or key == '\x03':  # x or Ctrl+C
                    print("\n\n[QUIT] Stopping robot...")
                    break
                
                # Unknown key
                else:
                    if key != '\x1b':  # Ignore escape sequences
                        print(f"\n[UNKNOWN KEY] '{key}' - Press a valid key")
                        self.update_status()
        
        except Exception as e:
            print(f"\n\nError: {e}")
        
        finally:
            # Stop robot on exit
            self.linear = 0.0
            self.angular = 0.0
            self.publish_velocity()
            print("\nRobot stopped. Goodbye!")
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    rclpy.init(args=args)
    teleop = WASDTeleop()
    
    try:
        teleop.run()
    except KeyboardInterrupt:
        pass
    finally:
        teleop.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
