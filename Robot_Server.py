#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import socket
import threading
import time
import math

class RobotServer(Node):
    def __init__(self, host='0.0.0.0', port=5000):
        super().__init__('robot_server')
        
        # ROS 2 Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # State variables for Ramping (Smoothness)
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.current_linear = 0.0
        self.current_angular = 0.0
        
        # Ramping Constants (Adjust these to make it smoother/faster)
        # Lower = Smoother but "sluggish"
        # Higher = More responsive but jerkier
        self.ACCEL_LIMIT_LIN = 0.6  # m/s^2 (Linear acceleration)
        self.ACCEL_LIMIT_ANG = 2.0  # rad/s^2 (Angular acceleration)
        self.DT = 0.1 # Loop time step
        
        self.running = True
        self.client_socket = None
        
        # Socket Server Setup
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((host, port))
        self.server_socket.listen(1)
        
        self.get_logger().info(f"Robot Server listening on port {port}...")

        # Start the movement loop
        self.move_thread = threading.Thread(target=self.movement_loop)
        self.move_thread.daemon = True
        self.move_thread.start()

        # Start the connection loop
        self.server_thread = threading.Thread(target=self.accept_connections)
        self.server_thread.daemon = True
        self.server_thread.start()

    def accept_connections(self):
        while self.running:
            try:
                self.get_logger().info("Waiting for connection...")
                client, addr = self.server_socket.accept()
                self.get_logger().info(f"Connected to {addr}")
                self.client_socket = client
                self.handle_client(client)
            except Exception as e:
                self.get_logger().error(f"Connection error: {e}")

    def handle_client(self, client):
        buffer = ""
        try:
            while self.running:
                data = client.recv(1024).decode('utf-8')
                if not data:
                    break
                
                buffer += data
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    self.process_command(line.strip())
                    
        except Exception:
            pass
        finally:
            self.stop_robot()
            client.close()
            self.client_socket = None
            self.get_logger().info("Client disconnected.")

    def process_command(self, command):
        try:
            parts = command.split(":")
            cmd_type = parts[0]

            if cmd_type == "VELOCITY":
                # Updates the TARGET. The loop handles the smoothing.
                if len(parts) >= 3:
                    self.target_linear = float(parts[1])
                    self.target_angular = float(parts[2])

            elif cmd_type == "STOP":
                self.target_linear = 0.0
                self.target_angular = 0.0
            
            elif cmd_type == "SHOOT_SPIN":
                # Direct spin for shooting - BYPASSES SMOOTHING for immediate max speed
                if len(parts) >= 3:
                    angular_speed = float(parts[1])
                    duration = float(parts[2])
                    self.execute_direct_move(0.0, angular_speed, duration)
            
            elif cmd_type == "NAVIGATE":
                # Navigate to position: NAVIGATE:x_cm:y_cm or NAVIGATE:x_cm:y_cm:heading
                if len(parts) >= 3:
                    x_cm = float(parts[1])
                    y_cm = float(parts[2])
                    heading = float(parts[3]) if len(parts) >= 4 else None
                    self.execute_navigate(x_cm, y_cm, heading)
            
            elif cmd_type == "TURN":
                # Turn degrees: TURN:degrees (positive=left, negative=right)
                if len(parts) >= 2:
                    degrees = float(parts[1])
                    self.execute_turn(degrees)
            
            elif cmd_type == "RESET_NAV":
                # Reset navigation origin
                self.execute_reset_navigation()

            elif cmd_type == "ROTATE_BACK":
                # Rotate immediately (open loop)
                if len(parts) >= 2:
                    degrees = float(parts[1])
                    radians = math.radians(degrees)
                    self.execute_direct_move(0.0, -1.0, abs(radians))

            elif cmd_type == "SPIN_SEARCH":
                self.target_linear = 0.0
                self.target_angular = 0.5 * (float(parts[1]) if len(parts) > 1 else 1)
            
            elif cmd_type == "ALIGN_TO_DOCK":
                 self.target_linear = 0.0
                 self.target_angular = 0.0

        except ValueError:
            pass
    
    def execute_navigate(self, x_cm, y_cm, heading_deg=None):
        """Execute navigation to position using CREATE3 navigate_to command"""
        self.get_logger().info(f"Navigating to ({x_cm}, {y_cm}) cm, heading={heading_deg}°")
        # Note: This is a placeholder - you'll need to implement with actual CREATE3 API
        # For now, we'll simulate with velocity commands
        # In production, use: await robot.navigate_to(x_cm, y_cm, heading_deg)
        pass
    
    def execute_turn(self, degrees):
        """Execute turn using CREATE3 turn command"""
        self.get_logger().info(f"Turning {degrees}°")
        # Note: This is a placeholder - you'll need to implement with actual CREATE3 API
        # For now, we'll simulate
        # In production, use: await robot.turn_left(degrees) or robot.turn_right(abs(degrees))
        pass
    
    def execute_reset_navigation(self):
        """Reset navigation origin using CREATE3 reset_navigation command"""
        self.get_logger().info("Resetting navigation origin")
        # Note: This is a placeholder - you'll need to implement with actual CREATE3 API
        # In production, use: await robot.reset_navigation()
        pass

    def execute_direct_move(self, linear, angular, duration):
        """Bypasses smoothing for specific actions (like shooting)."""
        # Stop smoothing momentarily
        self.target_linear = linear
        self.target_angular = angular
        self.current_linear = linear
        self.current_angular = angular
        
        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        
        # Publish for duration
        start = time.time()
        while time.time() - start < duration:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
            
        self.stop_robot()

    def stop_robot(self):
        self.target_linear = 0.0
        self.target_angular = 0.0

    def ramp_value(self, current, target, limit):
        """Helper to move current towards target by at most limit."""
        step = limit * self.DT
        if abs(target - current) < step:
            return target
        if target > current:
            return current + step
        else:
            return current - step

    def movement_loop(self):
        """Main loop that handles smoothing."""
        while self.running:
            # RAMPING LOGIC
            self.current_linear = self.ramp_value(self.current_linear, self.target_linear, self.ACCEL_LIMIT_LIN)
            self.current_angular = self.ramp_value(self.current_angular, self.target_angular, self.ACCEL_LIMIT_ANG)
            
            twist = Twist()
            twist.linear.x = float(self.current_linear)
            twist.angular.z = float(self.current_angular)
            self.cmd_vel_pub.publish(twist)
            time.sleep(self.DT)

    def destroy_node(self):
        self.running = False
        if self.server_socket:
            self.server_socket.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    robot_server = RobotServer()
    try:
        rclpy.spin(robot_server)
    except KeyboardInterrupt:
        pass
    finally:
        robot_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
