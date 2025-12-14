#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import time
from robojeep_msgs.msg import SteeringCommand, WheelCommand

WHEEL_RADIUS = 10.0
WHEEL_DISTANCE = 24.0

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        
        # ================================
        # PARAMETERS
        # ================================
        self.declare_parameter('front_mega_port', '/dev/ttyACM2')
        self.declare_parameter('rear_mega_port', '/dev/ttyACM3')
        self.declare_parameter('baudrate', 115200)
        
        front_port = self.get_parameter('front_mega_port').value
        rear_port  = self.get_parameter('rear_mega_port').value
        baud       = int(self.get_parameter('baudrate').value)
        
        # ================================
        # SERIAL CONNECTIONS
        # ================================
        self.front_mega = self.connect_serial(front_port, baud, "FRONT_MEGA")
        self.rear_mega  = self.connect_serial(rear_port, baud, "REAR_MEGA")
        
        # ================================
        # ROS SUBSCRIPTIONS
        # ================================
        self.create_subscription(
            SteeringCommand,
            'steering_cmd',
            self.handle_steering,
            10
        )
        self.create_subscription(
            WheelCommand,
            'wheel_cmd',
            self.handle_wheel_cmd,
            10
        )
        
        # Timing for reduced logging
        self.last_steering_log = time.time()
        self.last_wheel_log = time.time()
        
    # ----------------------------------------------------
    # Establish a Serial Connection with Arduino
    # ----------------------------------------------------
    def connect_serial(self, port, baud, label):
        try:
            ser = serial.Serial(
                port, 
                baud, 
                timeout=0.01,
                write_timeout=0.01
            )
            time.sleep(2.0)  # Allow Arduino to reboot on USB open
            
            # Wait for ready signal from Arduino
            self.get_logger().info(f"Waiting for {label} to be ready...")
            start = time.time()
            while time.time() - start < 5.0:
                if ser.in_waiting:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if "READY" in line:
                        self.get_logger().info(f"{label} is READY!")
                        break
                time.sleep(0.1)
            
            self.get_logger().info(f"{label} connected on {port} @ {baud}")
            return ser
        except Exception as e:
            self.get_logger().error(f"Failed to open {label} port {port}: {e}")
            return None
    
    # ----------------------------------------------------
    # STEERING COMMANDS (to Front Mega only)
    # ----------------------------------------------------
    def handle_steering(self, msg: SteeringCommand):
        if self.front_mega is None:
            self.get_logger().warn("No front mega connection; dropping steering command")
            return
        
        angle_rad = msg.steering_angle
        enable = 1 if msg.enable else 0
        
        # Format: "STEER,<angle_rad>,<enable>\n"
        cmd = f"STEER,{angle_rad:.4f},{enable}\n"
        
        try:
            self.front_mega.write(cmd.encode('utf-8'))
            
            # Log occasionally
            now = time.time()
            if now - self.last_steering_log > 0.5:
                angle_deg = angle_rad * 57.2958
                self.get_logger().info(
                    f"Steering: {angle_rad:.3f} rad ({angle_deg:.1f}°) | Enable: {enable}"
                )
                self.last_steering_log = now
                
        except Exception as e:
            self.get_logger().error(f"Front mega steering write failed: {e}")
    
    # ----------------------------------------------------
    # WHEEL COMMANDS (to Front and Rear Megas)
    # ----------------------------------------------------
    def handle_wheel_cmd(self, msg: WheelCommand):
        # Send to front mega
        if self.front_mega is not None:
            front_cmd = f"DRIVE,{msg.front_left_velocity:.3f},{msg.front_right_velocity:.3f}\n"
            try:
                self.front_mega.write(front_cmd.encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f"Front mega drive write failed: {e}")
        
        # Send to rear mega
        if self.rear_mega is not None:
            rear_cmd = f"DRIVE,{msg.rear_left_velocity:.3f},{msg.rear_right_velocity:.3f}\n"
            try:
                self.rear_mega.write(rear_cmd.encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f"Rear mega drive write failed: {e}")
        
        # Log occasionally
        now = time.time()
        if now - self.last_wheel_log > 0.5:
            self.get_logger().info(
                f"Wheels: FL={msg.front_left_velocity:.2f}, FR={msg.front_right_velocity:.2f}, "
                f"RL={msg.rear_left_velocity:.2f}, RR={msg.rear_right_velocity:.2f} m/s"
            )
            self.last_wheel_log = now

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == "__main__":
    main()
