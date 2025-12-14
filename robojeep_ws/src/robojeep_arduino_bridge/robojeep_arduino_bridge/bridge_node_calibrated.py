#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import time
from std_msgs.msg import String
from robojeep_msgs.msg import SteeringCommand, WheelCommand, WheelFeedback


class ArduinoBridgeCalibrated(Node):
    """
    Calibrated Arduino Bridge - Motor Control + Encoder Reading
    
    Backwards compatible - works with or without encoder configuration.
    """
    
    def __init__(self):
        super().__init__('arduino_bridge_calibrated')
        
        # ================================
        # EXISTING CALIBRATION PARAMETERS
        # ================================
        self.declare_parameter('front_left_scale', 1.0)
        self.declare_parameter('front_right_scale', 1.0)
        self.declare_parameter('rear_left_scale', 1.0)
        self.declare_parameter('rear_right_scale', 1.0)
        
        self.front_left_scale = float(self.get_parameter('front_left_scale').value)
        self.front_right_scale = float(self.get_parameter('front_right_scale').value)
        self.rear_left_scale = float(self.get_parameter('rear_left_scale').value)
        self.rear_right_scale = float(self.get_parameter('rear_right_scale').value)
        
        # ================================
        # EXISTING SERIAL PORT PARAMETERS
        # ================================
        self.declare_parameter('front_mega_port', '/dev/ttyACM2')
        self.declare_parameter('rear_mega_port', '/dev/ttyACM1')
        self.declare_parameter('baudrate', 115200)
        
        front_mega_port = self.get_parameter('front_mega_port').value
        rear_mega_port = self.get_parameter('rear_mega_port').value
        baud = int(self.get_parameter('baudrate').value)
        
        # ================================
        # NEW: OPTIONAL ENCODER PARAMETERS
        # ================================
        self.declare_parameter('front_micro_port', '')
        self.declare_parameter('rear_micro_port', '')
        self.declare_parameter('encoder_publish_rate', 10.0)
        
        front_micro_port = self.get_parameter('front_micro_port').value
        rear_micro_port = self.get_parameter('rear_micro_port').value
        encoder_rate = float(self.get_parameter('encoder_publish_rate').value)
        
        # ================================
        # SERIAL CONNECTIONS
        # ================================
        # Megas (motor control + left encoders)
        self.front_mega = self.connect_serial(front_mega_port, baud, "FRONT_MEGA")
        self.rear_mega = self.connect_serial(rear_mega_port, baud, "REAR_MEGA")
        
        # Micros (right encoders only) - NEW, OPTIONAL
        self.front_micro = self.connect_serial(front_micro_port, baud, "FRONT_MICRO") if front_micro_port else None
        self.rear_micro = self.connect_serial(rear_micro_port, baud, "REAR_MICRO") if rear_micro_port else None
        
        # ================================
        # ENCODER DATA
        # ================================
        self.encoder_data = {
            'front_left': {'rpm': 0.0, 'ok': False},
            'front_right': {'rpm': 0.0, 'ok': False},
            'rear_left': {'rpm': 0.0, 'ok': False},
            'rear_right': {'rpm': 0.0, 'ok': False},
        }
        
        # ================================
        # ROS SUBSCRIPTIONS (EXISTING)
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
        
        # ================================
        # NEW: ROS PUBLISHERS
        # ================================
        self.wheel_feedback_pub = self.create_publisher(
            WheelFeedback,
            'wheel_feedback',
            10
        )
        
        # FORMATTED STATUS PUBLISHER
        self.serial_status_pub = self.create_publisher(
            String,
            'serial_status',
            10
        )
        
        # Timer to read encoders and publish
        self.create_timer(1.0 / encoder_rate, self.read_and_publish_encoders)
        
        # Timing for reduced logging
        self.last_steering_log = time.time()
        self.last_wheel_log = time.time()
        
        # Track latest commands for formatted display
        self.latest_steer_cmd = "---,---"
        self.latest_front_drive_cmd = "---,---"
        self.latest_rear_drive_cmd = "---,---"
        
        # Timer for formatted serial status
        self.create_timer(0.1, self.publish_serial_status)
        
        self.get_logger().info(
            f"Calibration scales - FL:{self.front_left_scale:.2f}, "
            f"FR:{self.front_right_scale:.2f}, "
            f"RL:{self.rear_left_scale:.2f}, "
            f"RR:{self.rear_right_scale:.2f}"
        )
    
    # ----------------------------------------------------
    # Formatted Serial Status Publisher
    # ----------------------------------------------------
    def publish_serial_status(self):
        """Publish formatted serial command status to topic"""
        status_msg = String()
        status_msg.data = (
            f"FRONT_MEGA: STEER,{self.latest_steer_cmd} | DRIVE,{self.latest_front_drive_cmd}\n"
            f"REAR_MEGA:  DRIVE,{self.latest_rear_drive_cmd}"
        )
        self.serial_status_pub.publish(status_msg)
        
    # ----------------------------------------------------
    # Serial Connection (EXISTING)
    # ----------------------------------------------------
    def connect_serial(self, port, baud, label):
        if not port or port == "":
            self.get_logger().info(f"{label} not configured")
            return None
            
        try:
            ser = serial.Serial(
                port, 
                baud, 
                timeout=0.01,
                write_timeout=0.01
            )
            time.sleep(2.0)
            
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
    # STEERING COMMANDS (EXISTING - PRESERVED)
    # ----------------------------------------------------
    def handle_steering(self, msg: SteeringCommand):
        if not msg.enable:
            return
            
        if self.front_mega is None:
            return
        
        angle_rad = msg.steering_angle
        enable = 1 if msg.enable else 0
        
        # Update display state
        self.latest_steer_cmd = f"{angle_rad:.4f},{enable}"
        
        cmd = f"STEER,{angle_rad:.4f},{enable}\n"
        
        try:
            self.front_mega.write(cmd.encode('utf-8'))
            
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
    # WHEEL COMMANDS (EXISTING - UNCHANGED)
    # ----------------------------------------------------
    def handle_wheel_cmd(self, msg: WheelCommand):
        # Apply calibration scaling and convert to PWM
        fl_pwm = int(-msg.front_left_velocity * self.front_left_scale * 255)
        fr_pwm = int(-msg.front_right_velocity * self.front_right_scale * 255)
        rl_pwm = int(-msg.rear_left_velocity * self.rear_left_scale * 255)
        rr_pwm = int(-msg.rear_right_velocity * self.rear_right_scale * 255)
        
        # Clamp to valid PWM range
        fl_pwm = max(-255, min(255, fl_pwm))
        fr_pwm = max(-255, min(255, fr_pwm))
        rl_pwm = max(-255, min(255, rl_pwm))
        rr_pwm = max(-255, min(255, rr_pwm))
        
        # Update display state
        self.latest_front_drive_cmd = f"{fl_pwm},{fr_pwm}"
        self.latest_rear_drive_cmd = f"{rl_pwm},{rr_pwm}"
        
        # Send to front mega
        if self.front_mega is not None:
            front_cmd = f"DRIVE,{fl_pwm},{fr_pwm}\n"
            
            try:
                self.front_mega.write(front_cmd.encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f"Front mega drive write failed: {e}")
        
        # Send to rear mega
        if self.rear_mega is not None:
            rear_cmd = f"DRIVE,{rl_pwm},{rr_pwm}\n"
            
            try:
                self.rear_mega.write(rear_cmd.encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f"Rear mega drive write failed: {e}")
        
        # Log occasionally
        now = time.time()
        if now - self.last_wheel_log > 0.5:
            self.get_logger().info(
                f"PWM: FL={fl_pwm}, FR={fr_pwm}, RL={rl_pwm}, RR={rr_pwm}"
            )
            self.last_wheel_log = now
    
    # ----------------------------------------------------
    # NEW: ENCODER READING
    # ----------------------------------------------------
    def read_encoder(self, ser):
        """Read encoder data from serial port"""
        if ser is None:
            return None
            
        try:
            while ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                
                # Parse: "ENCODER,rpm,counts" or "ENCODER,rpm,counts,ERR"
                if line.startswith("ENCODER,"):
                    parts = line.split(',')
                    if len(parts) >= 3:
                        try:
                            rpm = float(parts[1])
                            ok = (len(parts) < 4) or (parts[3] != "ERR")
                            return {'rpm': rpm, 'ok': ok}
                        except ValueError:
                            pass
            return None
        except Exception:
            return None
    
    def read_and_publish_encoders(self):
        """Read all encoders and publish WheelFeedback"""
        # Read from Megas (left encoders)
        data = self.read_encoder(self.front_mega)
        if data:
            self.encoder_data['front_left'] = data
            
        data = self.read_encoder(self.rear_mega)
        if data:
            self.encoder_data['rear_left'] = data
        
        # Read from Micros (right encoders) - NEW, OPTIONAL
        if self.front_micro:
            data = self.read_encoder(self.front_micro)
            if data:
                self.encoder_data['front_right'] = data
        
        if self.rear_micro:
            data = self.read_encoder(self.rear_micro)
            if data:
                self.encoder_data['rear_right'] = data
        
        # Publish WheelFeedback
        msg = WheelFeedback()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # RPM values
        msg.front_left_rpm = self.encoder_data['front_left']['rpm']
        msg.front_right_rpm = self.encoder_data['front_right']['rpm']
        msg.rear_left_rpm = self.encoder_data['rear_left']['rpm']
        msg.rear_right_rpm = self.encoder_data['rear_right']['rpm']
        
        # Status flags
        msg.front_left_ok = self.encoder_data['front_left']['ok']
        msg.front_right_ok = self.encoder_data['front_right']['ok']
        msg.rear_left_ok = self.encoder_data['rear_left']['ok']
        msg.rear_right_ok = self.encoder_data['rear_right']['ok']
        
        self.wheel_feedback_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridgeCalibrated()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
