#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from robojeep_msgs.msg import WheelCommand, SteeringCommand


class RoboJeepBaseDifferential(Node):
    """
    DUAL STEERING MODE
    
    Right Joystick:
    - Up/down: Throttle (forward/backward)
    - Left/right: Tank steering (differential wheel speeds)
    
    Left Joystick:
    - Left/right: Ackermann steering (wheel angle)
    """

    def __init__(self):
        super().__init__('robojeep_base_differential')

        # Parameters
        self.declare_parameter('max_speed_scale', 1.0)
        self.declare_parameter('track_width', 0.6223)
        self.declare_parameter('tank_turn_sensitivity', 0.5)
        self.declare_parameter('max_steering_angle', 0.6)  # radians (~34 degrees)

        self.max_speed_scale = float(self.get_parameter('max_speed_scale').value)
        self.track_width = float(self.get_parameter('track_width').value)
        self.tank_turn_sensitivity = float(self.get_parameter('tank_turn_sensitivity').value)
        self.max_steering_angle = float(self.get_parameter('max_steering_angle').value)

        # I/O
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10,
        )

        self.wheel_cmd_pub = self.create_publisher(WheelCommand, 'wheel_cmd', 10)
        self.steering_cmd_pub = self.create_publisher(SteeringCommand, 'steering_cmd', 10)

        self.get_logger().info('Dual steering mode initialized.')
        self.get_logger().info('Right stick: throttle + tank turn | Left stick: Ackermann steering')

    def cmd_vel_callback(self, msg: Twist):
        """
        Convert cmd_vel to wheel commands + steering commands
        
        linear.x = throttle (right stick up/down)
        linear.y = tank steering (right stick left/right)
        angular.z = Ackermann steering (left stick left/right)
        """
        # Get raw values
        throttle = msg.linear.x          # Right stick up/down
        tank_steer = msg.linear.y        # Right stick left/right
        ackermann_steer = msg.angular.z  # Left stick left/right
        
        # DEADZONE
        DEADZONE = 0.01
        if abs(throttle) < DEADZONE:
            throttle = 0.0
        if abs(tank_steer) < DEADZONE:
            tank_steer = 0.0
        if abs(ackermann_steer) < DEADZONE:
            ackermann_steer = 0.0
        
        # EXPLICIT STOP when all zero
        if throttle == 0.0 and tank_steer == 0.0:
            stamp = self.get_clock().now().to_msg()
            
            wheel_cmd = WheelCommand()
            wheel_cmd.header.stamp = stamp
            wheel_cmd.front_left_velocity = 0.0
            wheel_cmd.front_right_velocity = 0.0
            wheel_cmd.rear_left_velocity = 0.0
            wheel_cmd.rear_right_velocity = 0.0
            wheel_cmd.enable = False
            self.wheel_cmd_pub.publish(wheel_cmd)
            
            # Still publish steering angle (tracks joystick position)
            steering_cmd = SteeringCommand()
            steering_cmd.header.stamp = stamp
            steering_cmd.steering_angle = ackermann_steer * self.max_steering_angle
            steering_cmd.steering_velocity = 0.0
            steering_cmd.enable = True  # Always enabled for position tracking
            self.steering_cmd_pub.publish(steering_cmd)
            
            return
        
        # Apply speed scaling
        throttle = throttle * self.max_speed_scale
        
        # Tank steering component (differential wheel speeds)
        tank_component = tank_steer * self.tank_turn_sensitivity
        
        # Calculate wheel velocities
        v_left = throttle - tank_component
        v_right = throttle + tank_component
        
        # Clamp to [-1.0, 1.0] range
        v_left = max(-1.0, min(1.0, v_left))
        v_right = max(-1.0, min(1.0, v_right))
        
        # All wheels on same side move together
        v_fl = v_rl = v_left
        v_fr = v_rr = v_right
        
        # Publish wheel commands
        stamp = self.get_clock().now().to_msg()

        wheel_cmd = WheelCommand()
        wheel_cmd.header.stamp = stamp
        wheel_cmd.front_left_velocity = float(v_fl)
        wheel_cmd.front_right_velocity = float(v_fr)
        wheel_cmd.rear_left_velocity = float(v_rl)
        wheel_cmd.rear_right_velocity = float(v_rr)
        wheel_cmd.enable = True
        self.wheel_cmd_pub.publish(wheel_cmd)

        # Ackermann steering (left joystick tracks wheel angle)
        steering_cmd = SteeringCommand()
        steering_cmd.header.stamp = stamp
        steering_cmd.steering_angle = ackermann_steer * self.max_steering_angle
        steering_cmd.steering_velocity = 0.0
        steering_cmd.enable = True
        self.steering_cmd_pub.publish(steering_cmd)

        # Debug
        self.get_logger().debug(
            f"Throttle={throttle:.2f}, Tank={tank_steer:.2f}, Steer={ackermann_steer:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = RoboJeepBaseDifferential()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
