#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from robojeep_msgs.msg import WheelCommand, SteeringCommand


class RoboJeepBase(Node):
    """
    Converts /cmd_vel (Twist) into per-wheel velocities and a steering angle.

    Subscribes:
      - /cmd_vel (geometry_msgs/Twist)

    Publishes:
      - /wheel_cmd (robojeep_msgs/WheelCommand)
      - /steering_cmd (robojeep_msgs/SteeringCommand)
    """

    def __init__(self):
        super().__init__('robojeep_base')

        # Parameters
        self.declare_parameter('wheel_base', 0.889)           # [m] front-rear axle distance
        self.declare_parameter('track_width', 0.6223)         # [m] left-right distance between wheels
        self.declare_parameter('max_speed', 2.2)              # [m/s] max wheel linear speed
        self.declare_parameter('max_steering_angle', 0.6)     # [rad] ~34 deg

        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.track_width = float(self.get_parameter('track_width').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
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

        self.get_logger().info('RoboJeepBase node initialized.')

    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x       # forward velocity [m/s]
        omega = msg.angular.z  # yaw rate [rad/s]

        # Clip speed
        v = max(-self.max_speed, min(self.max_speed, v))

        # -----------------------------
        # Steering-only mode: v ~ 0
        # -----------------------------
        if abs(v) < 1e-5:
            if abs(omega) > 1e-5:
                # Treat angular.z directly as desired steering angle [rad]
                steering_angle = max(-self.max_steering_angle,
                                     min(self.max_steering_angle, omega))
            else:
                steering_angle = 0.0

            # Wheels stay stopped in steering-only test
            v_fl = v_fr = v_rl = v_rr = 0.0

        # -----------------------------
        # Normal Ackermann mode: v != 0
        # -----------------------------
        else:
            if abs(omega) < 1e-5:
                # Straight line: all wheels same speed, zero steering
                steering_angle = 0.0
                v_fl = v_fr = v_rl = v_rr = v
            else:
                # Turning radius at vehicle center
                R = v / omega

                # Steering angle (virtual center)
                steering_angle = math.atan(self.wheel_base / R)
                steering_angle = max(-self.max_steering_angle,
                                     min(self.max_steering_angle, steering_angle))

                # Left/right radii
                R_left = R - self.track_width / 2.0
                R_right = R + self.track_width / 2.0

                eps = 1e-3
                if abs(R_left) < eps:
                    R_left = eps * (1 if R_left >= 0 else -1)
                if abs(R_right) < eps:
                    R_right = eps * (1 if R_right >= 0 else -1)

                # Scale wheel speeds
                v_fl = v_rl = v * (R_left / R)
                v_fr = v_rr = v * (R_right / R)

        # Clamp wheel speeds individually just in case
        v_fl = max(-self.max_speed, min(self.max_speed, v_fl))
        v_fr = max(-self.max_speed, min(self.max_speed, v_fr))
        v_rl = max(-self.max_speed, min(self.max_speed, v_rl))
        v_rr = max(-self.max_speed, min(self.max_speed, v_rr))

        stamp = self.get_clock().now().to_msg()

        wheel_cmd = WheelCommand()
        wheel_cmd.header.stamp = stamp
        wheel_cmd.front_left_velocity = float(v_fl)
        wheel_cmd.front_right_velocity = float(v_fr)
        wheel_cmd.rear_left_velocity = float(v_rl)
        wheel_cmd.rear_right_velocity = float(v_rr)
        wheel_cmd.enable = True
        self.wheel_cmd_pub.publish(wheel_cmd)

        steering_cmd = SteeringCommand()
        steering_cmd.header.stamp = stamp
        steering_cmd.steering_angle = float(steering_angle)
        steering_cmd.steering_velocity = 0.0
        steering_cmd.enable = True
        self.steering_cmd_pub.publish(steering_cmd)

        # Optional debug
        self.get_logger().debug(
            f"cmd_vel v={v:.2f}, omega={omega:.2f} "
            f"-> v_fl={v_fl:.2f}, v_fr={v_fr:.2f}, v_rl={v_rl:.2f}, v_rr={v_rr:.2f}, "
            f"steer={steering_angle:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = RoboJeepBase()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
